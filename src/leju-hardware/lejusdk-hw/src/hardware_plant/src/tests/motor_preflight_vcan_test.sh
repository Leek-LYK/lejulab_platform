#!/usr/bin/env bash
# motor_preflight vcan 冒烟测试 (手动执行, 需要 sudo 加载 vcan 模块)
#
# 场景:
#   A. 固件匹配 + 参数写入回读     → 预检通过 (exit 0)
#   B. 固件版本不匹配              → 仅黄色告警, 参数正常 → 放行 (exit 0)
#   C. 电机无响应                  → 预检失败 (exit 1)
#   D. 参数写不生效(回读不一致)    → 预检失败 (exit 1)
#
# 用法: sudo bash motor_preflight_vcan_test.sh [motor_preflight_check 路径]
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../../../../.." && pwd)"
CLI="${1:-$REPO_ROOT/build_cmake/src/leju-hardware/lejusdk-hw/tools/motor_preflight/motor_preflight_check}"
RESPONDER="$SCRIPT_DIR/motor_preflight_vcan_responder.py"
IFACE=vcan_pft
TMPDIR_T=$(mktemp -d /tmp/motor_preflight_test.XXXXXX)
RESPONDER_PID=""
PASS=0
FAIL=0

cleanup() {
    [ -n "$RESPONDER_PID" ] && kill "$RESPONDER_PID" 2>/dev/null
    ip link del "$IFACE" 2>/dev/null
    rm -rf "$TMPDIR_T"
}
trap cleanup EXIT

[ -x "$CLI" ] || { echo "motor_preflight_check 不存在: $CLI (先构建该目标)"; exit 2; }

# ===== vcan 接口 =====
modprobe vcan || { echo "modprobe vcan 失败 (需要 sudo)"; exit 2; }
ip link del "$IFACE" 2>/dev/null
ip link add dev "$IFACE" type vcan && ip link set up "$IFACE" || exit 2

# ===== 测试配置 =====
cat > "$TMPDIR_T/canbus.yaml" << EOF
canbus_interfaces:
  $IFACE:
    type: SOCKETCAN
    name: "$IFACE"
    nbitrate: 1000000
    dbitrate: 1000000
    nsamplepos: 80
    dsamplepos: 80
    enable: true
${IFACE}_devices:
  - name: Test_joint_01
    class: motor
    device_id: 0x01
    params: [0, 200, 50, 0, 0, 0, 0]
    ratio: 25
    negtive: false
    ignore: false
  - name: Test_joint_02
    class: motor
    device_id: 0x02
    params: [0, 200, 50, 0, 0, 0, 0]
    ratio: 25
    negtive: false
    ignore: false
EOF

cat > "$TMPDIR_T/firmware.json" << 'EOF'
{
  "models": {
    "test_motor": {
      "firmware_version": "0x02030001",
      "params": [
        { "idx": 61, "dtype": "uint32", "value": 100 },
        { "idx": 30, "dtype": "float",  "value": 1.5 }
      ]
    }
  },
  "device_map": {
    "Test_joint_01": "test_motor",
    "Test_joint_02": "test_motor"
  }
}
EOF

run_scenario() {
    local desc="$1" expect="$2"; shift 2
    RESPONDER_PID=""
    if [ $# -gt 0 ]; then
        python3 "$RESPONDER" "$IFACE" "$@" &
        RESPONDER_PID=$!
        sleep 0.3
    fi
    "$CLI" "$TMPDIR_T/firmware.json" "$TMPDIR_T/canbus.yaml"
    local rc=$?
    [ -n "$RESPONDER_PID" ] && kill "$RESPONDER_PID" 2>/dev/null && wait "$RESPONDER_PID" 2>/dev/null
    RESPONDER_PID=""
    if [ "$rc" -eq "$expect" ]; then
        echo "=== [PASS] $desc (exit=$rc)"; PASS=$((PASS+1))
    else
        echo "=== [FAIL] $desc (exit=$rc, 期望 $expect)"; FAIL=$((FAIL+1))
    fi
}

echo "===== 场景 A: 固件匹配 + 参数写入回读 ====="
run_scenario "全部通过" 0 1:0x02030001 2:0x02030001

echo "===== 场景 B: M2 固件版本不匹配 (仅告警, 应放行) ====="
run_scenario "固件不匹配仅告警放行" 0 1:0x02030001 2:0x01010101

echo "===== 场景 C: M2 无响应 ====="
run_scenario "无响应拒绝" 1 1:0x02030001

echo "===== 场景 D: 参数写不生效 (回读不一致) ====="
run_scenario "参数校验失败拒绝" 1 --ignore-writes 1:0x02030001 2:0x02030001

echo
echo "结果: $PASS 通过, $FAIL 失败"
[ "$FAIL" -eq 0 ]
