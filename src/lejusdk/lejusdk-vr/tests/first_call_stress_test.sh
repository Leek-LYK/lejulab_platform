#!/bin/bash
# 首次调用压力测试
# 测试 setArmMode RPC 的首次调用延迟（通过多次启动程序）
#
# Usage: ./first_call_stress_test.sh [count] [--visualize]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

source "$WS_DIR/devel/setup.bash"
export CYCLONEDDS_URI=file://$WS_DIR/src/leju_launch/config/cyclonedds_shm.xml

COUNT=${1:-100}
VISUALIZE=false
CSV_FILE="/tmp/first_call_latency.csv"

# 解析参数
for arg in "$@"; do
    case $arg in
        --visualize)
            VISUALIZE=true
            ;;
        [0-9]*)
            COUNT=$arg
            ;;
    esac
done

echo "=== First Call Stress Test ==="
echo "Test count: $COUNT"
echo "Output: $CSV_FILE"
echo ""

# 获取测试程序路径
TEST_BIN="$WS_DIR/devel/lib/lejusdk-vr/set_mode_stress_test"
echo "Test binary: $TEST_BIN"

# 启用 core dump
ulimit -c unlimited
sudo bash -c 'echo /tmp/core.%e.%p > /proc/sys/kernel/core_pattern'
echo "Core dump enabled: /tmp/core.%e.%p"
echo ""

# 初始化 CSV
echo "iteration,latency_us,status" > $CSV_FILE

SUCCESS_COUNT=0
TIMEOUT_COUNT=0
FAIL_COUNT=0
CRASH_COUNT=0

for i in $(seq 1 $COUNT); do
    # 每次启动新进程，只调用一次
    OUTPUT=$(rosrun lejusdk-vr set_mode_stress_test --count 1 --interval 0 2>&1)
    EXIT_CODE=$?

    if echo "$OUTPUT" | grep -qF -- "-> OK"; then
        LATENCY=$(echo "$OUTPUT" | grep -F -- "-> OK" | grep -oE "latency=[0-9.]+" | sed 's/latency=//')
        echo "$i,$LATENCY,OK" >> $CSV_FILE
        echo "[$i/$COUNT] OK  latency=${LATENCY}us"
        ((SUCCESS_COUNT++))
    elif echo "$OUTPUT" | grep -qF "TIMEOUT"; then
        LATENCY=$(echo "$OUTPUT" | grep -F "TIMEOUT" | grep -oE "latency=[0-9.]+" | sed 's/latency=//')
        echo "$i,$LATENCY,TIMEOUT" >> $CSV_FILE
        echo "[$i/$COUNT] TIMEOUT  latency=${LATENCY}ms"
        # 显示诊断信息（匹配 failed 或 not available）
        DIAG=$(echo "$OUTPUT" | grep -E "\[VRCommunication\].*(failed|not available)" | head -1)
        if [ -n "$DIAG" ]; then
            echo "         $DIAG"
        fi
        # 显示 Requester 诊断信息
        REQ_DIAG=$(echo "$OUTPUT" | grep -F "[Requester]" | head -1)
        if [ -n "$REQ_DIAG" ]; then
            echo "         $REQ_DIAG"
        fi
        ((TIMEOUT_COUNT++))
    elif echo "$OUTPUT" | grep -qF "FAIL"; then
        LATENCY=$(echo "$OUTPUT" | grep -F "FAIL" | grep -oE "latency=[0-9.]+" | sed 's/latency=//')
        echo "$i,$LATENCY,FAIL" >> $CSV_FILE
        echo "[$i/$COUNT] FAIL  latency=${LATENCY}ms"
        # 显示诊断信息（匹配 failed 或 not available）
        DIAG=$(echo "$OUTPUT" | grep -E "\[VRCommunication\].*(failed|not available)" | head -1)
        if [ -n "$DIAG" ]; then
            echo "         $DIAG"
        fi
        ((FAIL_COUNT++))
    else
        # 检查是否是崩溃（信号导致的退出）
        # 128+signal: 128+11=139 (SIGSEGV), 128+6=134 (SIGABRT)
        if [ $EXIT_CODE -gt 128 ]; then
            SIGNAL=$((EXIT_CODE - 128))
            echo "$i,0,CRASH" >> $CSV_FILE
            echo ""
            echo "=========================================="
            echo "[$i/$COUNT] CRASH detected! (signal $SIGNAL)"
            echo "=========================================="
            echo ""
            echo "Exit code: $EXIT_CODE"
            case $SIGNAL in
                11) echo "Signal: SIGSEGV (Segmentation fault)" ;;
                6)  echo "Signal: SIGABRT (Aborted)" ;;
                8)  echo "Signal: SIGFPE (Floating point exception)" ;;
                4)  echo "Signal: SIGILL (Illegal instruction)" ;;
                *)  echo "Signal: $SIGNAL" ;;
            esac
            echo ""
            echo "Raw output:"
            echo "$OUTPUT" | sed 's/^/    /'
            echo ""

            # 查找 core dump 文件
            echo "Looking for core dump..."
            sleep 1  # 等待 core dump 写入完成

            CORE_FILE=""
            # 检查常见的 core dump 位置
            # 1. 当前目录
            if [ -f "core" ]; then
                CORE_FILE="core"
            fi
            # 2. /tmp/core.* (推荐的 pattern: /tmp/core.%e.%p)
            if [ -z "$CORE_FILE" ]; then
                CORE_FILE=$(ls -t /tmp/core.* 2>/dev/null | head -1)
            fi
            # 3. 当前目录的 core.*
            if [ -z "$CORE_FILE" ]; then
                CORE_FILE=$(ls -t core.* 2>/dev/null | head -1)
            fi

            if [ -n "$CORE_FILE" ] && [ -f "$CORE_FILE" ]; then
                echo "Found core dump: $CORE_FILE"
                echo ""
                echo "=== Backtrace ==="
                gdb -batch -ex "bt full" -ex "info registers" -ex "quit" "$TEST_BIN" "$CORE_FILE" 2>/dev/null
            else
                # 尝试使用 systemd-coredump
                if command -v coredumpctl &>/dev/null; then
                    echo "No core file found. Trying coredumpctl..."
                    echo ""
                    echo "=== coredumpctl info ==="
                    coredumpctl info -1 2>/dev/null
                    echo ""
                    echo "=== Backtrace ==="
                    coredumpctl gdb -1 --batch -ex "bt full" -ex "quit" 2>/dev/null
                else
                    echo "No core dump found."
                    echo "To enable: sudo bash -c 'echo /tmp/core.%e.%p > /proc/sys/kernel/core_pattern'"
                fi
            fi

            ((CRASH_COUNT++))
            echo ""
            echo "Stopping test due to crash."
            break
        else
            echo "$i,0,UNKNOWN" >> $CSV_FILE
            echo "[$i/$COUNT] UNKNOWN (exit_code=$EXIT_CODE)"
            # 输出原始内容用于调试
            echo "         Raw output:"
            echo "$OUTPUT" | head -20 | sed 's/^/         /'
            ((FAIL_COUNT++))
        fi
    fi
done

TOTAL_RUN=$((SUCCESS_COUNT + TIMEOUT_COUNT + FAIL_COUNT + CRASH_COUNT))
echo ""
echo "=== Results ==="
echo "Total:   $TOTAL_RUN / $COUNT"
echo "Success: $SUCCESS_COUNT ($(echo "scale=1; $SUCCESS_COUNT * 100 / $TOTAL_RUN" | bc)%)"
echo "Timeout: $TIMEOUT_COUNT ($(echo "scale=1; $TIMEOUT_COUNT * 100 / $TOTAL_RUN" | bc)%)"
echo "Failed:  $FAIL_COUNT"
echo "Crash:   $CRASH_COUNT"
echo ""
echo "Data saved to: $CSV_FILE"

# 统计和可视化
if [ "$VISUALIZE" = true ]; then
    python3 << 'PYEOF'
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/first_call_latency.csv')
lat = df['latency_us']
ok_lat = df[df['status'] == 'OK']['latency_us']

print("")
print("=== Latency Statistics (All) ===")
print(f"Count:   {len(lat)}")
print(f"Min:     {lat.min():.0f} us")
print(f"Max:     {lat.max():.0f} us")
print(f"Mean:    {lat.mean():.0f} us")
print(f"Median:  {lat.median():.0f} us")
print(f"Std:     {lat.std():.0f} us")
print(f"P90:     {lat.quantile(0.90):.0f} us")
print(f"P99:     {lat.quantile(0.99):.0f} us")

if len(ok_lat) > 0:
    # 1000us = 1ms, normal is < 1000us (1ms)
    normal_lat = ok_lat[ok_lat < 1000]
    retry_lat = ok_lat[ok_lat >= 1000]
    print("")
    print("=== First Call Analysis ===")
    print(f"Normal (<1ms):  {len(normal_lat)} ({100*len(normal_lat)/len(ok_lat):.1f}%)")
    print(f"Slow (>=1ms):   {len(retry_lat)} ({100*len(retry_lat)/len(ok_lat):.1f}%)")
    if len(normal_lat) > 0:
        print(f"Normal avg:     {normal_lat.mean():.0f} us")

# Visualization
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. Time series
ax1 = axes[0, 0]
colors = ['green' if s == 'OK' else 'red' for s in df['status']]
ax1.scatter(range(len(lat)), lat, c=colors, alpha=0.7, s=20)
ax1.axhline(y=1000, color='orange', linestyle='--', label='1ms threshold')
ax1.axhline(y=10000, color='red', linestyle='--', label='10ms threshold')
ax1.set_xlabel('Call Index')
ax1.set_ylabel('Latency (us)')
ax1.set_title('First Call Latency per Program Launch')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 2. Normal latency histogram
ax2 = axes[0, 1]
if len(ok_lat) > 0:
    normal = ok_lat[ok_lat < 1000]
    if len(normal) > 0:
        ax2.hist(normal, bins=30, color='green', edgecolor='black', alpha=0.7)
        ax2.set_xlabel('Latency (us)')
        ax2.set_ylabel('Count')
        ax2.set_title(f'Normal Latency (<1ms)\nMean: {normal.mean():.0f}us')
        ax2.grid(True, alpha=0.3)

# 3. All latency histogram
ax3 = axes[1, 0]
ax3.hist(lat, bins=50, color='blue', edgecolor='black', alpha=0.7)
ax3.set_xlabel('Latency (us)')
ax3.set_ylabel('Count')
ax3.set_title('All Latency Distribution')
ax3.grid(True, alpha=0.3)

# 4. Status pie chart
ax4 = axes[1, 1]
status_counts = df['status'].value_counts()
colors_map = {'OK': '#2ecc71', 'TIMEOUT': '#e74c3c', 'FAIL': '#e67e22', 'UNKNOWN': '#95a5a6'}
pie_colors = [colors_map.get(s, 'gray') for s in status_counts.index]
ax4.pie(status_counts.values, labels=[f"{s}\n{v} ({100*v/len(df):.1f}%)"
        for s, v in zip(status_counts.index, status_counts.values)],
        colors=pie_colors, startangle=90)
ax4.set_title('Call Status Distribution')

plt.suptitle('setArmMode First Call Stress Test', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/first_call_latency.png', dpi=150, bbox_inches='tight')
print("")
print("Chart saved to: /tmp/first_call_latency.png")
PYEOF
else
    # 简单统计
    python3 << 'PYEOF'
import pandas as pd
import numpy as np

df = pd.read_csv('/tmp/first_call_latency.csv')
lat = df['latency_us']

print("")
print("=== Latency Statistics ===")
print(f"Count:   {len(lat)}")
print(f"Min:     {lat.min():.0f} us")
print(f"Max:     {lat.max():.0f} us")
print(f"Mean:    {lat.mean():.0f} us")
print(f"Median:  {lat.median():.0f} us")
print(f"P90:     {lat.quantile(0.90):.0f} us")
print(f"P99:     {lat.quantile(0.99):.0f} us")
PYEOF
fi
