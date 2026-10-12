#!/bin/bash
# 开源发布产物的 sanity 检查（三档模型：source / stub / binary）
# 用法: sanity.sh <staging-dir>
# 任一项失败 = 非零退出
#
# 与旧版（纯二进制"源码零泄漏"）的区别：
#   source 档现在按设计带 cpp/h/CMakeLists，故不再全树禁止源码；
#   改为检查三档结构是否正确（核心库 stub 完整、installed/ 无源码实现、
#   非 stub 库的内部头不泄漏）。

set -euo pipefail

# Keep the final status check safe under nounset when all checks pass.
FAIL=0

STAGING="${1:?usage: $0 <staging-dir>}"
[ -d "$STAGING" ] || { echo "[sanity] staging dir not found: $STAGING"; exit 1; }

MANIFEST_LIB="$(cd "$(dirname "$0")" && pwd)/opensource_manifest_lib.sh"
[ -f "$MANIFEST_LIB" ] || { echo "[sanity] manifest parser missing: $MANIFEST_LIB"; exit 1; }
source "$MANIFEST_LIB"
[ -f "$STAGING/opensource_packages.txt" ] || { echo "[sanity] release manifest missing from staging"; exit 1; }
parse_manifest "$STAGING/opensource_packages.txt"
parse_opensource_cmake "$STAGING/CMakeLists.txt"

is_in_list() {
    local needle="$1" item
    shift
    for item in "$@"; do
        [ "$item" = "$needle" ] && return 0
    done
    return 1
}
for pkg in "${STUB_PKGS[@]}" "${BINARY_PKGS[@]}"; do
    if is_in_list "$pkg" "${SRC_PKGS[@]}"; then
        echo "[sanity] manifest exception conflicts with source package: $pkg"
        exit 1
    fi
done

# ============================================================
# 1. 核心库 stub 完整：installed/{lib,include} 必须有 lowlevel + vr
#    （stub 的 IMPORTED_LOCATION / INTERFACE_INCLUDE_DIRECTORIES 指向它们）
# ============================================================
echo "[sanity 1/6] checking core lib stubs..."
MISSING_STUB=""
for pkg in "${STUB_PKGS[@]}"; do
    name="$(basename "$pkg")"
    case "$pkg" in
        lejusdk/lejusdk-comm-dds/lejusdk-dds-idl)
            paths=(installed/lib/liblejusdk-dds-idl.a installed/include/lejusdk-dds-idl) ;;
        lejusdk/lejusdk-comm-dds/lejusdk-topic-pubsub)
            paths=(installed/include/lejusdk-topic-pubsub) ;;
        lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc)
            paths=(installed/lib/liblejusdk-dds-rpc.a installed/include/dds_rpc installed/include/rpc_idl) ;;
        *)
            paths=("installed/lib/lib${name}.so" "installed/include/$name") ;;
    esac
    for p in "${paths[@]}"; do
        if [ ! -e "$STAGING/$p" ]; then
            MISSING_STUB+="$p "
        fi
    done
done
# 当前发布契约要求这两个核心 stub 始终存在，即使清单误删也必须失败。
for p in installed/lib/liblejusdk-lowlevel.so installed/lib/liblejusdk-vr.so installed/include/lejusdk-lowlevel installed/include/lejusdk-vr; do
    if [ ! -e "$STAGING/$p" ]; then MISSING_STUB+="$p "; fi
done
if [ -n "$MISSING_STUB" ]; then
    echo "[sanity 1/6] FAIL: core lib stub missing: $MISSING_STUB"
    FAIL=1
else
    echo "[sanity 1/6] OK"
fi

# ============================================================
# 2. installed/ 无源码实现
#    核心库是预编译 .so，不应有 .cpp/.c/.cc/CMakeLists/.cmake；
#    installed/include 下只允许核心库和 DDS SDK 公共头；其它内部实现头不随 installed 泄漏。
# ============================================================
echo "[sanity 2/6] checking installed/ has no source impl..."
LEAKED=$(find "$STAGING/installed" -type f -regextype posix-extended \
    -regex '.*\.(cpp|cc|cxx|c)$|.*/CMakeLists\.txt$|.*\.cmake$' 2>/dev/null || true)
BAD_INC=$(find "$STAGING/installed/include" -mindepth 1 -maxdepth 1 \
    ! -name 'lejusdk-lowlevel' ! -name 'lejusdk-vr' \
    ! -name 'lejusdk-dds-idl' ! -name 'lejusdk-topic-pubsub' \
    ! -name 'dds_rpc' ! -name 'rpc_idl' 2>/dev/null || true)
if [ -n "$LEAKED" ]; then
    echo "[sanity 2/6] FAIL: source impl leaked into installed/:"
    echo "$LEAKED"
    FAIL=1
elif [ -n "$BAD_INC" ]; then
    echo "[sanity 2/6] FAIL: non-core include dirs leaked into installed/include:"
    echo "$BAD_INC"
    FAIL=1
else
    echo "[sanity 2/6] OK"
fi

# ============================================================
# 3. ELF strip 状态
#    3rd_party (cyclonedds / iceoryx / onnxruntime / SDL3) 按设计不动符号 —— 跳过
# ============================================================
echo "[sanity 3/6] checking ELF strip status..."
is_third_party() {
    case "$(basename "$1")" in
        libddsc*|libddscxx*|libcycloneddsidl*|libcycloneddsidlcxx*) return 0 ;;
        libiceoryx*|libiox*) return 0 ;;
        libonnxruntime*) return 0 ;;
        libSDL3*) return 0 ;;
        libmujoco*) return 0 ;;
        iox-roudi) return 0 ;;
        *) return 1 ;;
    esac
}
UNSTRIPPED=""
while IFS= read -r f; do
    [ -L "$f" ] && continue
    is_third_party "$f" && continue
    if file "$f" 2>/dev/null | grep -q "ELF"; then
        if file "$f" | grep -q "not stripped"; then
            UNSTRIPPED+="$f"$'\n'
        fi
    fi
done < <(find "$STAGING/installed/bin" "$STAGING/installed/lib" -type f 2>/dev/null)

if [ -n "$UNSTRIPPED" ]; then
    echo "[sanity 3/6] FAIL: unstripped ELFs:"
    echo "$UNSTRIPPED"
    FAIL=1
else
    echo "[sanity 3/6] OK"
fi

# ============================================================
# 4. 关键二进制存在（纯产物档必须在 installed/bin）
# ============================================================
echo "[sanity 4/6] checking required binaries..."
REQUIRED_BINS=(leju-hardware)
MISSING_BINS=""
for b in "${REQUIRED_BINS[@]}"; do
    if [ ! -x "$STAGING/installed/bin/$b" ]; then
        MISSING_BINS+="$b "
    fi
done
# binary 清单按 source 包声明；不同包的 installed 目标名可能不同，显式维护映射。
for pkg in "${BINARY_PKGS[@]}"; do
    case "$pkg" in
        leju-hardware) expected="leju-hardware" ;;
        leju-controllers/leju-rl-controller) expected="run_rl_controller run_rl_mimic_controller controller_switch_cli posture_height_cmd_node" ;;
        leju-mujoco-sim) expected="leju-mujoco-sim" ;;
        lejusdk/lejusdk-recorder) expected="lejusdk_recorder" ;;
        lejusdk/lejusdk-csv-recorder) expected="lejusdk-csv-recorder" ;;
        *) expected="" ;;
    esac
    for b in $expected; do
        if [ ! -x "$STAGING/installed/bin/$b" ]; then
            MISSING_BINS+="$b "
        fi
    done
done
if [ -n "$MISSING_BINS" ]; then
    echo "[sanity 4/6] FAIL: missing required binaries: $MISSING_BINS"
    FAIL=1
else
    echo "[sanity 4/6] OK"
fi

# source 包的可执行文件应由下游用户编译，不应与源码重复出现在 installed/bin。
SOURCE_BINARIES=(
    run_dummy_controller
    motor_keyboard_ctrl motor_test_runner tests/test_keyboard_ctrl
    leju-joystick leju_joy_autostart_manager
    hw_state_subscriber joint_state_subscriber imu_data_subscriber
    joint_cmd_publisher joint_cmd_subscriber kuavo_control_example
    vr_01_basic_init vr_02_kuavo_arm vr_03_velocity_cmd vr_04_get_state
    vr_05_head_control vr_06_waist_control vr_07_roban_arm
    vr_08_controller_cli vr_09_play_tact
    micphone_to_dds_node audio_player_node play_audio_cli audio_stream_probe
    lejusdk-csv-recorder system_info_node system_info_echo
    quest_udp_to_dds_node quest_vr_control_node quest_vr_abs_control_node
    test_arm_ik_node
)
SOURCE_BINARIES_PRESENT=""
for b in "${SOURCE_BINARIES[@]}"; do
    if [ -e "$STAGING/installed/bin/$b" ]; then
        SOURCE_BINARIES_PRESENT+="$b "
    fi
done
if [ -n "$SOURCE_BINARIES_PRESENT" ]; then
    echo "[sanity 4/6] FAIL: source-buildable binaries remain in installed/bin: $SOURCE_BINARIES_PRESENT"
    FAIL=1
else
    echo "[sanity 4/6] source-buildable binaries absent OK"
fi

if is_in_list "leju-controllers/leju-rl-controller" "${BINARY_PKGS[@]}" \
    && [ ! -e "$STAGING/installed/lib/libkuavo_solver.so" ]; then
    echo "[sanity 4/6] FAIL: RL binary dependency missing: installed/lib/libkuavo_solver.so"
    FAIL=1
fi

if is_in_list "leju-mujoco-sim" "${BINARY_PKGS[@]}" \
    && [ ! -e "$STAGING/installed/lib/libmujoco.so" ]; then
    echo "[sanity 4/6] FAIL: MuJoCo binary dependency missing: installed/lib/libmujoco.so"
    FAIL=1
fi

# ============================================================
# 5. RPATH 仅 $ORIGIN（3rd_party 跳过）
# ============================================================
echo "[sanity 5/6] checking RPATH..."
READELF="$(which aarch64-linux-gnu-readelf 2>/dev/null || echo readelf)"
BAD_RPATH=""
while IFS= read -r so; do
    [ -L "$so" ] && continue
    case "$(basename "$so")" in
        libddsc*|libddscxx*|libcycloneddsidl*|libiceoryx*) continue ;;
        libonnxruntime*) continue ;;
        libSDL3*) continue ;;
        libmujoco*) continue ;;
    esac
    if $READELF -d "$so" 2>/dev/null | grep -qE 'RPATH|RUNPATH'; then
        rpath_line=$($READELF -d "$so" | grep -E 'RPATH|RUNPATH')
        if echo "$rpath_line" | grep -vqE '\$ORIGIN'; then
            BAD_RPATH+="$so: $rpath_line"$'\n'
        fi
    fi
done < <(find "$STAGING/installed/lib" -name "*.so*" -type f 2>/dev/null)

if [ -n "$BAD_RPATH" ]; then
    echo "[sanity 5/6] FAIL: non-\$ORIGIN RPATH:"
    echo "$BAD_RPATH"
    FAIL=1
else
    echo "[sanity 5/6] OK"
fi

# ============================================================
# 6. 开源树结构 + 关键资源
#    - 根 CMakeLists.txt + lejusdk-stubs.cmake 必须在（供用户 cmake 编译）
#    - source 档关键包必须有 CMakeLists.txt（否则编不了）
#    - 关键明文资源（自启动脚本 / launch / config）在
# ============================================================
echo "[sanity 6/6] checking opensource tree structure..."
MISSING_PATHS=""
for p in \
    "CMakeLists.txt" \
    "lejusdk-stubs.cmake" \
    "source_packages.cmake" \
    "src/leju-joystick/services/deploy_joy_autostart.sh" \
    "src/leju_launch/scripts/launch_real.sh" \
    "src/leju-hardware/config" \
    "src/leju-controllers/leju-rl-controller/config"; do
    if [ ! -e "$STAGING/$p" ]; then
        MISSING_PATHS+="$p "
    fi
done
MISSING_CMAKE=""
for pkg in "${SRC_PKGS[@]}"; do
    skip=0
    for exception in "${STUB_PKGS[@]}" "${BINARY_PKGS[@]}"; do
        [ "$pkg" = "$exception" ] && skip=1 && break
    done
    [ "$skip" -eq 1 ] && continue
    if [ ! -d "$STAGING/src/$pkg" ] || [ ! -f "$STAGING/src/$pkg/CMakeLists.txt" ]; then
        MISSING_CMAKE+="src/$pkg "
    fi
done
if [ -n "$MISSING_PATHS" ]; then
    echo "[sanity 6/6] FAIL: required paths missing: $MISSING_PATHS"
    FAIL=1
elif [ -n "$MISSING_CMAKE" ]; then
    echo "[sanity 6/6] FAIL: source pkgs missing CMakeLists.txt: $MISSING_CMAKE"
    FAIL=1
else
    echo "[sanity 6/6] OK"
fi

# 完整 VR 发布必须同时包含 IK 库和其运行资源。
for p in \
    installed/lib/libleju-ik.so \
    src/leju-ik/config/quest3_incremental_ik.yaml \
    src/leju-vr-control/config/quest_vr_control.yaml \
    src/leju-remote/CMakeLists.txt \
    src/leju-vr-control/CMakeLists.txt \
    src/leju_assets/models/biped_s17/urdf/drake/biped_v3_arm.urdf \
    src/leju_launch/launch/vr_teleop.launch; do
    if [ ! -e "$STAGING/$p" ]; then
        echo "[sanity 6/6] FAIL: VR path missing: $p"
        FAIL=1
    fi
done

if [ ! -f "$STAGING/installed/VR_BUILD.txt" ]; then
    echo "[sanity 6/6] FAIL: VR_BUILD.txt marker missing"
    FAIL=1
fi

# src/ 采用完整仓库导出策略，不再限制为 CMake 自动发现的目录；
# stub/binary 例外已由 release.sh 按 manifest 删除并在上方校验。

# build 残留零
BAD_DIRS=""
for bad in build_cmake build_aarch64 build .catkin_tools devel CMakeFiles; do
    if [ -e "$STAGING/$bad" ]; then
        BAD_DIRS+="$bad "
    fi
done
if [ -n "$BAD_DIRS" ]; then
    echo "[sanity 6/6] FAIL: build leftovers found: $BAD_DIRS"
    FAIL=1
fi

# ============================================================
echo "============================================"
if [ "$FAIL" -ne 0 ]; then
    echo "[sanity] FAILED — pipeline should not proceed"
    exit 1
fi
echo "[sanity] all checks passed"
