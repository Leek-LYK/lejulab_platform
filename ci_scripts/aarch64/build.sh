#!/bin/bash
# aarch64 build script - native aarch64 OR x86_64 cross-compile
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(realpath "$SCRIPT_DIR/../..")"
cd "$PROJECT_DIR"
HOST_ARCH="$(uname -m)"
unset LDFLAGS

# ccache 可选：装了就套在编译器前加速，没装则退回纯编译器，避免裸机 runner 直接挂。
# 注意：不能写 `export CC="ccache gcc-11" 2>/dev/null || ...`——赋值永远成功，|| 永不触发。
if command -v ccache >/dev/null 2>&1; then
    CCACHE="ccache "
    ccache --max-size=10G >/dev/null 2>&1 || true
    ccache --set-config compiler_check=content >/dev/null 2>&1 || true
    ccache -s | sed 's/^/[ccache] /' 2>/dev/null || true
else
    echo "[build] WARN: ccache not found, building without cache (slower)"
    CCACHE=""
fi

if [ "$HOST_ARCH" = "aarch64" ]; then
    echo "[build] native aarch64 mode (host=${HOST_ARCH})"
    # 必须 GCC 11（cyclonedds/iceoryx 内嵌 GCC 11 LTO 字节码）
    if ! command -v gcc-11 >/dev/null 2>&1 || ! command -v g++-11 >/dev/null 2>&1; then
        echo "[build] ERROR: gcc-11 / g++-11 not found, install: sudo apt install g++-11"; exit 1
    fi
    export CC="${CCACHE}gcc-11"
    export CXX="${CCACHE}g++-11"
    CROSS_ARGS=()
    BUILD_TESTS=ON; BUILD_TESTING=ON; BUILD_RL=ON; BUILD_MUJOCO=ON
    BUILD_VR_CONTROL=ON

elif [ "$HOST_ARCH" = "x86_64" ] && command -v aarch64-linux-gnu-gcc >/dev/null 2>&1; then
    echo "[build] cross-compile mode (host=${HOST_ARCH} -> target=aarch64)"
    export CC="${CCACHE}aarch64-linux-gnu-gcc"
    export CXX="${CCACHE}aarch64-linux-gnu-g++"
    CROSS_ARGS=(
        -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc
        -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
        -DCMAKE_SYSTEM_NAME=Linux
        -DCMAKE_SYSTEM_PROCESSOR=aarch64
        -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
        -DCMAKE_FIND_ROOT_PATH=/usr/aarch64-linux-gnu
        -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER
        -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH
        -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
        -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH
    )
    # 交叉模式关闭 RL/MuJoCo/测试；完整 VR 仍必须有目标架构 Drake。
    BUILD_TESTS=OFF; BUILD_TESTING=OFF; BUILD_RL=OFF; BUILD_MUJOCO=OFF

else
    echo "[build] ERROR: unsupported arch '${HOST_ARCH}' (need aarch64 host, or x86_64 with aarch64-linux-gnu-gcc)"; exit 1
fi

STAGING="$PROJECT_DIR/staging"; BUILD_DIR="$PROJECT_DIR/build_aarch64"
rm -rf "$STAGING" "$BUILD_DIR"; mkdir -p "$STAGING"

# ============================================================================
# 交叉编译 aarch64 依赖自动补齐
# CMake 的 find_library 在 BOTH 模式下先搜 CMAKE_FIND_ROOT_PATH，
# 而 apt 安装的 :arm64 库在 /usr/lib/aarch64-linux-gnu/，需桥接。
# 新增模块引入新的系统库依赖时只需追加 CROSS_PKGS 数组。
# ============================================================================
if [ "${HOST_ARCH}" = "x86_64" ]; then
    echo "[build] checking aarch64 cross-compile dependencies..."

    CROSS_PKGS=(
        "libasound2-dev:arm64"   # leju-audio ALSA
        "libudev-dev:arm64"      # dexhand libbc_stark_sdk.so
    )

    for pkg in "${CROSS_PKGS[@]}"; do
        if dpkg -l "$pkg" 2>/dev/null | grep -q '^ii'; then
            echo "[build]   ${pkg} — already installed"
        else
            echo "[build]   ${pkg} — installing..."
            apt-get update -qq 2>/dev/null
            apt-get install -y -qq "$pkg"
            echo "[build]   ${pkg} — done"
        fi
    done

    # 桥接 /usr/lib/aarch64-linux-gnu/ -> CMake FIND_ROOT_PATH
    SRC=/usr/lib/aarch64-linux-gnu
    DST=/usr/aarch64-linux-gnu/lib
    for lib in libasound.so libasound.so.2 libudev.so libudev.so.1; do
        if [ -e "$SRC/$lib" ] && [ ! -e "$DST/$lib" ]; then
            ln -sf "$SRC/$lib" "$DST/$lib"
            echo "[build]   linked $DST/$lib"
        fi
    done

    echo "[build] cross-compile dependencies ready"
fi

if [ "$HOST_ARCH" = "aarch64" ]; then
    # Native runners use the standard Drake prefix unless the environment
    # supplies an explicit target-architecture installation.
    if [ -z "${DRAKE_PREFIX:-}" ]; then
        for candidate in /opt/drake; do
            if [ -f "$candidate/lib/cmake/drake/drake-config.cmake" ]; then
                DRAKE_PREFIX="$candidate"
                break
            fi
        done
    fi
    [ -n "${DRAKE_PREFIX:-}" ] && [ -f "$DRAKE_PREFIX/lib/cmake/drake/drake-config.cmake" ] || {
        echo "[build] ERROR: aarch64 Drake was not found in standard locations" >&2
        echo "[build] Set DRAKE_PREFIX to a valid target-architecture Drake prefix." >&2
        exit 1
    }
elif [ "$HOST_ARCH" = "x86_64" ]; then
    # Cross builds use the target Drake staged at /opt/drake in the CI
    # sysroot. Never fall back to a host Drake discovered by CMake.
    DRAKE_PREFIX="${DRAKE_PREFIX:-/opt/drake}"
    [ -f "$DRAKE_PREFIX/lib/cmake/drake/drake-config.cmake" ] || {
        echo "[build] ERROR: aarch64 Drake not found at $DRAKE_PREFIX" >&2
        echo "[build] Provide a complete aarch64 sysroot/Drake prefix or use docker/run_build.sh" >&2
        exit 1
    }
fi

cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$STAGING/installed" \
    "${CROSS_ARGS[@]}" \
    -DCROSS_COMPILE_AARCH64=ON \
    -DBUILD_TESTS=$BUILD_TESTS -DBUILD_EXAMPLES=ON -DBUILD_TESTING=$BUILD_TESTING \
    -DBUILD_RL_CONTROLLER=$BUILD_RL -DBUILD_JOYSTICK=ON -DBUILD_MUJOCO_SIM=$BUILD_MUJOCO \
    -DBUILD_VR_CONTROL=ON -DDRAKE_PREFIX="$DRAKE_PREFIX"

cmake --build "$BUILD_DIR" -j"$(nproc)"
cmake --install "$BUILD_DIR"

cat > "$STAGING/installed/VR_BUILD.txt" <<EOF
BUILD_VR_CONTROL=ON
DRAKE_PREFIX=$DRAKE_PREFIX
TARGETS=quest_udp_to_dds_node,quest_vr_control_node,quest_vr_abs_control_node,libleju-ik.so
EOF

# Strip aarch64 ELF symbols to shrink artifact size
# On x86_64 cross-compile runners, must use the cross strip — native strip ignores ARM binaries
STRIP="$(which aarch64-linux-gnu-strip 2>/dev/null || echo strip)"
echo "[build] stripping symbols (using $STRIP)..."
while IFS= read -r f; do
    [ -L "$f" ] && continue
    if file "$f" 2>/dev/null | grep -q "ELF"; then
        $STRIP --strip-all "$f" 2>/dev/null || echo "[build] WARN: strip failed on $f"
    fi
done < <(find "$STAGING/installed/bin" "$STAGING/installed/lib" -type f 2>/dev/null || true)

command -v ccache >/dev/null 2>&1 && ccache -s | sed 's/^/[ccache] /' || true
echo "[build] done"; ls -la "$STAGING/installed/" || true
