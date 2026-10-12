#!/usr/bin/env bash
# 启动仿真容器：
# - NVIDIA CUDA/GPU
# - 原生 Linux X11
# - WSL2/WSLg D3D12 OpenGL 硬件渲染
# - host 网络用于 DDS
#
# 用法：
#   ./run_sim.sh
#   ./run_sim.sh <cmd...>
#
# 容器内最小 MuJoCo 构建示例：
#   cmake -S . -B build_sim -G Ninja \
#     -DFORCE_PURE_CMAKE=ON \
#     -DBUILD_MUJOCO_SIM=ON \
#     -DBUILD_RL_CONTROLLER=OFF \
#     -DBUILD_JOYSTICK=OFF \
#     -DBUILD_EXAMPLES=OFF \
#     -DBUILD_VR_CONTROL=OFF \
#     -DBUILD_CALIBRATION_TOOLS=OFF \
#     -DBUILD_DDS_RPC_TESTS=OFF \
#     -DBUILD_TESTS=OFF \
#     -DBUILD_TESTING=OFF
#
#   cmake --build build_sim --target leju-mujoco-sim -j$(nproc)
#
#   export ROBOT_VERSION=17
#   SIM=build_sim/src/leju-mujoco-sim/leju-mujoco-sim
#   $SIM src/leju_assets/models/biped_s17/xml/scene.xml
#   $SIM --headless src/leju_assets/models/biped_s17/xml/scene.xml

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

source "${SCRIPT_DIR}/_ensure_image.sh"

IMAGE_NAME="lejulab-sim-x86:latest"
CONTAINER_NAME="lejulab_sim_$(echo "$PROJECT_DIR" | md5sum | cut -c1-8)"

# 本地无镜像 → 云端拉取 → 回退本地构建
ensure_image "$IMAGE_NAME" bash "$SCRIPT_DIR/build_sim_image.sh" || exit 1

# ------------------------------------------------------------
# GPU
# ------------------------------------------------------------

GPU_ARGS=()

if docker info 2>/dev/null | grep -qi nvidia \
    || command -v nvidia-container-runtime >/dev/null 2>&1; then

    GPU_ARGS=(
        --gpus all
        --runtime nvidia
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all,display
    )
else
    echo "警告：未检出 NVIDIA runtime，将使用软件渲染。"
    GPU_ARGS=(
        -e LIBGL_ALWAYS_SOFTWARE=1
    )
fi

# ------------------------------------------------------------
# 显示 / WSLg
# ------------------------------------------------------------

DISPLAY_ARGS=(
    -e DISPLAY="${DISPLAY:-:0}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

# WSL2 / WSLg：
# 通过 /dev/dxg + /usr/lib/wsl 让 Mesa D3D12 使用 Windows GPU。
WSL_ARGS=()

if [ -e /dev/dxg ] && [ -d /usr/lib/wsl ]; then
    echo "检测到 WSLg：启用 D3D12 GPU OpenGL 透传"

    WSL_ARGS=(
        --device=/dev/dxg
        -v /usr/lib/wsl:/usr/lib/wsl:ro
        -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
        -e MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
    )

    # WSLg runtime
    if [ -d /mnt/wslg ]; then
        WSL_ARGS+=(
            -v /mnt/wslg:/mnt/wslg
        )
    fi

    if [ -n "${WAYLAND_DISPLAY:-}" ]; then
        WSL_ARGS+=(
            -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"
        )
    fi

    if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
        WSL_ARGS+=(
            -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
        )
    fi

    if [ -n "${PULSE_SERVER:-}" ]; then
        WSL_ARGS+=(
            -e PULSE_SERVER="${PULSE_SERVER}"
        )
    fi
else
    # 原生 Linux X11
    xhost +local:root >/dev/null 2>&1 \
        || echo "警告：xhost 不可用，GUI 可能无法启动；headless 不受影响。"
fi

# ------------------------------------------------------------
# 复用已有容器
# ------------------------------------------------------------

if docker ps -aq -f name="^${CONTAINER_NAME}$" | grep -q .; then
    docker start "$CONTAINER_NAME" >/dev/null 2>&1 || true

    if [ "$#" -gt 0 ]; then
        exec docker exec "$CONTAINER_NAME" bash -lc "$*"
    fi

    exec docker exec -it "$CONTAINER_NAME" bash
fi

# ------------------------------------------------------------
# 创建新容器
# ------------------------------------------------------------

RUN_ARGS=(
    docker run -it
    --name "$CONTAINER_NAME"
    --net host

    "${GPU_ARGS[@]}"
    "${DISPLAY_ARGS[@]}"
    "${WSL_ARGS[@]}"

    -e ROBOT_VERSION="${ROBOT_VERSION:-17}"

    -v "$PROJECT_DIR":/workspace
    -w /workspace

    "$IMAGE_NAME"
)

if [ "$#" -gt 0 ]; then
    exec "${RUN_ARGS[@]}" bash -lc "$*"
fi

exec "${RUN_ARGS[@]}" bash