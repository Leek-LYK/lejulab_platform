#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

IMAGE_NAME="${IMAGE_NAME:-lejulab-aarch64:latest}"
BUILD_DIR="${BUILD_DIR:-build_arm64}"
JOBS="${JOBS:-4}"
ROBOT_VERSION="${ROBOT_VERSION:-17}"

echo "========================================"
echo " ARM64 Real-Robot Build"
echo "========================================"
echo "Project       : $PROJECT_DIR"
echo "Docker image  : $IMAGE_NAME"
echo "Build dir     : $BUILD_DIR"
echo "Robot version : $ROBOT_VERSION"
echo "Jobs          : $JOBS"
echo "========================================"

docker run --rm \
    --platform linux/arm64 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e ROBOT_VERSION="$ROBOT_VERSION" \
    -e BUILD_DIR="$BUILD_DIR" \
    -e JOBS="$JOBS" \
    -v "$PROJECT_DIR:/workspace" \
    -w /workspace \
    "$IMAGE_NAME" \
    bash -lc '
        echo "===== ARM64 ENV ====="
        uname -m
        echo "ROBOT_VERSION=$ROBOT_VERSION"

        echo "===== CMAKE CONFIGURE ====="
        cmake -S . -B "$BUILD_DIR" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_RL_CONTROLLER=ON \
            -DBUILD_JOYSTICK=ON \
            -DBUILD_MUJOCO_SIM=OFF \
            -DBUILD_VR_CONTROL=OFF \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_TESTING=OFF \
            -DALSA_LIBRARY:FILEPATH=/usr/lib/aarch64-linux-gnu/libasound.so \
            -DALSA_INCLUDE_DIR:PATH=/usr/include

        echo "===== ARM64 BUILD ====="
        cmake --build "$BUILD_DIR" -j"$JOBS"

        echo "===== BUILD COMPLETE ====="
    '

echo "========================================"
echo " ARM64 build complete: $BUILD_DIR"
echo "========================================"
