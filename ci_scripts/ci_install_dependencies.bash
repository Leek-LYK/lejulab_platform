#!/bin/bash

echo "Checking for required dependencies..."
sudo apt-get update -q && DEBIAN_FRONTEND=noninteractive sudo apt-get install -y \
    curl \
    git \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libosmesa6-dev \
    software-properties-common \
    net-tools \
    vim \
    virtualenv \
    wget \
    xpra \
    xserver-xorg-dev \
    libglfw3-dev \
    liburdfdom-dev \
    liboctomap-dev \
    libassimp-dev \
    ros-noetic-rqt-multiplot \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-realtime-tools \
    build-essential \
    libglib2.0-dev \
    libacl1-dev \
    ros-noetic-controller-interface

echo "Checking for MuJoCo..."
MUJOCO_VERSION="mujoco210"
MUJOCO_DIR="$HOME/.mujoco/$MUJOCO_VERSION"
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    MUJOCO_TAR="$MUJOCO_VERSION-linux-aarch64.tar.gz"
    GLEW_LIB="/usr/lib/aarch64-linux-gnu/libGLEW.so"
else
    MUJOCO_TAR="$MUJOCO_VERSION-linux-x86_64.tar.gz"
    GLEW_LIB="/usr/lib/x86_64-linux-gnu/libGLEW.so"
fi
MUJOCO_URL="https://mujoco.org/download/$MUJOCO_TAR"

# Check if MuJoCo is already installed
if [ ! -d "$MUJOCO_DIR" ]; then
    echo "Installing MuJoCo..."

    # Create the .mujoco directory if it doesn't exist
    mkdir -p "$HOME/.mujoco"

    # Download and extract MuJoCo
    wget "$MUJOCO_URL" -O "$MUJOCO_TAR" \
    && tar -xf "$MUJOCO_TAR" -C "$HOME/.mujoco" \
    && rm "$MUJOCO_TAR"

    # Function to append environment variables if not already present
    append_if_not_exists() {
        local file="$1"
        local line="$2"
        grep -qxF "$line" "$file" || echo "$line" >> "$file"
    }

    # Define environment variable lines
    LD_LIBRARY_PATH_LINE="export LD_LIBRARY_PATH=$MUJOCO_DIR/bin:\$LD_LIBRARY_PATH"
    PATH_LINE="export PATH=\$LD_LIBRARY_PATH:\$PATH"
    LD_PRELOAD_LINE="export LD_PRELOAD=$GLEW_LIB"

    # Append to .bashrc
    append_if_not_exists "$HOME/.bashrc" "$LD_LIBRARY_PATH_LINE"
    append_if_not_exists "$HOME/.bashrc" "$PATH_LINE"
    append_if_not_exists "$HOME/.bashrc" "$LD_PRELOAD_LINE"

    # Append to .zshrc
    append_if_not_exists "$HOME/.zshrc" "$LD_LIBRARY_PATH_LINE"
    append_if_not_exists "$HOME/.zshrc" "$PATH_LINE"
    append_if_not_exists "$HOME/.zshrc" "$LD_PRELOAD_LINE"

    # Source the updated configuration files
    if [ -n "$BASH_VERSION" ]; then
        source "$HOME/.bashrc"
    elif [ -n "$ZSH_VERSION" ]; then
        source "$HOME/.zshrc"
    fi

    echo "MuJoCo installation completed."
else
    echo "MuJoCo is already installed."
fi
# 安装 mujoco-py
echo "Installing mujoco-py..."
if ! pip3 show mujoco-py &> /dev/null; then
    [ -d "/tmp/mujoco-py" ] && rm -rf /tmp/mujoco-py
    git clone https://github.com/openai/mujoco-py.git /tmp/mujoco-py \
        && cd /tmp/mujoco-py/ \
        && pip3 install -U 'mujoco-py<2.2,>=2.1' \
        && pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple \
        && pip3 install -r requirements.dev.txt -i https://pypi.tuna.tsinghua.edu.cn/simple \
        && python3 setup.py install \
        && pip3 install mujoco \
        && pip3 install pynput \
        && cd .. && rm -rf mujoco-py
else
    echo "mujoco-py is already installed."
fi

# if ! ls /usr/local/lib | grep hpp-fcl; then
#     echo "Installing hpp-fcl..."
#     [ -d "/tmp/hpp-fcl" ] && sudo rm -rf /tmp/hpp-fcl
#     git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git /tmp/hpp-fcl \
#         && cd /tmp/hpp-fcl \
#         && mkdir build && cd build \
#         && cmake .. \
#         && make -j$(nproc) \
#         && sudo make install \
#         && cd ../.. 
# else
#     echo "hpp-fcl is already installed."
# fi

# if ! ls /usr/local/lib | grep pinocchio; then
#     echo "Installing pinocchio..."
#     [ -d "/tmp/pinocchio" ] && sudo rm -rf /tmp/pinocchio
#     git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git /tmp/pinocchio \
#         && cd /tmp/pinocchio \
#         && mkdir build && cd build \
#         && cmake .. \
#         && make -j$(nproc) \
#         && sudo make install \
#         && cd ../.. 
# else
#     echo "pinocchio is already installed."
# fi

# if ! ls /usr/local/lib | grep lcm; then
#     echo "Installing lcm..."
#     [ -d "/tmp/lcm" ] && sudo rm -rf /tmp/lcm
#     git clone https://github.com/lcm-proj/lcm.git /tmp/lcm \
#         && cd /tmp/lcm \
#         && mkdir build && cd build \
#         && cmake .. \
#         && make -j$(nproc) \
#         && sudo make install \
#         && cd ../.. \
#         && rm -f /usr/local/lib/liblcm.so \
#         && ln -s /usr/lib/x86_64-linux-gnu/liblcm.so /usr/local/lib/liblcm.so
# else
#     echo "lcm is already installed."
# fi

function install_openvino() {
    local install_script_path="$(dirname "$(realpath "$0")")/install_openvino.sh"
    local packages=(
        "openvino"
        "gnupg"
    )

    # Check if packages are already installed
    local all_installed=true
    for pkg in "${packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            all_installed=false
            break
        fi
    done

    # If all packages are installed, return early
    if [ "$all_installed" = true ]; then
        echo "OpenVINO and GnuPG are already installed."
        return 0
    fi

    # Check for sudo permissions
    if sudo -n true 2>/dev/null; then
        echo "Detected sudo permissions, executing OpenVINO installation script..."
        bash "$install_script_path"
        if [ $? -eq 0 ]; then
            echo "OpenVINO installation completed successfully."
        else
            echo "OpenVINO installation failed." >&2
            return 1
        fi
    else
        echo -e "\033[1;31m======================================= 警告 =======================================" >&2
        echo -e "\033[1;31m    检测到没有 sudo 权限！" >&2
        echo -e "\033[1;31m    OpenVINO 和 GnuPG 安装需要 sudo 权限。" >&2
        echo -e "\033[1;31m    请使用 sudo 权限运行此脚本或手动安装 OpenVINO。" >&2
        echo -e "\033[1;31m    可以手动执行: bash $install_script_path" >&2
        echo -e "\033[1;31m====================================================================================" >&2
        echo -e "\033[0m" >&2
        return 1
    fi
}

# Install OpenVINO
echo "Installing OpenVINO..."
install_openvino
