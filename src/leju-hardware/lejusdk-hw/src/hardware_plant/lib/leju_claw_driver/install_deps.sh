#!/bin/bash
SCRIPT_DIR=$(dirname $(readlink -f "$0"))

# 更新包列表
sudo apt-get update || { echo "Failed to update package list"; exit 1; }

# 安装 libusb-1.0-0-dev
sudo apt-get install -y libusb-1.0-0-dev || { echo "Failed to install libusb-1.0-0-dev"; exit 1; }

# 根据架构选择 BMAPI 库路径
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    BMAPI_ARCH_DIR="arm64"
else
    BMAPI_ARCH_DIR="unix64"
fi

# 复制动态库文件到 /usr/local/lib/
sudo cp $SCRIPT_DIR/../BMAPI/bin/${BMAPI_ARCH_DIR}/release/libbmapi64.so /usr/local/lib/ || { echo "Failed to copy libbmapi64.so"; exit 1; }

# 修改权限
sudo chmod +x /usr/local/lib/libbmapi64.so || { echo "Failed to change permissions for libbmapi64.so"; exit 1; }

# 刷新共享库缓存
sudo ldconfig || { echo "Failed to run ldconfig"; exit 1; }

echo "Script executed successfully"
