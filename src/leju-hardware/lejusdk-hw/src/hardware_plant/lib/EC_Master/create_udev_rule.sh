#!/bin/bash

# 定义 udev 规则内容
UDEV_RULE='ACTION=="add", SUBSYSTEM=="pci", DRIVERS=="igb", KERNELS=="0000:04:00.0", RUN+="/bin/sh -c '\''echo 0000:04:00.0 > /sys/bus/pci/drivers/igb/unbind'\''"'

# 定义 udev 规则文件路径
UDEV_RULE_FILE="/etc/udev/rules.d/99-unbind-igb-04.rules"

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
  echo "请以 root 权限运行该脚本。"
  exit 1
fi

# 创建或覆盖 udev 规则文件
echo "$UDEV_RULE" > "$UDEV_RULE_FILE"

# 设置文件权限
chmod 644 "$UDEV_RULE_FILE"

# 重新加载 udev 规则
udevadm control --reload
udevadm trigger

echo "Udev 规则已创建并重新加载。"
