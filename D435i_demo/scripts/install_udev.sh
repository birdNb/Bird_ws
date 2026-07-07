#!/usr/bin/env bash
# 安装 Intel RealSense udev 规则（改善 hidraw 权限，便于后续使用 SDK）
set -euo pipefail

RULES_URL="https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-realsense-libusb.rules"

echo "[install_udev] 下载 udev 规则 ..."
tmp="$(mktemp)"
curl -fsSL "$RULES_URL" -o "$tmp"
sudo cp "$tmp" "$RULES_DST"
rm -f "$tmp"

echo "[install_udev] 重载 udev ..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[install_udev] 完成: $RULES_DST"
echo "请重新插拔 D435i，并用普通用户运行 ./start.sh（不要用 sudo）"
