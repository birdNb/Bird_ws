#!/bin/bash
# 蓝牙适配器丢失（No default controller / Invalid Index）时尝试恢复
set -euo pipefail

echo "========== 当前状态 =========="
bluetoothctl show 2>&1 | head -5 || true
echo ""
hciconfig -a 2>&1 || true
echo ""
if command -v btmgmt >/dev/null; then
  btmgmt info 2>&1 | head -5 || true
fi
echo ""

if hciconfig hci0 2>/dev/null | grep -q "hci0"; then
  echo "[OK] hci0 已存在，无需恢复。直接: ./start.sh"
  exit 0
fi

echo "========== 尝试恢复 hci0（需 sudo）=========="
if [ "$(id -u)" -ne 0 ]; then
  echo "请执行: sudo $0"
  exit 1
fi

rfkill unblock bluetooth 2>/dev/null || true
rfkill unblock all 2>/dev/null || true

systemctl restart bluetooth
sleep 2

# 部分平台蓝牙走串口，重载常见模块
modprobe -r btusb 2>/dev/null || true
sleep 1
modprobe btusb 2>/dev/null || true
sleep 2

hciconfig hci0 up 2>/dev/null || true

echo ""
echo "========== 恢复后状态 =========="
if hciconfig hci0 2>/dev/null | grep -q "hci0"; then
  echo "[OK] hci0 已恢复"
  hciconfig hci0
  bluetoothctl show | head -6
  echo ""
  echo "现在可运行: cd $(dirname "$0") && ./start.sh"
else
  echo "[失败] 仍无 hci0 适配器"
  echo ""
  echo "建议依次尝试:"
  echo "  1. sudo reboot"
  echo "  2. 重启后执行: hciconfig -a  应能看到 hci0"
  echo "  3. 若仍无: dmesg | grep -i bluetooth  查看驱动报错"
  echo "  4. 确认板载蓝牙未被硬件开关/BIOS 禁用"
  exit 1
fi
