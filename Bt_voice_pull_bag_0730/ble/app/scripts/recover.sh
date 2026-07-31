#!/bin/bash
# 蓝牙适配器丢失时恢复（按平台：Orin USB / RK 板载）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../platform_hw.sh"
cd "${BT_DIR}"
cd "${BT_DIR}"

echo "========== 平台 =========="
echo "主控: ${BLE_PLATFORM} | ${BLE_BOARD_NAME}"
echo "蓝牙: ${BLE_HW_DESC} (${BLE_CHIP})"
echo ""

echo "========== 当前状态 =========="
bluetoothctl show 2>&1 | head -5 || true
echo ""
hciconfig -a 2>&1 || true
echo ""

if hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
  echo "[OK] ${BLE_HCI_DEV} 已存在，无需恢复。直接: ${BT_DIR}/start.sh"
  exit 0
fi

echo "========== 尝试恢复 ${BLE_HCI_DEV}（需 sudo）=========="
if [ "$(id -u)" -ne 0 ]; then
  echo "请执行: sudo $0"
  exit 1
fi

rfkill unblock bluetooth 2>/dev/null || true
rfkill unblock wifi 2>/dev/null || true
systemctl restart bluetooth
sleep 2

if [ "${BLE_BT_KIND}" = "usb_dongle" ]; then
  echo "[Orin] 重载 USB 蓝牙驱动 btusb..."
  modprobe -r btusb 2>/dev/null || true
  sleep 1
  modprobe btusb 2>/dev/null || true
  sleep 2
else
  echo "[RK] 重载板载网卡蓝牙 (btusb + 固件等待 ${BLE_FW_WAIT_SEC}s)..."
  modprobe -r btusb 2>/dev/null || true
  sleep 1
  modprobe btusb 2>/dev/null || true
  sleep "${BLE_FW_WAIT_SEC}"
fi

hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true

echo ""
if hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
  echo "[OK] ${BLE_HCI_DEV} 已恢复"
  hciconfig "${BLE_HCI_DEV}"
  bluetoothctl show | head -6
  echo ""
  echo "现在可运行: cd ${BT_DIR} && ./start.sh"
else
  echo "[失败] 仍无 ${BLE_HCI_DEV}"
  if [ "${BLE_BT_KIND}" = "usb_dongle" ]; then
    echo "  Orin: 检查 USB 蓝牙棒是否插好，或 sudo reboot"
  else
    echo "  RK: rfkill list / sudo reboot"
  fi
  exit 1
fi
