#!/bin/bash
# 平台蓝牙硬件说明（自动识别 Orin USB / RK 板载）
#
# Orin：外接 USB 蓝牙模块 → btmgmt 广播
# RK3588：板载 RTL8822CE WiFi+蓝牙一体网卡 → Legacy HCI 广播

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]:-$0}")/platform_env.sh"

_eval_platform() {
  python3 -c "
from platform_detect import detect_platform
p = detect_platform()
print(p.platform_id)
print(p.board_name)
print(p.bt_kind)
print(p.bt_chip)
print(p.bt_hci_dev)
print(p.adv_mode)
print(p.fw_wait_sec)
print(p.hw_desc)
"
}

if _lines="$(_eval_platform 2>/dev/null)" && [ -n "$_lines" ]; then
  BLE_PLATFORM="$(echo "$_lines" | sed -n '1p')"
  BLE_BOARD_NAME="$(echo "$_lines" | sed -n '2p')"
  BLE_BT_KIND="$(echo "$_lines" | sed -n '3p')"
  BLE_CHIP="$(echo "$_lines" | sed -n '4p')"
  BLE_HCI_DEV="$(echo "$_lines" | sed -n '5p')"
  BLE_ADV_MODE="$(echo "$_lines" | sed -n '6p')"
  BLE_FW_WAIT_SEC="$(echo "$_lines" | sed -n '7p')"
  BLE_HW_DESC="$(echo "$_lines" | sed -n '8p')"
  export BLE_PLATFORM BLE_BOARD_NAME BLE_BT_KIND BLE_CHIP BLE_HCI_DEV
  export BLE_ADV_MODE BLE_FW_WAIT_SEC BLE_HW_DESC
else
  export BLE_PLATFORM="${BLE_PLATFORM:-rk3588s}"
  export BLE_BOARD_NAME="${BLE_BOARD_NAME:-unknown}"
  export BLE_BT_KIND="${BLE_BT_KIND:-onboard_combo}"
  export BLE_CHIP="${BLE_CHIP:-RTL8822CE}"
  export BLE_HCI_DEV="${BLE_HCI_DEV:-hci0}"
  export BLE_ADV_MODE="${BLE_ADV_MODE:-legacy_hci}"
  export BLE_FW_WAIT_SEC="${BLE_FW_WAIT_SEC:-6}"
  export BLE_HW_DESC="${BLE_HW_DESC:-板载蓝牙}"
fi
unset _lines _eval_platform
