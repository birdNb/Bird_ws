#!/bin/bash
# 平台路径自动检测（RK3588s LubanCat / Jetson Orin 等）

_pe_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export BT_DIR="${BT_DIR:-${_pe_dir}}"
# 发布包：ble_device_name.conf 在包根目录（app/ 的上一级）
if [ -f "${BT_DIR}/../ble_device_name.conf" ]; then
  export PKG_DIR="${PKG_DIR:-$(cd "${BT_DIR}/.." && pwd)}"
else
  export PKG_DIR="${PKG_DIR:-${BT_DIR}}"
fi
export BIRD_WS="${BIRD_WS:-$(cd "${PKG_DIR}/.." && pwd)}"
export BLE_DEVICE_NAME_FILE="${BLE_DEVICE_NAME_FILE:-${PKG_DIR}/ble_device_name.conf}"
_ws_owner="$(stat -c '%U' "${BIRD_WS}" 2>/dev/null || true)"
if [ -n "${BIRD_USER:-}" ]; then
  :
elif [ -n "$_ws_owner" ] && [ "$_ws_owner" != "root" ]; then
  export BIRD_USER="$_ws_owner"
else
  export BIRD_USER="${USER:-hightorque}"
fi
export BIRD_HOME="${BIRD_HOME:-/home/${BIRD_USER}}"
export SIM2REAL_WS="${SIM2REAL_WS:-${BIRD_HOME}/sim2real}"
export BIRD_BLE_UID="${BIRD_BLE_UID:-$(id -u "${BIRD_USER}" 2>/dev/null || echo 1000)}"
unset _pe_dir _ws_owner
