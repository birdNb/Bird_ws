#!/bin/bash
# 平台路径自动检测（RK3588s LubanCat / Jetson Orin 等）

_pe_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export BT_DIR="${BT_DIR:-${_pe_dir}}"
# 三合一包：ble_device_name.conf 在包根（app/../../）；旧 BLE 单包：在 ble/（app/../）
if [ -f "${BT_DIR}/../../ble_device_name.conf" ]; then
  export PKG_DIR="${PKG_DIR:-$(cd "${BT_DIR}/../.." && pwd)}"
elif [ -f "${BT_DIR}/../ble_device_name.conf" ]; then
  export PKG_DIR="${PKG_DIR:-$(cd "${BT_DIR}/.." && pwd)}"
else
  export PKG_DIR="${PKG_DIR:-${BT_DIR}}"
fi
export BIRD_WS="${BIRD_WS:-$(cd "${PKG_DIR}/.." && pwd)}"
export BLE_DEVICE_NAME_FILE="${BLE_DEVICE_NAME_FILE:-/var/lib/bird-ble/ble_device_name.conf}"
_ws_owner="$(stat -c '%U' "${BIRD_WS}" 2>/dev/null || true)"
if [ -n "${BIRD_USER:-}" ]; then
  :
elif [ -n "$_ws_owner" ] && [ "$_ws_owner" != "root" ]; then
  export BIRD_USER="$_ws_owner"
else
  export BIRD_USER="${USER:-hightorque}"
fi
export BIRD_HOME="${BIRD_HOME:-/home/${BIRD_USER}}"
if [ -z "${COLCON_WS:-}" ] || [ ! -f "${COLCON_WS}/install/setup.bash" ]; then
  for _cw in "${BIRD_HOME}/hightorque_workspace" "${BIRD_HOME}/colcon_ws"; do
    if [ -f "${_cw}/install/setup.bash" ]; then
      export COLCON_WS="${_cw}"
      break
    fi
  done
fi
export COLCON_WS="${COLCON_WS:-${BIRD_HOME}/colcon_ws}"
unset _cw
export BIRD_BLE_UID="${BIRD_BLE_UID:-$(id -u "${BIRD_USER}" 2>/dev/null || echo 1000)}"
unset _pe_dir _ws_owner
