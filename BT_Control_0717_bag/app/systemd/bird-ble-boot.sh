#!/bin/bash
# systemd 开机启动入口
set -eo pipefail

# systemd 启动时加载安装环境
if [ -f /etc/default/bird-ble ]; then
  # shellcheck disable=SC1091
  source /etc/default/bird-ble
fi

BT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BT_DIR"

# shellcheck disable=SC1091
source "${BT_DIR}/platform_env.sh"
source "${BT_DIR}/platform_hw.sh"

export PULSE_SERVER="${PULSE_SERVER:-unix:/run/user/${BIRD_BLE_UID}/pulse/native}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${BIRD_BLE_UID}}"

source "${BT_DIR}/ros_env.sh"

wait_roscore() {
  local i
  for i in $(seq 1 45); do
    if rostopic list >/dev/null 2>&1; then
      echo "[bird-ble] roscore 已就绪"
      return 0
    fi
    sleep 2
  done
  echo "[bird-ble] 警告: roscore 未就绪，BLE 将在后台继续等待" >&2
  return 0
}

wait_roscore

wait_hci() {
  local i
  for i in $(seq 1 45); do
    if hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
      return 0
    fi
    sleep 1
  done
  echo "[bird-ble] 超时：未检测到 ${BLE_HCI_DEV} (${BLE_HW_DESC})" >&2
  return 1
}

prep_bluetooth() {
  systemctl restart bluetooth
  sleep 2
  wait_hci

  BLE_NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
  hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true
  hciconfig "${BLE_HCI_DEV}" name "${BLE_NAME}" 2>/dev/null || true
  if [ "${BLE_BT_KIND}" = "onboard_combo" ]; then
    hciconfig "${BLE_HCI_DEV}" noscan 2>/dev/null || true
  fi
  bluetoothctl system-alias "${BLE_NAME}" 2>/dev/null || true
  bluetoothctl discoverable off 2>/dev/null || true
  bluetoothctl pairable off 2>/dev/null || true
  if command -v btmgmt >/dev/null; then
    btmgmt -i 0 le on 2>/dev/null || true
    btmgmt -i 0 connectable on 2>/dev/null || true
    btmgmt -i 0 discov off 2>/dev/null || true
    btmgmt -i 0 pairable off 2>/dev/null || true
    btmgmt -i 0 bondable off 2>/dev/null || true
  fi
}

prep_bluetooth

EXTRA_ARGS=()
if [ -f /etc/default/bird-ble ]; then
  source /etc/default/bird-ble
fi

exec "${BT_DIR}/run_ble_with_ros.sh" "${EXTRA_ARGS[@]}"
