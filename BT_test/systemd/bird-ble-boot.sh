#!/bin/bash
# systemd 开机启动入口：蓝牙准备 + ROS 环境 + BLE GATT 服务
set -eo pipefail

BT_DIR="/home/nvidia/Bird_ws/BT_test"
cd "$BT_DIR"

# 语音播放走 nvidia 用户 PulseAudio
export PULSE_SERVER="${PULSE_SERVER:-unix:/run/user/1000/pulse/native}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"

# shellcheck disable=SC1091
source "${BT_DIR}/ros_env.sh"

wait_hci0() {
  local i
  for i in $(seq 1 45); do
    if hciconfig hci0 2>/dev/null | grep -q "hci0"; then
      return 0
    fi
    sleep 1
  done
  echo "[bird-ble] 超时：未检测到 hci0" >&2
  return 1
}

ensure_bluez_experimental() {
  local conf="/etc/bluetooth/main.conf"
  if [ -f "$conf" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "$conf" 2>/dev/null; then
    echo "[bird-ble] 启用 BlueZ Experimental（GATT/LE 广播需要）" >&2
    if grep -q '^\[General\]' "$conf"; then
      sed -i '/^\[General\]/a Experimental=true' "$conf"
    else
      printf '%s\n' '[General]' 'Experimental=true' >>"$conf"
    fi
  fi
}

prep_bluetooth() {
  ensure_bluez_experimental
  # 清理上次异常退出残留的 LE 广播占用（bluetoothd: Busy 0x0a）
  systemctl restart bluetooth
  sleep 2
  wait_hci0

  BLE_NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"

  hciconfig hci0 up 2>/dev/null || true
  hciconfig hci0 name "${BLE_NAME}" 2>/dev/null || true
  hciconfig hci0 noscan 2>/dev/null || true
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

# 额外启动参数写在 /etc/default/bird-ble（可选）
EXTRA_ARGS=()
if [ -f /etc/default/bird-ble ]; then
  # shellcheck disable=SC1091
  source /etc/default/bird-ble
fi

exec "${BT_DIR}/run_ble_with_ros.sh" "${EXTRA_ARGS[@]}"
