#!/bin/bash
# systemd 开机启动入口（尽快拉起 GATT 广播；ROS 由桥接后台等待）
set -eo pipefail

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

wait_hci() {
  local i
  for i in $(seq 1 40); do
    if hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "[bird-ble] 超时：未检测到 ${BLE_HCI_DEV} (${BLE_HW_DESC})" >&2
  return 1
}

hci_up() {
  hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "UP RUNNING"
}

prep_bluetooth() {
  if ! systemctl is-active --quiet bluetooth 2>/dev/null; then
    echo "[bird-ble] 启动 bluetooth.service"
    systemctl start bluetooth 2>/dev/null || true
    sleep 0.8
  fi
  wait_hci || return 1

  # USB 棒冷启动：先 down/up 一次，避免「能连不能写 FFE1」
  if [ "${BLE_BT_KIND}" = "usb_dongle" ]; then
    echo "[bird-ble] USB 蓝牙 settle: ${BLE_HCI_DEV} down/up"
    hciconfig "${BLE_HCI_DEV}" down 2>/dev/null || true
    sleep 0.6
    hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true
    sleep 1.0
  fi

  if ! hci_up; then
    hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true
    sleep 0.5
  fi

  BLE_NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
  hciconfig "${BLE_HCI_DEV}" name "${BLE_NAME}" 2>/dev/null || true
  if [ "${BLE_BT_KIND}" = "onboard_combo" ]; then
    hciconfig "${BLE_HCI_DEV}" noscan 2>/dev/null || true
  fi
  timeout 1 bluetoothctl system-alias "${BLE_NAME}" >/dev/null 2>&1 || true
  timeout 1 bluetoothctl discoverable off >/dev/null 2>&1 || true
  timeout 1 bluetoothctl pairable off >/dev/null 2>&1 || true
  # 给 bluez/GATT 再留一点稳定时间（与手动 install 后“热机”接近）
  sleep 1.5
}

prep_bluetooth

# CycloneDDS 绑定 wlan0；必须等 IPv4（仅 link UP 时 DDS 会报不支持 UDP）
wait_dds_network() {
  local iface="wlan0"
  if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
    iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
    iface="${iface:-wlan0}"
  fi
  local i
  local up=0
  for i in $(seq 1 120); do
    if ip link show "${iface}" 2>/dev/null | grep -q "UP"; then
      up=1
      if ip -4 addr show "${iface}" 2>/dev/null | grep -q "inet "; then
        echo "[bird-ble] DDS 网卡就绪: ${iface} (已有 IPv4)"
        # 再等 2s，避免「有 IP 但仍不支持 UDP」的窗口
        sleep 2
        return 0
      fi
    fi
    sleep 0.5
  done
  if [ "${up}" = "1" ]; then
    echo "[bird-ble] 警告: ${iface} 已 UP 但尚无 IPv4，先启动 BLE（ROS 桥会重试）" >&2
  else
    echo "[bird-ble] 警告: ${iface} 未 UP，ROS2 桥接可能启动失败（稍后自动重试）" >&2
  fi
}
wait_dds_network

# 与 ros2-bringup 共用 runtime DDS（本机 IP Peer），避免无组播时 DDS 空转占满 CPU
_PREPARE="$(cd "$(dirname "$0")/../../.." && pwd)/scripts/prepare-cyclonedds-runtime.sh"
if [ -x "${_PREPARE}" ]; then
  # shellcheck disable=SC1090
  source "${_PREPARE}" || true
  echo "[bird-ble] CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
fi
unset _PREPARE

# 若 ROS2 栈尚未起来，多等一会再提示（不永久阻塞 GATT）
if ! pgrep -f hightorque_controller_node >/dev/null 2>&1; then
  echo "[bird-ble] 提示: 控制器尚未运行；若刚开机，ros2-bringup 可能仍在等 DDS" >&2
fi

systemctl stop torque-cmd-vel.service 2>/dev/null || true
systemctl disable torque-cmd-vel.service 2>/dev/null || true
pkill -f 'locate_face_cpp/build/locate_face' 2>/dev/null || true
pkill -f 'locate_face\.py' 2>/dev/null || true
pkill -f 'face_yunet_worker' 2>/dev/null || true

# joy_mapper 热替换放到后台，避免拖慢 GATT 注册、与冷启动抢资源
if [ -x "${BT_DIR}/ensure_bfm_joy_mapper.sh" ]; then
  (
    sleep 8
    "${BT_DIR}/ensure_bfm_joy_mapper.sh" || true
  ) >/tmp/ensure_bfm_joy_mapper.boot.log 2>&1 &
fi

EXTRA_ARGS=()
if [ -f /etc/default/bird-ble ]; then
  source /etc/default/bird-ble
fi

exec "${BT_DIR}/run_ble_with_ros.sh" "${EXTRA_ARGS[@]}"
