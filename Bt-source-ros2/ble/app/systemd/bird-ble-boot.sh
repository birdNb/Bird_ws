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
  for i in $(seq 1 20); do
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
    sleep 0.4
  fi
  wait_hci || return 1

  if ! hci_up; then
    hciconfig "${BLE_HCI_DEV}" up 2>/dev/null || true
  fi

  BLE_NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
  hciconfig "${BLE_HCI_DEV}" name "${BLE_NAME}" 2>/dev/null || true
  if [ "${BLE_BT_KIND}" = "onboard_combo" ]; then
    hciconfig "${BLE_HCI_DEV}" noscan 2>/dev/null || true
  fi
  timeout 1 bluetoothctl system-alias "${BLE_NAME}" >/dev/null 2>&1 || true
  timeout 1 bluetoothctl discoverable off >/dev/null 2>&1 || true
  timeout 1 bluetoothctl pairable off >/dev/null 2>&1 || true
}

prep_bluetooth

# CycloneDDS 绑定 wlan0；WiFi 未 UP 时创建 ROS 节点会失败并导致控制桥退出
wait_dds_network() {
  local iface="wlan0"
  if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
    iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
    iface="${iface:-wlan0}"
  fi
  local i
  for i in $(seq 1 60); do
    if ip link show "${iface}" 2>/dev/null | grep -q "UP"; then
      echo "[bird-ble] DDS 网卡就绪: ${iface}"
      return 0
    fi
    sleep 0.5
  done
  echo "[bird-ble] 警告: ${iface} 未 UP，ROS2 桥接可能启动失败（稍后自动重试）" >&2
}
wait_dds_network

# 注意：不要阻塞等待 midware/joint_states。
# 否则 GATT 服务迟迟不启动 → 手机能搜到名称/残留广播但连不上。
# ROS2 由 ble_ros_bridge 后台等待；指令在栈未就绪时会打 warn。
if ! pgrep -f hightorque_controller_node >/dev/null 2>&1; then
  echo "[bird-ble] 提示: 控制器尚未运行，先启动 BLE；请确认 systemctl status ros2-bringup" >&2
fi

systemctl stop torque-cmd-vel.service 2>/dev/null || true
systemctl disable torque-cmd-vel.service 2>/dev/null || true
pkill -f 'locate_face_cpp/build/locate_face' 2>/dev/null || true
pkill -f 'locate_face\.py' 2>/dev/null || true
pkill -f 'face_yunet_worker' 2>/dev/null || true

# BFM：确保 joy_mapper 回中不发零速（与 install.sh 同一套逻辑）
if [ -x "${BT_DIR}/ensure_bfm_joy_mapper.sh" ]; then
  "${BT_DIR}/ensure_bfm_joy_mapper.sh" || echo "[bird-ble] warn: ensure_bfm_joy_mapper 失败" >&2
fi

EXTRA_ARGS=()
if [ -f /etc/default/bird-ble ]; then
  source /etc/default/bird-ble
fi

exec "${BT_DIR}/run_ble_with_ros.sh" "${EXTRA_ARGS[@]}"
