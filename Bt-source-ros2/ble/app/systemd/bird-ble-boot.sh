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

# USB 冷启动：等 WiFi 有 IP 再拉 GATT。
# 否则广播刚起来就被 WiFi 关联抖动触发 BlueZ Release，看门狗虽显示 ActiveInstances=1，
# 空中常无有效广播，需手动 systemctl restart 才可搜到。
wait_wifi_for_usb_bt() {
  if [ "${BLE_BT_KIND}" != "usb_dongle" ]; then
    return 0
  fi
  local iface="wlan0"
  if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
    iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
    iface="${iface:-wlan0}"
  fi
  echo "[bird-ble] USB 蓝牙：等待 ${iface} IPv4 稳定后再广播（最多 90s）..."
  local i
  for i in $(seq 1 90); do
    if ip -4 addr show "${iface}" 2>/dev/null | grep -q "inet "; then
      echo "[bird-ble] ${iface} 已有 IPv4，再稳定 5s"
      sleep 5
      if ip -4 addr show "${iface}" 2>/dev/null | grep -q "inet "; then
        echo "[bird-ble] ${iface} 稳定: $(ip -4 -o addr show "${iface}" | awk '{print $4}' | head -1)"
        return 0
      fi
    fi
    sleep 1
  done
  echo "[bird-ble] 警告: ${iface} 长时间无 IPv4，仍启动 BLE" >&2
}
wait_wifi_for_usb_bt

# 尽快拉起 GATT/广播；DDS 由 ROS 桥后台等待
# 若已有 IPv4，顺手生成 runtime DDS，减少桥接空转 CPU
_PREPARE="$(cd "$(dirname "$0")/../../.." && pwd)/scripts/prepare-cyclonedds-runtime.sh"
_dds_iface="wlan0"
if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
  _dds_iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
  _dds_iface="${_dds_iface:-wlan0}"
fi
if ip -4 addr show "${_dds_iface}" 2>/dev/null | grep -q "inet "; then
  if [ -x "${_PREPARE}" ]; then
    # shellcheck disable=SC1090
    source "${_PREPARE}" || true
    echo "[bird-ble] CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
  fi
else
  echo "[bird-ble] ${_dds_iface} 尚无 IPv4，先启动 BLE 广播；ROS 桥稍后重试" >&2
fi
unset _PREPARE _dds_iface

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
