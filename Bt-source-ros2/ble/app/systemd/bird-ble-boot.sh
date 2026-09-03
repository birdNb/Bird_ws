#!/bin/bash
# systemd 开机启动入口：
# 优先拉起 GATT 广播（可被扫描）；ROS 核心由桥接后台等待，不堵死蓝牙。
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
  sleep 1.5
}

# USB 冷启动：等 WiFi 有 IP（减少 BlueZ Release 幽灵广播）
wait_wifi_for_usb_bt() {
  if [ "${BLE_BT_KIND}" != "usb_dongle" ]; then
    return 0
  fi
  local iface="wlan0"
  if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
    iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
    iface="${iface:-wlan0}"
  fi
  echo "[bird-ble] USB 蓝牙：等待 ${iface} IPv4 稳定（最多 90s）..."
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
  echo "[bird-ble] 警告: ${iface} 长时间无 IPv4，仍启动 BLE 广播" >&2
}

# 软等待：尽量等 ROS 核心，超时也不挡 GATT（否则 midware 挂掉时手机永远扫不到）
# 桥接侧仍会等 controller+midware 再建节点。
wait_ros2_stack_soft() {
  local wait_sec="${ROS2_SOFT_WAIT_SEC:-45}"
  local -a required=(
    hightorque_controller_node
    hightorque_midware_node
  )

  if command -v systemctl >/dev/null 2>&1; then
    if ! systemctl is-active --quiet ros2-bringup.service 2>/dev/null; then
      echo "[bird-ble] ros2-bringup 未 active，尝试 start…"
      systemctl start ros2-bringup.service 2>/dev/null || true
    fi
  fi

  echo "[bird-ble] 软等待 ROS2 核心（最多 ${wait_sec}s，超时仍启蓝牙）: ${required[*]}"
  local i missing p
  for i in $(seq 1 "${wait_sec}"); do
    missing=()
    for p in "${required[@]}"; do
      if ! pgrep -f "${p}" >/dev/null 2>&1; then
        missing+=("${p}")
      fi
    done
    if [ "${#missing[@]}" -eq 0 ]; then
      echo "[bird-ble] ROS2 核心已就绪 (wait ${i}s)"
      sleep 2
      return 0
    fi
    if [ $((i % 10)) -eq 0 ] || [ "${i}" -eq 1 ]; then
      echo "[bird-ble] 仍等待 ROS2 (${i}/${wait_sec}s)，缺少: ${missing[*]}"
    fi
    sleep 1
  done
  echo "[bird-ble] 警告: ${wait_sec}s 内未见 ${required[*]}，先启动 BLE 广播；ROS 桥后台继续等" >&2
  return 0
}

# ---------- 启动顺序：网络 →（软等 ROS）→ 蓝牙/GATT（可扫描优先）----------
wait_wifi_for_usb_bt

_PREPARE="$(cd "$(dirname "$0")/../../.." && pwd)/scripts/prepare-cyclonedds-runtime.sh"
_dds_iface="wlan0"
if [ -f "${BIRD_HOME}/cyclonedds.xml" ]; then
  _dds_iface="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${BIRD_HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
  _dds_iface="${_dds_iface:-wlan0}"
fi
if ip -4 addr show "${_dds_iface}" 2>/dev/null | grep -q "inet "; then
  if [ -x "${_PREPARE}" ] || [ -f "${_PREPARE}" ]; then
    # shellcheck disable=SC1090
    source "${_PREPARE}" || true
  fi
fi
_dds_runtime="${BIRD_HOME}/.config/bird/cyclonedds.runtime.xml"
if [ -f "${_dds_runtime}" ]; then
  export CYCLONEDDS_URI="file://${_dds_runtime}"
fi
echo "[bird-ble] CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
unset _PREPARE _dds_iface _dds_runtime

# 先起蓝牙射频，再软等 ROS（避免硬等 midware 导致完全无广播）
prep_bluetooth

wait_ros2_stack_soft

systemctl stop torque-cmd-vel.service 2>/dev/null || true
systemctl disable torque-cmd-vel.service 2>/dev/null || true
pkill -f 'locate_face_cpp/build/locate_face' 2>/dev/null || true
pkill -f 'locate_face\.py' 2>/dev/null || true
pkill -f 'face_yunet_worker' 2>/dev/null || true

if [ -x "${BT_DIR}/ensure_bfm_joy_mapper.sh" ]; then
  (
    # 等 midware/controller 再替换，避免空转
    for _ in $(seq 1 60); do
      if pgrep -f hightorque_controller_node >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
    sleep 2
    "${BT_DIR}/ensure_bfm_joy_mapper.sh" || true
  ) >/tmp/ensure_bfm_joy_mapper.boot.log 2>&1 &
fi

EXTRA_ARGS=()
if [ -f /etc/default/bird-ble ]; then
  # shellcheck disable=SC1091
  source /etc/default/bird-ble
fi

exec "${BT_DIR}/run_ble_with_ros.sh" "${EXTRA_ARGS[@]}"
