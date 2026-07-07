#!/bin/bash
# Bird BLE 遥控服务启动
set -euo pipefail
cd "$(dirname "$0")"

# shellcheck disable=SC1091
source "$(pwd)/platform_env.sh"
source "$(pwd)/platform_hw.sh"

SUDO_PW="${BIRD_BLE_SUDO_PW:-${BIRD_USER:-hightorque}}"

sudo_n() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    echo "$SUDO_PW" | sudo -S -p '' "$@"
  fi
}

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项]

启动 BLE GATT 从机，将小程序指令转发至 ROS。

小程序连接参数（复制到微信开发者工具）:
  设备名: HT_88888888  （可用 rename HT_12345678 修改后8位）
  服务 FFE0:  0000FFE0-0000-1000-8000-00805F9B34FB  ← 扫描必用
  写入 FFE1:  0000FFE1-0000-1000-8000-00805F9B34FB
  通知 FFE2:  0000FFE2-0000-1000-8000-00805F9B34FB
  板子 MAC 参考:  00:19:86:00:2E:AF

重要: 只在【微信小程序】里连接，不要在手机系统蓝牙里点配对
      若手机反复弹连接框：设置里「忽略」设备后重试
      必须保持本脚本运行，小程序才能发现 BLE 广播

选项:
  --name NAME     广播名称 (默认 ble_device_name.conf 或 HT_88888888)
  --no-echo       不回显 ACK 到 notify 特征
  --no-ros        不转发 ROS（仅验证 BLE 连接）
  --enable-voice  启用 FFE3 语音 PCM 播放（sound_demo）
  --setup         尝试开启 BlueZ Experimental 并重启 bluetooth
  -h, --help      显示帮助

开机自启动:
  sudo ./install-autostart.sh          # 安装 systemd 服务 bird-ble
  sudo systemctl status bird-ble     # 查看状态
  journalctl -u bird-ble -f          # 查看日志

示例:
  ./start.sh
  ./start.sh --setup
  sudo ./start.sh --name MyRobot

注意:
  - 注册 GATT 通常需要 root（脚本会自动 sudo）
  - 需已安装: bluez python3-dbus python3-gi
  - 默认启动仅调整蓝牙运行时状态，不修改系统配置，不影响开机
  - --setup 会修改 /etc/bluetooth/main.conf（自动备份），仅首次配置时需要
EOF
}

NAME="$(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())")"
EXTRA_ARGS=()
DO_SETUP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      show_help
      exit 0
      ;;
    --setup)
      DO_SETUP=1
      shift
      ;;
    --name)
      NAME="${2:-HT_88888888}"
      EXTRA_ARGS+=(--name "$NAME")
      shift 2
      ;;
    --no-echo)
      EXTRA_ARGS+=(--no-echo)
      shift
      ;;
    --no-ros)
      EXTRA_ARGS+=(--no-ros)
      shift
      ;;
    --enable-voice)
      EXTRA_ARGS+=(--enable-voice)
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ "$DO_SETUP" -eq 1 ]; then
  echo "[setup] 检查 BlueZ Experimental..."
  CONF="/etc/bluetooth/main.conf"
  if [ -f "$CONF" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "$CONF" 2>/dev/null; then
    BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
    sudo_n cp "$CONF" "$BACKUP"
    echo "[setup] 已备份: $BACKUP"
    if grep -q '^\[General\]' "$CONF"; then
      sudo_n sed -i '/^\[General\]/a Experimental=true' "$CONF"
      echo "[setup] 已添加 Experimental=true"
    else
      printf '%s\n' '[General]' 'Experimental=true' | sudo_n tee -a "$CONF" >/dev/null
      echo "[setup] 已追加 [General] Experimental=true"
    fi
    if ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "$CONF" 2>/dev/null; then
      echo "[error] 写入 $CONF 失败，正在从备份恢复..."
      sudo_n cp "$BACKUP" "$CONF"
      exit 1
    fi
    sudo_n systemctl restart bluetooth
    sleep 2
  else
    echo "[setup] 已配置或无法修改 $CONF"
  fi
  sudo_n hciconfig hci0 up 2>/dev/null || true
  sudo_n hciconfig hci0 name "${NAME}" 2>/dev/null || true
  bluetoothctl power on 2>/dev/null || true
  bluetoothctl discoverable off 2>/dev/null || true
  bluetoothctl pairable off 2>/dev/null || true
  bluetoothctl system-alias "${NAME}" 2>/dev/null || true
fi

if ! systemctl is-active --quiet bluetooth 2>/dev/null; then
  echo "[warn] bluetooth 服务未运行，尝试启动..."
  sudo_n systemctl start bluetooth || true
  sleep 1
fi

# 启动前检查蓝牙适配器
_BLE_DEV="${BLE_HCI_DEV:-hci0}"
if ! hciconfig "${_BLE_DEV}" 2>/dev/null | grep -q "${_BLE_DEV}"; then
  echo "[error] 未检测到蓝牙 ${_BLE_DEV}"
  echo "  平台: ${BLE_PLATFORM} | ${BLE_HW_DESC}"
  if [ "${BLE_BT_KIND:-}" = "usb_dongle" ]; then
    echo "  Orin: 请确认 USB 蓝牙模块已插入"
  else
    echo "  RK: 板载 RTL8822CE 蓝牙未就绪"
  fi
  echo "  恢复: sudo ./scripts/recover.sh"
  exit 1
fi
unset _BLE_DEV

echo "平台: ${BLE_PLATFORM} | ${BLE_HW_DESC} | 广播: ${BLE_ADV_MODE}"

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "缺少依赖，请执行:"
  echo "  sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

chmod +x ble_gatt_server.py ble_ros_bridge.py ble_neck_bridge.py ble_motor_power_manager.py ble_locate_face_manager.py ble_command_dispatcher.py ble_status_telemetry.py ble_log.py neck_smooth_home.py run_ble_with_ros.sh ros_env.sh

# ROS 环境（sim2real_msg 在 install 目录，见 ros_env.sh）
# shellcheck disable=SC1091
source "$(pwd)/ros_env.sh"

if ! python3 -c "import rospy" 2>/dev/null; then
  echo "[error] 本机无法 import rospy，BLE 模式指令将无法控制机器人"
  echo "  请执行: source /opt/ros/noetic/setup.bash"
  echo "  或加 --no-ros 仅做 BLE 打印测试"
  exit 1
fi
if ! python3 -c "import sim2real_msg" 2>/dev/null; then
  echo "[warn] 无法 import sim2real_msg — M_* 模式切换需要此包"
  echo "  请执行: source ~/sim2real/install/setup.bash"
fi

# BLE 广播名 + 关闭经典蓝牙配对（避免手机系统反复弹窗）
sudo_n hciconfig hci0 up 2>/dev/null || true
sudo_n hciconfig hci0 name "${NAME}" 2>/dev/null || true
sudo_n hciconfig hci0 noscan 2>/dev/null || true
bluetoothctl system-alias "${NAME}" 2>/dev/null || true
bluetoothctl discoverable off 2>/dev/null || true
bluetoothctl pairable off 2>/dev/null || true
if command -v btmgmt >/dev/null && hciconfig hci0 2>/dev/null | grep -q "hci0"; then
  sudo_n btmgmt -i 0 le on 2>/dev/null || true
  sudo_n btmgmt -i 0 connectable on 2>/dev/null || true
  sudo_n btmgmt -i 0 discov off 2>/dev/null || true
  sudo_n btmgmt -i 0 pairable off 2>/dev/null || true
  sudo_n btmgmt -i 0 bondable off 2>/dev/null || true
fi

MAC=$(bluetoothctl show 2>/dev/null | awk '/Controller/ {print $2}' | head -1)
echo "========================================"
echo " BLE 遥控 | 广播名: $NAME"
echo " MAC: ${MAC:-见 bluetoothctl show}"
echo " 协议: BLE_PROTOCOL.md | 参考: docs/miniprogram_ble_snippet.js"
echo " 诊断: ./scripts/check.sh"
echo "========================================"

if [ "$(id -u)" -ne 0 ]; then
  echo "[tip] 使用 sudo 注册 GATT 服务（经 run_ble_with_ros.sh 加载 ROS）..."
  echo "$SUDO_PW" | sudo -S -p '' -E "$(pwd)/run_ble_with_ros.sh" --name "$NAME" "${EXTRA_ARGS[@]}"
  exit $?
fi

exec "$(pwd)/run_ble_with_ros.sh" --name "$NAME" "${EXTRA_ARGS[@]}"
