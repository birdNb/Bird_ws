#!/bin/bash
# BLE 蓝牙接收测试：供手机/微信小程序连接并发送数据
set -euo pipefail
cd "$(dirname "$0")"

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项]

启动 BLE GATT 从机，打印手机/小程序写入的数据。

小程序连接参数（复制到微信开发者工具）:
  设备名: Bird_BLE_Test  （可能扫不到名称，请用下面 UUID 扫描）
  服务 serviceId:  0000FFF0-0000-1000-8000-00805F9B34FB  ← 扫描必用
  写入特征 UUID:  0000FFF1-0000-1000-8000-00805F9B34FB
  通知特征 UUID:  0000FFF2-0000-1000-8000-00805F9B34FB  (可选)
  板子 MAC 参考:  00:19:86:00:2E:AF

重要: 手机【系统蓝牙】能配对 ≠ 小程序能扫到
      必须保持本脚本运行，小程序才能发现 BLE 广播
      EDIFIER BLE 是附近耳机，不是你的板子

选项:
  --name NAME     广播名称 (默认 Bird_BLE_Test)
  --no-echo       不回显 ACK 到 notify 特征
  --setup         尝试开启 BlueZ Experimental 并重启 bluetooth
  -h, --help      显示帮助

示例:
  ./start.sh
  ./start.sh --setup
  sudo ./start.sh --name MyRobot

注意:
  - 注册 GATT 通常需要 root（脚本会自动 sudo）
  - 需已安装: bluez python3-dbus python3-gi
EOF
}

NAME="Bird_BLE_Test"
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
      NAME="${2:-Bird_BLE_Test}"
      EXTRA_ARGS+=(--name "$NAME")
      shift 2
      ;;
    --no-echo)
      EXTRA_ARGS+=(--no-echo)
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
  if [ -f "$CONF" ] && ! grep -q '^Experimental=true' "$CONF" 2>/dev/null; then
    if grep -q '^\[General\]' "$CONF"; then
      sudo sed -i '/^\[General\]/a Experimental=true' "$CONF"
      echo "[setup] 已添加 Experimental=true"
    else
      echo -e "[General]\nExperimental=true" | sudo tee -a "$CONF" >/dev/null
      echo "[setup] 已追加 [General] Experimental=true"
    fi
    sudo systemctl restart bluetooth
    sleep 2
  else
    echo "[setup] 已配置或无法修改 $CONF"
  fi
  sudo hciconfig hci0 up 2>/dev/null || true
  sudo hciconfig hci0 name "Bird_BLE_Test" 2>/dev/null || true
  bluetoothctl power on 2>/dev/null || true
  bluetoothctl discoverable on 2>/dev/null || true
  bluetoothctl pairable on 2>/dev/null || true
  bluetoothctl system-alias "Bird_BLE_Test" 2>/dev/null || true
fi

if ! systemctl is-active --quiet bluetooth 2>/dev/null; then
  echo "[warn] bluetooth 服务未运行，尝试启动..."
  sudo systemctl start bluetooth || true
  sleep 1
fi

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "缺少依赖，请执行:"
  echo "  sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

chmod +x ble_gatt_server.py

# 确保广播名为 Bird_BLE_Test（与小程序扫描名一致）
sudo hciconfig hci0 name "Bird_BLE_Test" 2>/dev/null || true
bluetoothctl system-alias "Bird_BLE_Test" 2>/dev/null || true

MAC=$(bluetoothctl show 2>/dev/null | awk '/Controller/ {print $2}' | head -1)
echo "========================================"
echo " BLE 测试 | 广播名: $NAME"
echo " MAC: ${MAC:-见 bluetoothctl show}"
echo " 小程序请用 services=[FFF0] 扫描，见 miniprogram_ble_snippet.js"
echo " 诊断: ./ble_check.sh"
echo "========================================"

if [ "$(id -u)" -ne 0 ]; then
  echo "[tip] 使用 sudo 注册 GATT 服务..."
  exec sudo -E python3 "$(pwd)/ble_gatt_server.py" --name "$NAME" "${EXTRA_ARGS[@]}"
fi

exec python3 ble_gatt_server.py --name "$NAME" "${EXTRA_ARGS[@]}"
