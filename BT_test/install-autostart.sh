#!/bin/bash
# 安装 Bird BLE 开机自启动（systemd）
set -euo pipefail

BT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_NAME="bird-ble.service"
UNIT_SRC="${BT_DIR}/systemd/bird-ble.service"
BOOT_SRC="${BT_DIR}/systemd/bird-ble-boot.sh"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_ENV="/etc/default/bird-ble"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0"
  exit 1
fi

chmod +x "${BOOT_SRC}" "${BT_DIR}/run_ble_with_ros.sh" "${BT_DIR}/start.sh"

# 首次安装：确保 BlueZ Experimental（否则 GATT/广播注册会失败）
if [ -f /etc/bluetooth/main.conf ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' /etc/bluetooth/main.conf 2>/dev/null; then
  echo "[setup] 配置 BlueZ Experimental=true ..."
  su - nvidia -c "cd '${BT_DIR}' && ./start.sh --setup" || true
fi

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "[error] 缺少依赖: sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

cp -f "${UNIT_SRC}" "${UNIT_DST}"

if [ ! -f "${DEFAULT_ENV}" ]; then
  cat >"${DEFAULT_ENV}" <<'EOF'
# Bird BLE 额外启动参数（传给 run_ble_with_ros.sh）
# 示例：开启语音 FFE3 备用通道
# EXTRA_ARGS=(--enable-voice)
EXTRA_ARGS=()
EOF
  echo "[ok] 已创建 ${DEFAULT_ENV}"
fi

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo ""
echo "=========================================="
echo " Bird BLE 已设置开机自启动"
echo " 服务名: ${SERVICE_NAME}"
echo "=========================================="
echo ""
echo "常用命令:"
echo "  sudo systemctl start ${SERVICE_NAME}    # 立即启动"
echo "  sudo systemctl stop ${SERVICE_NAME}     # 停止"
echo "  sudo systemctl status ${SERVICE_NAME}   # 状态"
echo "  journalctl -u ${SERVICE_NAME} -f        # 日志"
echo ""
echo "首次若 GATT 注册失败，请先执行一次:"
echo "  cd ${BT_DIR} && ./start.sh --setup"
echo ""
echo "修改启动参数: 编辑 ${DEFAULT_ENV} 后"
echo "  sudo systemctl daemon-reload"
echo "  sudo systemctl restart ${SERVICE_NAME}"
