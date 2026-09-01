#!/bin/bash
# 安装 Bird BLE 开机自启动（systemd）
set -euo pipefail

BT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${BT_DIR}/platform_env.sh"
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

# 持久化蓝牙名（rename 写入 /var/lib/bird-ble/，重启不丢失）
PERSIST_NAME="/var/lib/bird-ble/ble_device_name.conf"
mkdir -p /var/lib/bird-ble
chmod 755 /var/lib/bird-ble
if [ ! -f "${PERSIST_NAME}" ] && [ -f "${PKG_DIR}/ble_device_name.conf" ]; then
  _n="$(head -1 "${PKG_DIR}/ble_device_name.conf" | tr -d '[:space:]')"
  if [ -n "${_n}" ] && [ "${_n}" != "HT_88888888" ]; then
    echo "${_n}" >"${PERSIST_NAME}"
    chmod 644 "${PERSIST_NAME}"
    echo "[ok] 已迁移广播名 → ${PERSIST_NAME}: ${_n}"
  fi
fi
unset PERSIST_NAME _n

# 首次安装：确保 BlueZ Experimental（否则 GATT/广播注册会失败）
if [ -f /etc/bluetooth/main.conf ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' /etc/bluetooth/main.conf 2>/dev/null; then
  echo "[setup] 配置 BlueZ Experimental=true ..."
  su - "${BIRD_USER}" -c "cd '${BT_DIR}' && ./start.sh --setup" || true
fi

if ! python3 -c "import dbus; from gi.repository import GLib" 2>/dev/null; then
  echo "[error] 缺少依赖: sudo apt install -y bluez python3-dbus python3-gi"
  exit 1
fi

sed \
  -e "s|@BT_DIR@|${BT_DIR}|g" \
  -e "s|@PKG_DIR@|${PKG_DIR}|g" \
  -e "s|@BIRD_WS@|${BIRD_WS}|g" \
  -e "s|@BIRD_USER@|${BIRD_USER}|g" \
  -e "s|@BIRD_HOME@|${BIRD_HOME}|g" \
  -e "s|@BIRD_BLE_UID@|${BIRD_BLE_UID}|g" \
  -e "s|@COLCON_WS@|${COLCON_WS}|g" \
  "${UNIT_SRC}" >"${UNIT_DST}"

if [ ! -f "${DEFAULT_ENV}" ]; then
  cat >"${DEFAULT_ENV}" <<EOF
# Bird BLE 环境（install-autostart 维护）
BLE_DEVICE_NAME_FILE=/var/lib/bird-ble/ble_device_name.conf
# 等待 ROS2 量产栈就绪的最长时间（秒）
ROS2_WAIT_SEC=300
EXTRA_ARGS=()
EOF
  echo "[ok] 已创建 ${DEFAULT_ENV}"
elif ! grep -q '^BLE_DEVICE_NAME_FILE=' "${DEFAULT_ENV}" 2>/dev/null; then
  echo 'BLE_DEVICE_NAME_FILE=/var/lib/bird-ble/ble_device_name.conf' >>"${DEFAULT_ENV}"
  echo "[ok] 已向 ${DEFAULT_ENV} 追加 BLE_DEVICE_NAME_FILE"
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
