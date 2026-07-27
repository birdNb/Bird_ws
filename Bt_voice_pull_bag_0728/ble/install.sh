#!/bin/bash
# Bird BLE 一键安装：系统依赖 + 开机自启
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="${PKG_DIR}/app"
cd "${PKG_DIR}"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"
BIRD_WS="${BIRD_WS:-${INSTALL_HOME}/Bird_ws}"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"

echo "=========================================="
echo " Bird BLE 遥控安装包 (BT_Control_0717_bag)"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " ROS 工作空间: ${SIM2REAL_WS}"
echo "=========================================="

echo "[1/4] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  rfkill

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "[warn] 未检测到 ROS Noetic，请先安装 ROS 与 sim2real_msg"
else
  echo "[ok] ROS Noetic 已安装"
fi

echo "[2/4] 配置 BlueZ Experimental..."
CONF="/etc/bluetooth/main.conf"
if [ -f "${CONF}" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "${CONF}" 2>/dev/null; then
  BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
  cp "${CONF}" "${BACKUP}"
  if grep -q '^\[General\]' "${CONF}"; then
    sed -i '/^\[General\]/a Experimental=true' "${CONF}"
  else
    printf '%s\n' '[General]' 'Experimental=true' >>"${CONF}"
  fi
  systemctl restart bluetooth
  sleep 2
  echo "[ok] 已启用 Experimental=true（备份: ${BACKUP}）"
fi

echo "[3/4] 写入环境 /etc/default/bird-ble ..."
cat >/etc/default/bird-ble <<EOF
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
BIRD_BLE_UID=${INSTALL_UID}
BIRD_WS=${BIRD_WS}
SIM2REAL_WS=${SIM2REAL_WS}
BLE_DEVICE_NAME_FILE=/var/lib/bird-ble/ble_device_name.conf
EXTRA_ARGS=()
EOF

mkdir -p /var/lib/bird-ble
chmod 755 /var/lib/bird-ble
if [ ! -f /var/lib/bird-ble/ble_device_name.conf ] && [ -f "${PKG_DIR}/ble_device_name.conf" ]; then
  _bn="$(tr -d '[:space:]' < "${PKG_DIR}/ble_device_name.conf")"
  if [ -n "${_bn}" ]; then
    echo "${_bn}" > /var/lib/bird-ble/ble_device_name.conf
    chmod 644 /var/lib/bird-ble/ble_device_name.conf
    echo "[ok] 广播名持久化: /var/lib/bird-ble/ble_device_name.conf (${_bn})"
  fi
fi
unset _bn

export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
export BIRD_BLE_UID="${INSTALL_UID}"
export BIRD_WS="${BIRD_WS}"
export SIM2REAL_WS="${SIM2REAL_WS}"

chown "${INSTALL_USER}:${INSTALL_GID}" "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/scripts/"*.sh "${APP_DIR}/systemd/"*.sh

echo "[4/5] 安装 systemd 开机自启..."
"${APP_DIR}/install-autostart.sh"

echo "[5/5] 检查 ROS / sim2real 环境..."
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/install"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/devel"
else
  echo "[warn] 未找到 sim2real 工作空间（${SIM2REAL_WS}）"
fi

set +u
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f "${SIM2REAL_WS}/install/setup.bash" ] && source "${SIM2REAL_WS}/install/setup.bash"
[ -f "${SIM2REAL_WS}/devel/setup.bash" ] && source "${SIM2REAL_WS}/devel/setup.bash"
set -u

python3 -c "import sim2real_msg" 2>/dev/null && echo "[ok] sim2real_msg 可导入" \
  || echo "[warn] 无法 import sim2real_msg — 检查 SIM2REAL_WS 路径"

systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 5
echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 状态: sudo systemctl status bird-ble"
echo " 日志: journalctl -u bird-ble -f"
echo " 手动: cd ${APP_DIR} && ./start.sh"
"${APP_DIR}/scripts/check.sh" || true
