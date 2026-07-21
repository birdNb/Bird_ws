#!/bin/bash
# Bt_voice_pull_bag_0721 — 蓝牙 + 语音提醒 + 拖拽控制 一键安装
# 安装前自动清理旧服务与残留单元
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
BLE_DIR="${PKG_DIR}/ble"
PULL_DIR="${PKG_DIR}/pull_move"
VOICE_DIR="${PKG_DIR}/voice_remind"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"
# 语音包在本集成包根目录；BLE 通过 BIRD_WS/voice_remind 导入
BIRD_WS="${BIRD_WS:-${PKG_DIR}}"

cleanup_residuals() {
  echo "[0/4] 清理旧服务与残留..."
  local units=(
    bird-ble.service
    torque-cmd-vel.service
    bird-ble-boot.service
    bt-control.service
    ble-gatt.service
  )
  local u
  for u in "${units[@]}"; do
    systemctl stop "${u}" 2>/dev/null || true
    systemctl disable "${u}" 2>/dev/null || true
    rm -f "/etc/systemd/system/${u}"
    rm -f "/lib/systemd/system/${u}"
    rm -f "/usr/lib/systemd/system/${u}"
  done

  # 旧进程兜底（单元已删但进程仍在）
  pkill -f 'ble_gatt_server\.pyc?' 2>/dev/null || true
  pkill -f 'torque_cmd_vel_bridge\.pyc?' 2>/dev/null || true

  # 旧默认环境文件（安装阶段会重写）
  rm -f /etc/default/bird-ble /etc/default/torque-cmd-vel

  systemctl daemon-reload 2>/dev/null || true
  systemctl reset-failed 2>/dev/null || true
  echo "[ok] 残留服务已清理"
}

echo "=========================================="
echo " Bt_voice_pull_bag_0721 一键安装"
echo " 蓝牙 + 语音提醒 + 拖拽控制"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " BIRD_WS:  ${BIRD_WS}"
echo " sim2real: ${SIM2REAL_WS}"
echo "=========================================="

for d in "${BLE_DIR}" "${PULL_DIR}" "${VOICE_DIR}"; do
  if [ ! -d "${d}" ]; then
    echo "[error] 缺少子包: ${d}"
    exit 1
  fi
done
if [ ! -f "${VOICE_DIR}/__init__.py" ] && [ ! -f "${VOICE_DIR}/__init__.pyc" ]; then
  echo "[error] voice_remind 不完整: ${VOICE_DIR}"
  exit 1
fi

cleanup_residuals

echo "[1/4] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  pulseaudio-utils \
  rfkill

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
  echo "[ok] BlueZ Experimental=true（备份: ${BACKUP}）"
fi

echo "[2/4] 安装蓝牙 BLE（含语音提醒加载路径）..."
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
export BIRD_BLE_UID="${INSTALL_UID}"
export BIRD_WS="${BIRD_WS}"
export SIM2REAL_WS="${SIM2REAL_WS}"
export PKG_DIR="${BLE_DIR}"
export BT_DIR="${BLE_DIR}/app"

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
if [ ! -f /var/lib/bird-ble/ble_device_name.conf ] && [ -f "${BLE_DIR}/ble_device_name.conf" ]; then
  _bn="$(tr -d '[:space:]' < "${BLE_DIR}/ble_device_name.conf")"
  if [ -n "${_bn}" ]; then
    echo "${_bn}" >/var/lib/bird-ble/ble_device_name.conf
    chmod 644 /var/lib/bird-ble/ble_device_name.conf
    echo "[ok] 广播名持久化: ${_bn}"
  fi
fi
unset _bn

chown "${INSTALL_USER}:${INSTALL_GID}" "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${BLE_DIR}/app"/*.sh "${BLE_DIR}/app/scripts/"*.sh "${BLE_DIR}/app/systemd/"*.sh

"${BLE_DIR}/app/install-autostart.sh"

echo "[3/4] 安装拖拽控制（默认不开机自启，由小程序 PULL ON 启动）..."
chmod +x "${PULL_DIR}/app"/*.sh "${PULL_DIR}/app/systemd/"*.sh
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
"${PULL_DIR}/app/install-autostart.sh"
# 确保拖拽不随开机自动跑（避免与站立态冲突）
systemctl disable torque-cmd-vel.service 2>/dev/null || true
systemctl stop torque-cmd-vel.service 2>/dev/null || true

echo "[4/4] 启动蓝牙服务并自检..."
systemctl daemon-reload
systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 5

set +u
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f "${SIM2REAL_WS}/install/setup.bash" ] && source "${SIM2REAL_WS}/install/setup.bash"
[ -f "${SIM2REAL_WS}/devel/setup.bash" ] && source "${SIM2REAL_WS}/devel/setup.bash"
set -u

python3 -c "import sim2real_msg" 2>/dev/null && echo "[ok] sim2real_msg" \
  || echo "[warn] 无法 import sim2real_msg — 检查 SIM2REAL_WS"
python3 -c "import sys; sys.path.insert(0, '${BIRD_WS}'); import voice_remind; print('[ok] voice_remind')" \
  || echo "[warn] voice_remind 导入失败"

echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 蓝牙:   sudo systemctl status bird-ble"
echo " 拖拽:   sudo systemctl status torque-cmd-vel  # PULL ON 后运行"
echo " 语音:   ${VOICE_DIR}  （由 bird-ble 加载）"
echo " 日志:   journalctl -u bird-ble -f"
"${BLE_DIR}/app/scripts/check.sh" || true
