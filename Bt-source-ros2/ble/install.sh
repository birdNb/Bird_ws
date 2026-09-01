#!/bin/bash
# Bird BLE 一键安装/更新（仅改 Bt-source-ros2，不碰 hightorque_workspace / action_library 源码）
# 用法: cd ~/Bird_ws/Bt-source-ros2/ble && sudo ./install.sh
#
# ROS2 量产栈自启请另执行:
#   sudo ~/Bird_ws/Bt-source-ros2/scripts/install-ros2-autostart.sh
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
BIRD_WS="${BIRD_WS:-${INSTALL_HOME}/Bird_ws}"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"

if [ -z "${COLCON_WS:-}" ] || [ ! -f "${COLCON_WS}/install/setup.bash" ]; then
  COLCON_WS=""
  for _ws in \
    "${INSTALL_HOME}/hightorque_workspace" \
    "${INSTALL_HOME}/colcon_ws"; do
    if [ -f "${_ws}/install/setup.bash" ]; then
      COLCON_WS="${_ws}"
      break
    fi
  done
  COLCON_WS="${COLCON_WS:-${INSTALL_HOME}/hightorque_workspace}"
fi
unset _ws

echo "=========================================="
echo " Bird BLE ROS2 遥控安装/更新 (Bt-source-ros2)"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " ROS2 工作空间(只读引用): ${COLCON_WS}"
echo "=========================================="

echo "[1/5] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  rfkill

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "[warn] 未检测到 ROS2 Foxy，请先安装 ros-foxy-ros-base"
else
  echo "[ok] ROS2 Foxy 已安装"
fi

echo "[2/5] 配置 BlueZ Experimental..."
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

echo "[3/5] 写入环境 /etc/default/bird-ble ..."
cat >/etc/default/bird-ble <<EOF
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
BIRD_BLE_UID=${INSTALL_UID}
BIRD_WS=${BIRD_WS}
COLCON_WS=${COLCON_WS}
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://${INSTALL_HOME}/cyclonedds.xml
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
export COLCON_WS="${COLCON_WS}"
export PKG_DIR="${PKG_DIR}"

chown "${INSTALL_USER}:${INSTALL_GID}" "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/scripts/"*.sh "${APP_DIR}/systemd/"*.sh 2>/dev/null || true
chmod +x "${APP_DIR}/ensure_bfm_joy_mapper.sh" "${APP_DIR}/joy_mapper_bfm_fix.py" 2>/dev/null || true

echo "[4/5] 安装 systemd 开机自启 (bird-ble)..."
"${APP_DIR}/install-autostart.sh"

echo "[5/5] 重启 bird-ble 使本目录代码生效..."
set +u
[ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash
[ -f "${COLCON_WS}/install/setup.bash" ] && source "${COLCON_WS}/install/setup.bash"
set -u

python3 -c "import rclpy" 2>/dev/null && echo "[ok] rclpy 可导入" \
  || echo "[warn] 无法 import rclpy — 检查 ROS2 Foxy"

systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 3

# bird-ble-boot 内也会 ensure；此处再补一次，覆盖手动 launch 已拉起的 joy_mapper
"${APP_DIR}/ensure_bfm_joy_mapper.sh" || true

echo ""
echo "=========================================="
echo " BLE 安装/更新完成（未改 hightorque_workspace 源码）"
echo "=========================================="
echo " 以后更新 BLE:  cd ${PKG_DIR} && sudo ./install.sh"
echo " ROS2 开机自启: sudo ${BIRD_WS}/Bt-source-ros2/scripts/install-ros2-autostart.sh"
echo " 状态: sudo systemctl status bird-ble"
echo " 日志: journalctl -u bird-ble -f"
"${APP_DIR}/scripts/check.sh" || true
