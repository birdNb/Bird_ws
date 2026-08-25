#!/bin/bash
# 开机跑 ROS2 量产算法（AMP）；关闭 ROS1 sim2real 步态自启。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
if [ "${INSTALL_USER}" = "root" ]; then
  INSTALL_USER="hightorque"
fi
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}")"
INSTALL_GID="$(id -g "${INSTALL_USER}")"
COLCON_WS="${COLCON_WS:-${INSTALL_HOME}/colcon_ws}"
AUTOSTART_DIR="${INSTALL_HOME}/.config/autostart"
SRC_START="${SCRIPT_DIR}/ros2_start.sh"
if [ ! -f "${SRC_START}" ]; then
  SRC_START="${SCRIPT_DIR}/ros2_start.sh"
fi
DST_START="${COLCON_WS}/ros2_start.sh"
ROS1_DESKTOP="${AUTOSTART_DIR}/Pi_plus_start.desktop"
ROS2_DESKTOP="${AUTOSTART_DIR}/Pi_plus_ros2.desktop"

mkdir -p "${COLCON_WS}" "${AUTOSTART_DIR}"
install -m 0755 "${SRC_START}" "${DST_START}"
chown "${INSTALL_UID}:${INSTALL_GID}" "${DST_START}"

if [ -f "${ROS1_DESKTOP}" ]; then
  if grep -q '^Hidden=' "${ROS1_DESKTOP}"; then
    sed -i 's/^Hidden=.*/Hidden=true/' "${ROS1_DESKTOP}"
  else
    printf '\nHidden=true\n' >>"${ROS1_DESKTOP}"
  fi
  chown "${INSTALL_UID}:${INSTALL_GID}" "${ROS1_DESKTOP}"
  echo "[ok] 已关闭 ROS1 步态自启: ${ROS1_DESKTOP}"
else
  echo "[warn] 未找到 ROS1 自启 ${ROS1_DESKTOP}"
fi

NEWTRAJ="${AUTOSTART_DIR}/pi plus newTraj.desktop"
if [ -f "${NEWTRAJ}" ]; then
  if grep -q '^Hidden=' "${NEWTRAJ}"; then
    sed -i 's/^Hidden=.*/Hidden=true/' "${NEWTRAJ}"
  else
    printf '\nHidden=true\n' >>"${NEWTRAJ}"
  fi
fi

# 关闭 ROS1 群控（noetic / ROS_MASTER_URI）
if [ -f "${INSTALL_HOME}/.config/systemd/user/robot-control.service" ]; then
  sudo -u "${INSTALL_USER}" XDG_RUNTIME_DIR="/run/user/${INSTALL_UID}" \
    systemctl --user disable --now robot-control.service >/dev/null 2>&1 || true
  echo "[ok] 已关闭 ROS1 robot-control.service"
fi

cat >"${ROS2_DESKTOP}" <<EOF
[Desktop Entry]
Encoding=UTF-8
Version=0.9.4
Type=Application
Name=Pi_plus_ros2
Comment=量产 ROS2 bringup（AMP，不启动 BFM）
Exec=${DST_START}
OnlyShowIn=XFCE;
RunHook=0
StartupNotify=false
Terminal=false
Hidden=false
EOF
chown "${INSTALL_UID}:${INSTALL_GID}" "${ROS2_DESKTOP}"
echo "[ok] 已安装 ROS2 开机自启: ${ROS2_DESKTOP}"
echo "    启动脚本: ${DST_START}"
echo "    ROS1 Pi_plus_start / robot-control 已关闭"
