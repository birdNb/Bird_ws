#!/bin/bash
# 力矩拖拽控制 一键安装：systemd 开机自启
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="${PKG_DIR}/app"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"

echo "=========================================="
echo " pull_move 力矩→cmd_vel 安装包"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " ROS 工作空间: ${SIM2REAL_WS}"
echo "=========================================="

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "[warn] 未检测到 ROS Noetic，请先安装 ROS 与 sim2real"
else
  echo "[ok] ROS Noetic 已安装"
fi

export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"

chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/systemd/"*.sh

echo "[1/2] 安装 systemd 开机自启..."
"${APP_DIR}/install-autostart.sh"

echo "[2/2] 检查 sim2real..."
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/install"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/devel"
else
  echo "[warn] 未找到 sim2real（${SIM2REAL_WS}）"
fi

systemctl restart torque-cmd-vel.service || systemctl start torque-cmd-vel.service || true
sleep 3

echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 状态: sudo systemctl status torque-cmd-vel"
echo " 日志: journalctl -u torque-cmd-vel -f"
echo " 手动: cd ${APP_DIR} && ./run_torque_bridge.sh"
echo ""
echo " 仅 FSM=行走模式(EXEC_DEFAULT) 时发布 /cmd_vel"
