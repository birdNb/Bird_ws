#!/bin/bash
# 安装 ROS2 量产栈开机自启（systemd ros2-bringup）
# 不修改 hightorque_workspace / action_library 源码；启动命令写死如下。
#
# 用法: sudo ./scripts/install-ros2-autostart.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_USER="${SUDO_USER:-nvidia}"
if [ "${INSTALL_USER}" = "root" ]; then
  INSTALL_USER="nvidia"
fi
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
WS="/home/nvidia/hightorque_workspace"
AUTOSTART_DIR="${INSTALL_HOME}/.config/autostart"
UNIT_DST="/etc/systemd/system/ros2-bringup.service"
DEFAULT_ENV="/etc/default/ros2-bringup"
BOOT_SRC="${SCRIPT_DIR}/ros2-bringup-boot.sh"
UNIT_SRC="${SCRIPT_DIR}/ros2-bringup.service"

LAUNCH_CMD=$(cat <<'EOF'
cd /home/nvidia/hightorque_workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch hightorque_bringup bfm_real.launch.py \
  auto_stand:=true \
  auto_start_bfm:=false \
  enable_gamepad_commands:=true \
  enable_fall_detector:=false \
  enable_auto_fall_recovery:=false
EOF
)

echo "========== 1) 清理旧 ROS2 自启（桌面/systemd） =========="

systemctl stop ros2-bringup.service 2>/dev/null || true
systemctl disable ros2-bringup.service 2>/dev/null || true
rm -f "${UNIT_DST}" "${DEFAULT_ENV}"
systemctl reset-failed ros2-bringup.service 2>/dev/null || true

mkdir -p "${AUTOSTART_DIR}"
for desk in \
  "${AUTOSTART_DIR}/Pi_plus_ros2.desktop" \
  "${AUTOSTART_DIR}/hightorque_ros2_bfm.desktop" \
  "${AUTOSTART_DIR}/Pi_plus_start.desktop"
do
  if [ -f "${desk}" ]; then
    rm -f "${desk}"
    echo "[ok] 已删除桌面自启: ${desk}"
  fi
done

# 清残留进程，避免和新服务抢电机/DDS（不改工作空间文件）
pkill -f 'ros2 launch hightorque_bringup bfm_real' 2>/dev/null || true
pkill -f hightorque_controller_node 2>/dev/null || true
pkill -f hightorque_midware_node 2>/dev/null || true
sleep 2

echo "========== 2) 安装 systemd（命令写死在 boot 脚本） =========="

if [ ! -f "${WS}/install/setup.bash" ]; then
  echo "[err] 缺少 ${WS}/install/setup.bash" >&2
  exit 1
fi
if [ ! -f "${BOOT_SRC}" ] || [ ! -f "${UNIT_SRC}" ]; then
  echo "[err] 缺少 ${BOOT_SRC} 或 ${UNIT_SRC}" >&2
  exit 1
fi

chmod 0755 "${BOOT_SRC}" "${SCRIPT_DIR}/ros2_start.sh" 2>/dev/null || true
install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"

cat >"${DEFAULT_ENV}" <<EOF
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
COLCON_WS=${WS}
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://${INSTALL_HOME}/cyclonedds.xml
EOF
chmod 644 "${DEFAULT_ENV}"

systemctl daemon-reload
systemctl enable ros2-bringup.service
systemctl restart ros2-bringup.service
sleep 4

if systemctl is-active --quiet ros2-bringup.service; then
  echo "[ok] ros2-bringup 已启动"
else
  echo "[warn] ros2-bringup 未 active，请查: journalctl -u ros2-bringup -n 40 --no-pager" >&2
fi

# BLE 依赖 ROS2；若已安装则重启以重新挂桥
if systemctl list-unit-files bird-ble.service 2>/dev/null | grep -q bird-ble; then
  systemctl restart bird-ble.service 2>/dev/null || true
  echo "[ok] 已重启 bird-ble"
fi

echo ""
echo "自启等价命令（与 boot 脚本一致）:"
echo "${LAUNCH_CMD}"
echo ""
echo "检查: systemctl status ros2-bringup"
echo "日志: journalctl -u ros2-bringup -f"
echo "手动: ${SCRIPT_DIR}/ros2_start.sh"
