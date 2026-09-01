#!/bin/bash
# 手动启动（与开机自启同一命令）
# 启动前先杀掉旧量产栈，避免多实例抢电机 / 无 /imu / GAIT 异常
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

_kill_old_stack() {
  echo "[ros2_start] 清理旧 ROS2 量产进程..."
  # 若 systemd 自启仍在跑，尽量停掉（无权限则依赖后面 pkill）
  if systemctl is-active --quiet ros2-bringup.service 2>/dev/null; then
    systemctl stop ros2-bringup.service 2>/dev/null || true
  fi

  # 先温和结束 launch 与核心节点
  local patterns=(
    'ros2 launch hightorque_bringup bfm_real'
    'ros2-bringup-boot.sh'
    'hightorque_controller_node'
    'hightorque_midware_node'
    'input_arbiter_walk'
    'bfm_motion_source'
    'recorded_bfm_joint_publisher'
    'bfm_startup_guard'
    'yesense_imu_node'
    'hightorque_oled_node'
    'hightorque_power'
    'power_node'
    'joy_mapper_node'
    'joy_linux_node'
    'wait_and_stand'
  )
  local p
  for p in "${patterns[@]}"; do
    pkill -f "$p" 2>/dev/null || true
  done
  sleep 1
  # 残留则强杀
  for p in "${patterns[@]}"; do
    pkill -KILL -f "$p" 2>/dev/null || true
  done
  sleep 1

  if pgrep -af 'bfm_real|hightorque_controller_node|hightorque_midware_node|yesense_imu_node' >/dev/null 2>&1; then
    echo "[ros2_start][warn] 仍有残留（可能属 root/其它用户）:"
    pgrep -af 'bfm_real|hightorque_controller_node|hightorque_midware_node|yesense_imu_node' || true
    echo "[ros2_start][warn] 可执行: sudo ${SCRIPT_DIR}/stop-ros2-autostart.sh"
  else
    echo "[ros2_start] 旧进程已清空"
  fi
}

_kill_old_stack

# 修复 IMU 串口映射（不改 hightorque 源码）；失败不阻断启动
if [ -x "${SCRIPT_DIR}/ensure_imu_serial.sh" ]; then
  "${SCRIPT_DIR}/ensure_imu_serial.sh" || true
fi

cd /home/nvidia/hightorque_workspace

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/nvidia/cyclonedds.xml}"
unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "[ros2_start] 启动 bfm_real.launch.py"
exec ros2 launch hightorque_bringup bfm_real.launch.py \
  auto_stand:=true \
  auto_start_bfm:=false \
  enable_gamepad_commands:=true \
  enable_fall_detector:=false \
  enable_auto_fall_recovery:=false
