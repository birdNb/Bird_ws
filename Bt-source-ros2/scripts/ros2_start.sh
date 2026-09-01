#!/bin/bash
# 手动启动（与开机自启同一命令）
# 启动前先杀掉旧量产栈，避免多实例抢电机 / 无 /imu / GAIT 异常
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

_kill_old_stack() {
  if [ -x "${SCRIPT_DIR}/ros2-kill-old-stack.sh" ]; then
    "${SCRIPT_DIR}/ros2-kill-old-stack.sh" || true
    return
  fi
  echo "[ros2_start] 清理旧 ROS2 量产进程..."
  local patterns=(
    'ros2 launch hightorque_bringup bfm_real'
    'hightorque_controller_node'
    'hightorque_midware_node'
    'yesense_imu_node'
  )
  local p
  for p in "${patterns[@]}"; do
    pkill -f "$p" 2>/dev/null || true
  done
  sleep 1
}

# 手动启动时若 systemd 栈在跑，先停掉以免双开
if systemctl is-active --quiet ros2-bringup.service 2>/dev/null; then
  systemctl stop ros2-bringup.service 2>/dev/null || true
fi

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
