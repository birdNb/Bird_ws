#!/bin/bash
# ROS2 量产栈开机启动（严格按用户指定命令）
set -eo pipefail

WS="/home/nvidia/hightorque_workspace"
LOG_DIR="${WS}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ros2-bringup.service.log"

export HOME="/home/nvidia"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/nvidia/cyclonedds.xml}"
export RCUTILS_COLORIZED_OUTPUT=0
unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP

# CycloneDDS 绑定 wlan0，网卡未 UP 时节点会全部崩溃
for _ in $(seq 1 60); do
  if ip link show wlan0 2>/dev/null | grep -q "UP"; then
    echo "[ros2-bringup] wlan0 UP" >>"${LOG_FILE}"
    break
  fi
  sleep 0.5
done

cd "${WS}"

# Yesense 常落在 ttyUSB1，量产配置写死 ttyUSB0 → 无 /imu → GAIT ON 被拒
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -x "${SCRIPT_DIR}/ensure_imu_serial.sh" ]; then
  "${SCRIPT_DIR}/ensure_imu_serial.sh" >>"${LOG_FILE}" 2>&1 || true
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "[ros2-bringup] $(date -Iseconds) start bfm_real" >>"${LOG_FILE}"

exec ros2 launch hightorque_bringup bfm_real.launch.py \
  auto_stand:=true \
  auto_start_bfm:=false \
  enable_gamepad_commands:=true \
  enable_fall_detector:=false \
  enable_auto_fall_recovery:=false
