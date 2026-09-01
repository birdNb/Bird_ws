#!/bin/bash
# 手动启动（与开机自启同一命令，不进桌面 autostart）
set -eo pipefail

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

exec ros2 launch hightorque_bringup bfm_real.launch.py \
  auto_stand:=true \
  auto_start_bfm:=false \
  enable_gamepad_commands:=true \
  enable_fall_detector:=false \
  enable_auto_fall_recovery:=false
