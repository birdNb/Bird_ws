#!/bin/bash
# 杀掉旧量产 ROS2 栈（供 systemd ExecStartPre / 手动复用）
# 不停止 bird-ble；不杀本脚本调用方以外的无关进程。
set -uo pipefail

patterns=(
  'ros2 launch hightorque_bringup bfm_real'
  'hightorque_controller_node'
  'hightorque_midware_node'
  'input_arbiter_walk'
  'bfm_motion_source'
  'recorded_bfm_joint_publisher'
  'bfm_startup_guard'
  'yesense_imu_node'
  'hightorque_oled_node'
  'hightorque_power'
  '/power_node'
  'joy_mapper_node'
  'joy_linux_node'
  'wait_and_stand'
)

echo "[ros2-kill-old] cleaning leftover bringup processes..."
for p in "${patterns[@]}"; do
  pkill -f "$p" 2>/dev/null || true
done
sleep 1
for p in "${patterns[@]}"; do
  pkill -KILL -f "$p" 2>/dev/null || true
done
sleep 1
echo "[ros2-kill-old] done"
exit 0
