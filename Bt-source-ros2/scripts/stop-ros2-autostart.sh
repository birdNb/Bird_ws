#!/bin/bash
# 停用 ROS2 systemd 自启并杀掉当前量产栈，便于手动启动
# 用法: sudo ./scripts/stop-ros2-autostart.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0" >&2
  exit 1
fi

echo "========== 停止 systemd ros2-bringup =========="
systemctl stop ros2-bringup.service 2>/dev/null || true
systemctl disable ros2-bringup.service 2>/dev/null || true
systemctl reset-failed ros2-bringup.service 2>/dev/null || true

echo "========== 杀掉量产栈进程 =========="
pkill -f 'ros2 launch hightorque_bringup bfm_real' 2>/dev/null || true
pkill -f 'ros2-bringup-boot.sh' 2>/dev/null || true
pkill -f 'hightorque_controller_node' 2>/dev/null || true
pkill -f 'hightorque_midware_node' 2>/dev/null || true
pkill -f 'input_arbiter_walk' 2>/dev/null || true
pkill -f 'bfm_motion_source' 2>/dev/null || true
pkill -f 'bfm_startup_guard' 2>/dev/null || true
pkill -f 'yesense_imu_node' 2>/dev/null || true
pkill -f 'hightorque_oled_node' 2>/dev/null || true
pkill -f 'joy_mapper_node' 2>/dev/null || true
pkill -f 'joy_linux_node' 2>/dev/null || true
sleep 1

# 二次确认
pkill -9 -f 'ros2 launch hightorque_bringup bfm_real' 2>/dev/null || true
pkill -9 -f 'hightorque_controller_node' 2>/dev/null || true
pkill -9 -f 'hightorque_midware_node' 2>/dev/null || true
sleep 1

echo ""
echo "ros2-bringup enabled: $(systemctl is-enabled ros2-bringup.service 2>/dev/null || echo n/a)"
echo "ros2-bringup active:  $(systemctl is-active ros2-bringup.service 2>/dev/null || echo n/a)"
if pgrep -af 'bfm_real|hightorque_controller_node|hightorque_midware_node' >/dev/null 2>&1; then
  echo "[warn] 仍有残留进程:"
  pgrep -af 'bfm_real|hightorque_controller_node|hightorque_midware_node' || true
else
  echo "[ok] 量产栈进程已清空，可手动启动"
fi

echo ""
echo "手动启动示例:"
echo "  cd /home/nvidia/hightorque_workspace"
echo "  source /opt/ros/foxy/setup.bash && source install/setup.bash"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml"
echo "  ros2 launch hightorque_bringup bfm_real.launch.py \\"
echo "    auto_stand:=true auto_start_bfm:=false \\"
echo "    enable_gamepad_commands:=true \\"
echo "    enable_fall_detector:=false \\"
echo "    enable_auto_fall_recovery:=false"
echo ""
echo "或: /home/nvidia/hightorque_workspace/ros2_start.sh"
echo "恢复自启: sudo ~/Bird_ws/Bt-source-ros2/scripts/install-ros2-autostart.sh"
