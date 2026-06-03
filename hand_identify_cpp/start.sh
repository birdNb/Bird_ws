#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

# 结束冲突进程
pkill -f vision_controller 2>/dev/null || true
pkill -f zed_gesture_recognition 2>/dev/null || true
pkill -f locate_face 2>/dev/null || true
pkill -f hand_tracking 2>/dev/null || true
pkill -f distance_hold 2>/dev/null || true
sleep 1

if [ ! -x ./build/vision_controller ]; then
  echo "未找到可执行文件，正在编译..."
  ./build.sh
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${HOME}/sim2real/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/devel/setup.bash"
elif [ -f "${HOME}/sim2real/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/install/setup.bash"
fi

export DISPLAY="${DISPLAY:-:0}"
./build/vision_controller "$@"

echo "正在复位机器人..."
rostopic pub -1 /pi_plus_absolute sensor_msgs/JointState \
  "{name: ['head_yaw_joint','head_pitch_joint'], position: [0.0, 0.0]}" 2>/dev/null || true
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" 2>/dev/null || true
echo "已复位"
