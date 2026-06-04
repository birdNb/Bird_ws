#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项...]

功能断点调试（传给 vision_controller）:
  （无模式参数时默认 --gesture_action）
  --all              完整视觉控制
  --locate_face      仅测试脸部/脖子跟踪（同 locate_face）
  --loacate_face     同上（拼写兼容）
  --gesture          手势识别预览（不发机器人指令）
  --gesture_action   手势0~4 + 动作库(/joy_msg)，推荐用于动作控制
  --actions          与 --gesture 合用：启用动作下发
  --hand_follow      仅五指底盘跟随
  --coquette         仅测手势1撒娇扭腰
  --no-joy           跳过手柄5秒仲裁，便于单机调试
  --no-gui           不显示 OpenCV 窗口（默认已开启）
  --gui              显示 OpenCV 预览窗口
  --help, -h         显示帮助

示例:
  ./start.sh
  ./start.sh --locate_face --no-joy
  ./start.sh --gesture_action --no-joy
  ./start.sh --gesture --actions --no-joy
  ./start.sh --hand_follow --no-joy
  ./build/vision_controller --locate_face --no-joy   # 也可直接运行

EOF
}

for arg in "$@"; do
  case "$arg" in
    --help|-h)
      show_help
      exit 0
      ;;
  esac
done

# 结束冲突进程（勿用宽泛关键字 locate_face / hand_tracking，会误杀
# 含 --locate_face、--hand_follow 的本脚本与 vision_controller）
pkill -f 'hand_identify_cpp/build/vision_controller' 2>/dev/null || true
pkill -f 'zed_gesture_recognition\.py' 2>/dev/null || true
pkill -f 'locate_face\.py' 2>/dev/null || true
pkill -f 'distance_hold\.py' 2>/dev/null || true
pkill -f 'hand_perception\.py' 2>/dev/null || true
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

# 避免 ROS 中文日志显示为 ????（需 UTF-8 locale）
export LANG="${LANG:-C.UTF-8}"
export LC_ALL="${LC_ALL:-C.UTF-8}"
export LANGUAGE="${LANGUAGE:-en_US:en}"
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ] && [ -f "${HOME}/.Xauthority" ]; then
  export XAUTHORITY="${HOME}/.Xauthority"
fi
VC_ARGS=("$@")
HAS_MODE=0
HAS_GUI=0
for a in "${VC_ARGS[@]}"; do
  case "$a" in
    --all|--locate_face|--loacate_face|--gesture|--gesture_action|--gesture_actions|--hand_follow|--hand_tracking|--coquette)
      HAS_MODE=1
      ;;
    --gui) HAS_GUI=1 ;;
    --no-gui) HAS_GUI=1 ;;
  esac
done
if [ "$HAS_MODE" -eq 0 ]; then
  VC_ARGS=(--gesture_action "${VC_ARGS[@]}")
fi
if [ "$HAS_GUI" -eq 0 ]; then
  VC_ARGS+=(--no-gui)
fi

echo "Starting: DISPLAY=$DISPLAY ./build/vision_controller ${VC_ARGS[*]}"
./build/vision_controller "${VC_ARGS[@]}"

echo "Resetting robot..."
rostopic pub -1 /pi_plus_absolute sensor_msgs/JointState \
  "{name: ['head_yaw_joint','head_pitch_joint'], position: [0.0, 0.0]}" 2>/dev/null || true
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" 2>/dev/null || true
echo "Reset done"
