#!/bin/bash
# 确保 ROS2 midware 已起来、/joint_states 有数据。
# 常见原因：开机默认跑了 ROS1 sim2real，占住 /dev/ttyACM*，ROS2 起不来。
#
# 用法:
#   ./ensure_midware.sh              # 检查；没有 joint_states 则停 ROS1 并拉起 ROS2 bringup
#   ./ensure_midware.sh --status     # 只检查，不改动
#   ./ensure_midware.sh --restart    # 强制重启 ROS2 midware
#   ./ensure_midware.sh --keep-ros1  # 不自动停 ROS1（若串口冲突会失败）
#
# 成功后再跑: ./run_all_limit.sh
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="${ROOT}/.logs"
mkdir -p "$LOG_DIR"
BRINGUP_LOG="${LOG_DIR}/pi_plus_orin.bringup.log"
PID_FILE="${LOG_DIR}/pi_plus_orin.pid"

# 避免写 ~/.ros/log 失败（只读/权限）导致 rclpy 起不来
export ROS_HOME="$LOG_DIR/ros_home"
mkdir -p "$ROS_HOME/log"

STATUS_ONLY=0
RESTART=0
KEEP_ROS1=0
WAIT_SEC=30

while [[ $# -gt 0 ]]; do
  case "$1" in
    --status) STATUS_ONLY=1; shift ;;
    --restart) RESTART=1; shift ;;
    --keep-ros1) KEEP_ROS1=1; shift ;;
    --wait)
      WAIT_SEC="${2:-30}"
      shift 2
      ;;
    -h|--help)
      sed -n '2,14p' "$0"
      exit 0
      ;;
    *)
      echo "未知参数: $1（可用 --help）" >&2
      exit 2
      ;;
  esac
done

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml
export RCUTILS_COLORIZED_OUTPUT=0

has_joint_states() {
  # 优先用 topic echo（不依赖写用户目录）；失败再试 python
  if timeout 2.5s ros2 topic echo /joint_states --once 2>/dev/null | grep -q 'position:'; then
    return 0
  fi
  python3 - <<'PY' 2>/dev/null
import sys, time
import rclpy
from sensor_msgs.msg import JointState

rclpy.init(args=None)
node = rclpy.create_node("ensure_midware_js_check")
got = {"ok": False}

def cb(msg):
    if msg.name and len(msg.position) > 0:
        got["ok"] = True

node.create_subscription(JointState, "/joint_states", cb, 10)
t0 = time.time()
while time.time() - t0 < 2.0 and not got["ok"]:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got["ok"] else 1)
PY
}

ros1_running() {
  pgrep -af 'roslaunch|rosmaster|sim2real' 2>/dev/null | grep -v grep >/dev/null
}

ros2_bringup_running() {
  pgrep -af 'ros2 launch hightorque_bringup pi_plus_orin|pi_plus_orin.launch' 2>/dev/null \
    | grep -v grep >/dev/null
}

stop_ros1() {
  echo "[ensure_midware] 检测到 ROS1 (sim2real / roslaunch)，会占用串口，正在停止…"
  # 先温和停 bringup，再清残留
  pkill -INT -f 'roslaunch sim2real' 2>/dev/null || true
  pkill -INT -f 'roslaunch.*pi_plus_orin' 2>/dev/null || true
  sleep 1
  pkill -TERM -f 'rosmaster|roslaunch|sim2real|livelybot_|humanoid_driver|yesense_imu' 2>/dev/null || true
  sleep 1
  pkill -KILL -f 'rosmaster|roslaunch|sim2real' 2>/dev/null || true
  # 等串口释放
  local i
  for i in 1 2 3 4 5 6 7 8; do
    if ! ros1_running; then
      echo "[ensure_midware] ROS1 已停止"
      return 0
    fi
    sleep 0.5
  done
  echo "[warn] ROS1 进程可能仍在，继续尝试启动 ROS2" >&2
}

stop_ros2_bringup() {
  echo "[ensure_midware] 停止已有 ROS2 pi_plus_orin bringup…"
  if [[ -f "$PID_FILE" ]]; then
    local old
    old="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ -n "${old:-}" ]] && kill -0 "$old" 2>/dev/null; then
      kill -INT "$old" 2>/dev/null || true
      sleep 1
      kill -TERM "$old" 2>/dev/null || true
    fi
    rm -f "$PID_FILE"
  fi
  pkill -INT -f 'ros2 launch hightorque_bringup pi_plus_orin' 2>/dev/null || true
  pkill -INT -f 'pi_plus_orin.launch' 2>/dev/null || true
  sleep 1
  pkill -TERM -f 'hightorque_midware|motor_sdk|pi_plus_orin' 2>/dev/null || true
  sleep 0.5
}

start_ros2_bringup() {
  echo "[ensure_midware] 启动: ros2 launch hightorque_bringup pi_plus_orin.launch.py"
  echo "[ensure_midware] 日志: $BRINGUP_LOG"
  nohup ros2 launch hightorque_bringup pi_plus_orin.launch.py \
    >"$BRINGUP_LOG" 2>&1 &
  echo $! >"$PID_FILE"
  echo "[ensure_midware] bringup pid=$(cat "$PID_FILE")"
}

wait_joint_states() {
  local deadline=$((SECONDS + WAIT_SEC))
  echo "[ensure_midware] 等待 /joint_states （最长 ${WAIT_SEC}s）…"
  while (( SECONDS < deadline )); do
    if has_joint_states; then
      echo "[ensure_midware] OK: /joint_states 已有数据"
      return 0
    fi
    sleep 0.5
  done
  echo "[error] 超时仍无 /joint_states。请查看: $BRINGUP_LOG" >&2
  tail -n 40 "$BRINGUP_LOG" 2>/dev/null || true
  return 1
}

echo "=========================================="
echo "  ensure_midware  (ROS2 pi_plus_orin)"
echo "=========================================="

if has_joint_states; then
  echo "[ensure_midware] 已有 /joint_states"
  if [[ $STATUS_ONLY -eq 1 ]]; then
    exit 0
  fi
  if [[ $RESTART -eq 0 ]]; then
    echo "[ensure_midware] 无需操作。可直接: ./run_all_limit.sh"
    exit 0
  fi
  echo "[ensure_midware] --restart：将重启 midware"
fi

if [[ $STATUS_ONLY -eq 1 ]]; then
  echo "[ensure_midware] 状态: 无 /joint_states"
  if ros1_running; then
    echo "[ensure_midware] 原因提示: ROS1 正在运行（常占 /dev/ttyACM*）"
  fi
  if ! ros2_bringup_running; then
    echo "[ensure_midware] 原因提示: ROS2 pi_plus_orin bringup 未运行"
  fi
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "[ensure_midware] 当前无串口设备"
  exit 1
fi

if ros1_running; then
  if [[ $KEEP_ROS1 -eq 1 ]]; then
    echo "[error] ROS1 仍在跑且指定了 --keep-ros1；无法与 ROS2 共用串口" >&2
    exit 1
  fi
  stop_ros1
fi

if [[ $RESTART -eq 1 ]] || ros2_bringup_running; then
  if [[ $RESTART -eq 1 ]]; then
    stop_ros2_bringup
    start_ros2_bringup
  elif ! has_joint_states; then
    # bringup 进程在但无 joint_states：重启更稳
    echo "[ensure_midware] bringup 似在跑但无 joint_states，重启一次"
    stop_ros2_bringup
    start_ros2_bringup
  fi
else
  start_ros2_bringup
fi

wait_joint_states
echo "[ensure_midware] 完成。下一步: cd ${ROOT} && ./run_all_limit.sh"
