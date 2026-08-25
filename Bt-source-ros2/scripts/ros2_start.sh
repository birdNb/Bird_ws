#!/bin/bash
# 开机只跑 ROS2（量产 AMP）。若发现 ROS1 进程则停掉，不再拉 sim2real / roscore。
set -euo pipefail

COLCON_WS="${COLCON_WS:-/home/hightorque/colcon_ws}"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
TITLE="ros2-amp"
LOG_DIR="${COLCON_WS}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ros2_start.log"

stop_ros1() {
  if pgrep -x rosmaster >/dev/null 2>&1 || pgrep -x roscore >/dev/null 2>&1; then
    echo "[ros2] 发现 ROS1 master，正在停止..." | tee -a "${LOG_FILE}"
  fi
  pkill -f '/opt/ros/noetic/.*/roslaunch' >/dev/null 2>&1 || true
  pkill -f 'roslaunch sim2real_master' >/dev/null 2>&1 || true
  pkill -f 'sim2real/sim2real.sh' >/dev/null 2>&1 || true
  killall -q roslaunch rosmaster roscore >/dev/null 2>&1 || true
}

pick_launch() {
  if [ -d "${COLCON_WS}/install/hightorque_controller" ]; then
    echo "ros2 launch hightorque_bringup robot_bringup.launch.py"
  elif [ -d "${COLCON_WS}/install/sim2real_master" ]; then
    echo "ros2 launch hightorque_bringup robot_bringup.launch.py"
  else
    echo "ros2 launch hightorque_bringup pi_plus_rknn.launch.py"
  fi
}

run_in_terminal() {
  local cmd="$1"
  local inner="set +u; source /opt/ros/${ROS_DISTRO}/setup.bash; source ${COLCON_WS}/install/setup.bash; set -u; echo '[ros2] ${cmd}'; ${cmd}; exec bash"
  if command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="${TITLE}" -x bash -lc "${inner}"
    return 0
  fi
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${TITLE}" -- bash -lc "${inner}"
    return 0
  fi
  return 1
}

sleep 1
stop_ros1

if [ ! -f "${COLCON_WS}/install/setup.bash" ]; then
  echo "[ros2] 未找到 ${COLCON_WS}/install/setup.bash" | tee -a "${LOG_FILE}"
  exit 1
fi

LAUNCH_CMD="$(pick_launch)"
echo "[ros2] $(date -Iseconds) ${LAUNCH_CMD}" | tee -a "${LOG_FILE}"

if run_in_terminal "${LAUNCH_CMD}"; then
  wait || true
  exit 0
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${COLCON_WS}/install/setup.bash"
set -u
echo "[ros2] ${LAUNCH_CMD}"
eval "${LAUNCH_CMD}"
