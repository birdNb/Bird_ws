#!/bin/bash
# systemd 入口：肩/脖子力矩 → /cmd_vel（仅 ROS2，不再拉 roscore）
# FSM 守门默认开启：仅 EXEC_DEFAULT(5)=行走模式 才发布
set -eo pipefail

DEMO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO_DIR"

export ROS_DISTRO="${ROS_DISTRO:-foxy}"
export HOME="${HOME:-${BIRD_HOME:-/home/hightorque}}"
export BIRD_HOME="${BIRD_HOME:-$HOME}"
unset ROS_HOSTNAME ROS_IP ROS_MASTER_URI

_ros_strict_u=0
case $- in *u*) _ros_strict_u=1 ;; esac
set +u

if [ -f /opt/ros/foxy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
fi

COLCON_WS="${COLCON_WS:-${BIRD_HOME}/colcon_ws}"
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${COLCON_WS}/install/setup.bash"
fi

if [ "$_ros_strict_u" -eq 1 ]; then
  set -u
fi
unset _ros_strict_u

_extra="/opt/ros/foxy/lib/python3.8/site-packages"
_extra="${_extra}:/opt/ros/foxy/local/lib/python3.8/dist-packages"
export PYTHONPATH="${_extra}${PYTHONPATH:+:${PYTHONPATH}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

wait_ros2() {
  local i
  for i in $(seq 1 90); do
    if ros2 topic list >/dev/null 2>&1; then
      echo "[torque-cmd-vel] ROS2 已就绪"
      return 0
    fi
    sleep 2
  done
  echo "[torque-cmd-vel] 警告: ROS2 话题列表暂不可用，继续启动" >&2
  return 0
}

wait_topics() {
  local i
  for i in $(seq 1 90); do
    if ros2 topic list 2>/dev/null | grep -qx '/error_joint_states' \
      && ros2 topic list 2>/dev/null | grep -qx '/fsm_state'; then
      echo "[torque-cmd-vel] /error_joint_states 与 /fsm_state 已就绪"
      return 0
    fi
    sleep 2
  done
  echo "[torque-cmd-vel] 警告: 目标话题尚未出现，bridge 将自行等待" >&2
  return 0
}

wait_ros2
wait_topics

EXTRA_ARGS=()
if [ -f /etc/default/torque-cmd-vel ]; then
  # shellcheck disable=SC1091
  source /etc/default/torque-cmd-vel
fi

FILTERED_ARGS=()
for a in "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"; do
  if [ "$a" = "--no-fsm" ]; then
    echo "[torque-cmd-vel] 忽略 EXTRA_ARGS 中的 --no-fsm（开机必须走 FSM 守门）" >&2
    continue
  fi
  FILTERED_ARGS+=("$a")
done

echo "[torque-cmd-vel] 启动 torque_cmd_vel_bridge（仅 FSM=EXEC_DEFAULT 发速度）"
if [ -f "${DEMO_DIR}/torque_cmd_vel_bridge.py" ]; then
  exec python3 "${DEMO_DIR}/torque_cmd_vel_bridge.py" "${FILTERED_ARGS[@]}"
fi
exec python3 "${DEMO_DIR}/torque_cmd_vel_bridge.pyc" "${FILTERED_ARGS[@]}"
