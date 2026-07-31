#!/bin/bash
# systemd 开机入口：肩/脖子力矩 → /cmd_vel
# FSM 守门默认开启：仅 EXEC_DEFAULT(5)=行走模式 才发布 /cmd_vel
set -eo pipefail

DEMO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEMO_DIR"

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export HOME="${HOME:-${BIRD_HOME:-/home/hightorque}}"
export BIRD_HOME="${BIRD_HOME:-$HOME}"
unset ROS_HOSTNAME ROS_IP

# systemd + set -u 时 ROS setup 可能引用未定义变量
_ros_strict_u=0
case $- in *u*) _ros_strict_u=1 ;; esac
set +u

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SIM2REAL_WS="${SIM2REAL_WS:-${BIRD_HOME}/sim2real}"
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/install/setup.bash"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/devel/setup.bash"
fi

if [ "$_ros_strict_u" -eq 1 ]; then
  set -u
fi
unset _ros_strict_u

_extra="/opt/ros/noetic/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/install/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/devel/lib/python3/dist-packages"
export PYTHONPATH="${_extra}${PYTHONPATH:+:${PYTHONPATH}}"

wait_roscore() {
  local i
  for i in $(seq 1 90); do
    if rostopic list >/dev/null 2>&1; then
      echo "[torque-cmd-vel] roscore 已就绪"
      return 0
    fi
    sleep 2
  done
  echo "[torque-cmd-vel] 警告: roscore 长时间未就绪，继续启动并等待话题" >&2
  return 0
}

wait_topics() {
  local i
  for i in $(seq 1 90); do
    if rostopic list 2>/dev/null | grep -qx '/error_joint_states' \
      && rostopic list 2>/dev/null | grep -qx '/fsm_state'; then
      echo "[torque-cmd-vel] /error_joint_states 与 /fsm_state 已就绪"
      return 0
    fi
    sleep 2
  done
  echo "[torque-cmd-vel] 警告: 目标话题尚未出现，bridge 将自行等待" >&2
  return 0
}

wait_roscore
wait_topics

EXTRA_ARGS=()
if [ -f /etc/default/torque-cmd-vel ]; then
  # shellcheck disable=SC1091
  source /etc/default/torque-cmd-vel
fi

# 强制保留 FSM 守门：禁止在开机服务里关掉行走模式判断
FILTERED_ARGS=()
for a in "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"; do
  if [ "$a" = "--no-fsm" ]; then
    echo "[torque-cmd-vel] 忽略 EXTRA_ARGS 中的 --no-fsm（开机必须走 FSM 守门）" >&2
    continue
  fi
  FILTERED_ARGS+=("$a")
done

echo "[torque-cmd-vel] 启动 torque_cmd_vel_bridge（仅 FSM=EXEC_DEFAULT/行走模式发 /cmd_vel）"
exec python3 "${DEMO_DIR}/torque_cmd_vel_bridge.pyc" "${FILTERED_ARGS[@]}"
