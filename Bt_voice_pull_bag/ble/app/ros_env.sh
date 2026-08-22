#!/bin/bash
# 加载 ROS + sim2real 工作空间

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]:-$0}")/platform_env.sh"

# systemd/sudo + set -u 时 ROS setup 脚本会引用未定义变量，须预先设置
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
unset ROS_HOSTNAME

_ros_strict_u=0
case $- in *u*) _ros_strict_u=1 ;; esac
set +u

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SIM2REAL_WS="${SIM2REAL_WS:-${BIRD_HOME:-$HOME}/sim2real}"
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

# sudo 下 PYTHONPATH 可能丢失，显式兜底
_extra="/opt/ros/noetic/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/install/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/devel/lib/python3/dist-packages"
# 三合一包根：voice_remind（BIRD_WS 在装机后指向 Bt_voice_pull_bag）
if [ -n "${BIRD_WS:-}" ] && [ -d "${BIRD_WS}" ]; then
  _extra="${BIRD_WS}:${_extra}"
fi
export PYTHONPATH="${_extra}${PYTHONPATH:+:${PYTHONPATH}}"
