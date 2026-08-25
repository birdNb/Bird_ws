#!/bin/bash
# 加载 ROS2 Foxy + colcon 工作空间

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]:-$0}")/platform_env.sh"

export ROS_DISTRO="${ROS_DISTRO:-foxy}"
unset ROS_MASTER_URI
unset ROS_HOSTNAME
unset ROS_IP

_ros_strict_u=0
case $- in *u*) _ros_strict_u=1 ;; esac
set +u

if [ -f /opt/ros/foxy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
fi

COLCON_WS="${COLCON_WS:-${BIRD_HOME:-$HOME}/colcon_ws}"
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${COLCON_WS}/install/setup.bash"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -f "${BIRD_HOME:-$HOME}/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${BIRD_HOME:-$HOME}/cyclonedds.xml}"
fi

if [ "$_ros_strict_u" -eq 1 ]; then
  set -u
fi
unset _ros_strict_u

_extra="/opt/ros/foxy/lib/python3.8/site-packages"
_extra="${_extra}:/opt/ros/foxy/local/lib/python3.8/dist-packages"
if [ -n "${BIRD_WS:-}" ] && [ -d "${BIRD_WS}" ]; then
  _extra="${BIRD_WS}:${_extra}"
fi
export PYTHONPATH="${_extra}${PYTHONPATH:+:${PYTHONPATH}}"
export COLCON_WS
