#!/bin/bash
# 加载 ROS + sim2real 工作空间（供 start.sh / run_ble_with_ros.sh 共用）
# sim2real_msg 在 install 目录，仅 source devel 会缺包导致 FSM 模式无效

# systemd 最小环境下 ROS setup 脚本会引用 ROS_DISTRO，须预先设置
export ROS_DISTRO="${ROS_DISTRO:-noetic}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SIM2REAL_WS="/home/nvidia/sim2real"
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/install/setup.bash"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/devel/setup.bash"
fi

# sudo 下 PYTHONPATH 可能丢失，显式兜底
_extra="/opt/ros/noetic/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/install/lib/python3/dist-packages"
_extra="${_extra}:${SIM2REAL_WS}/devel/lib/python3/dist-packages"
export PYTHONPATH="${_extra}${PYTHONPATH:+:${PYTHONPATH}}"
