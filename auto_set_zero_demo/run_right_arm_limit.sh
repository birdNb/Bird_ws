#!/bin/bash
# 右手 4 轴：2 rad/s；肘 1 / 上臂 2 / 肩 roll 2 / 肩 pitch 2 N·m
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

# Foxy setup.bash 会读未定义的 AMENT_TRACE_SETUP_FILES，source 期间必须关掉 nounset
set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/cw_limit.py" --group right_arm "$@"
