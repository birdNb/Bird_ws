#!/bin/bash
# 腰 + 脖子：1 rad/s 顺时针；头 pitch/yaw 1 N·m，腰 1 N·m
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/cw_limit.py" --group waist_neck "$@"
