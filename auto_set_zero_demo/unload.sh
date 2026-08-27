#!/bin/bash
# 卸力：关节不发力（kp=0, kd=0, τ=0），不进阻尼
# 控制器在则 FSM init；否则经 midware 直接零力矩（--takeover 后走这条）
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/unload.py" "$@"
