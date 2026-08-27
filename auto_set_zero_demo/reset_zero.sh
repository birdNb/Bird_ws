#!/bin/bash
# 官方调零 reset_zero：FSM init → prev → confirm
# 与手柄 / BLE M_resetzero 相同，会把当前全部电机角写成 0
# 必须先摆到出厂写零姿态再跑
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/reset_zero.py" "$@"
