#!/bin/bash
# 上层 default_bt 站立。控制器被 --takeover 停掉时会自动拉起。
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/stand.py" "$@"
