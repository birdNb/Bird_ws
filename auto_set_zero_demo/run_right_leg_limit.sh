#!/bin/bash
# 右腿限位：
#   1) 腰右转 90° → 到位后卸力
#   2) 右踝 pitch -30° 与 右膝 calf +30° 同时到位 → 保持
#   3) 右髋 roll 上抬 45° → 保持（5 N·m）
#   4) 踝 roll：抬腿前卸力；碰限位复位后固定
#   5) 从下往上寻硬限位：仅右腿轴固定，其余卸力
#   6) 全部结束后回到程序启动时读到的关节角
# 各轴保护力矩见 cw_limit.py（勿加 --tau-protect，否则会盖掉 hip 的 5 N·m）
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/cw_limit.py" --group right_leg "$@"
