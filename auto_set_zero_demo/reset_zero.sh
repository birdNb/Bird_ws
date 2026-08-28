#!/bin/bash
# 写零：midware /reset_zero（pi_plus_orin 无控制器时自动跳过 FSM）
# 与手柄调零写同一处 Flash，但绕过本机 confirm 时控制器 segfault 的问题
# 必须先摆到目标写零姿态再跑
#
# 用法:
#   ./reset_zero.sh              # 全机写零（推荐）
#   ./reset_zero.sh --recover    # OLED 卡在 reset 候选时，仅 init 退出
#   ./reset_zero.sh --motor-ids "16 17 18 19"   # 只写指定电机
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

exec python3 "${ROOT}/reset_zero.py" "$@"
