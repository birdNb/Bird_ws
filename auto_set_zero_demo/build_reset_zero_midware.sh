#!/bin/bash
# 编译 midware：使 /reset_zero 支持按电机索引写零（motor_ids）
# 在可写根文件系统的终端执行（本机普通终端，不要在只读沙箱里）:
#   cd ~/Bird_ws/auto_set_zero_demo && ./build_reset_zero_midware.sh
set -eo pipefail
WS=/home/nvidia/hightorque_workspace
source /opt/ros/foxy/setup.bash
cd "$WS"
colcon build --packages-select hightorque_msgs hightorque_midware \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo
echo "编译完成。请重启 pi_plus_orin bringup 后再跑 ./run_all_limit.sh"
echo "  ./ensure_midware.sh --restart"
echo "  ./run_all_limit.sh"
