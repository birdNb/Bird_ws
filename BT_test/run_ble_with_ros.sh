#!/bin/bash
# 在已 source ROS 的环境下启动 BLE 服务（供 sudo 调用）
set -euo pipefail
cd "$(dirname "$0")"

# source setup.bash 会把当前 $@ 传给 _setup_util.py，必须先清空
BLE_ARGS=("$@")
set --
# shellcheck disable=SC1091
source "$(pwd)/ros_env.sh"
set -- "${BLE_ARGS[@]}"

if ! python3 -c "import rospy" 2>/dev/null; then
  echo "[error] 无法 import rospy"
  echo "  请确认: source /opt/ros/noetic/setup.bash"
  exit 1
fi

if ! python3 -c "import sim2real_msg" 2>/dev/null; then
  echo "[warn] 无法 import sim2real_msg — FSM 模式(M_*) 需要此包"
  echo "  请确认: source ~/sim2real/install/setup.bash"
else
  echo "[ros] 环境 OK: rospy + sim2real_msg"
fi

exec python3 "$(pwd)/ble_gatt_server.py" "$@"
