#!/bin/bash
# 在已 source ROS 的环境下启动 BLE 服务（供 sudo 调用）
set -eo pipefail
cd "$(dirname "$0")"

# shellcheck disable=SC1091
source "$(pwd)/platform_env.sh"

BLE_ARGS=("$@")
set --
# shellcheck disable=SC1091
source "$(pwd)/ros_env.sh"
set -- "${BLE_ARGS[@]}"

if ! python3 -c "import rospy" 2>/dev/null; then
  echo "[error] 无法 import rospy"
  exit 1
fi

if ! python3 -c "import sim2real_msg" 2>/dev/null; then
  echo "[warn] 无法 import sim2real_msg — FSM 模式(M_*) 需要此包"
else
  echo "[ros] 环境 OK: rospy + sim2real_msg"
fi

# 源码树优先 .py；发布包仅有 .pyc
if [ -f "$(pwd)/ble_gatt_boot.py" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.py" "$@"
fi
if [ -f "$(pwd)/ble_gatt_boot.pyc" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.pyc" "$@"
fi
if [ -f "$(pwd)/ble_gatt_server.py" ]; then
  exec python3 "$(pwd)/ble_gatt_server.py" "$@"
fi
exec python3 "$(pwd)/ble_gatt_server.pyc" "$@"
