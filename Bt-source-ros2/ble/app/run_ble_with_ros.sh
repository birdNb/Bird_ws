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

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "[error] 无法 import rclpy（需要 ROS2 Foxy）"
  exit 1
fi
if ! python3 -c "from sensor_msgs.msg import Joy" 2>/dev/null; then
  echo "[warn] 无法 import sensor_msgs.msg.Joy"
else
  echo "[ros] 环境 OK: rclpy + sensor_msgs（ROS2 /joy）"
fi

# 源码树优先 .py；兼容仅有 .pyc 的编译版
if [ -f "$(pwd)/ble_gatt_boot.py" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.py" "$@"
fi
if [ -f "$(pwd)/ble_gatt_boot.pyc" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.pyc" "$@"
fi
if [ -f "$(pwd)/ble_gatt_server.py" ]; then
  exec python3 "$(pwd)/ble_gatt_server.py" "$@"
fi
if [ -f "$(pwd)/ble_gatt_server.pyc" ]; then
  exec python3 "$(pwd)/ble_gatt_server.pyc" "$@"
fi
echo "[error] 未找到 ble_gatt_boot / ble_gatt_server 入口" >&2
exit 1
