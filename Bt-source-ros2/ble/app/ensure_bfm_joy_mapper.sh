#!/usr/bin/env bash
# BFM：用 Bird_ws 内 Python joy_mapper 替代运行中的量产节点（回中不发零速）。
# 不修改 hightorque_workspace / action_library 任何源码或 install 文件。
set -euo pipefail

BT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAPPER_PY="${BT_DIR}/joy_mapper_bfm_fix.py"
LOG="${JOY_MAPPER_BFM_LOG:-/tmp/joy_mapper_bfm_fix.log}"

# shellcheck disable=SC1091
source "${BT_DIR}/platform_env.sh"
# shellcheck disable=SC1091
source "${BT_DIR}/ros_env.sh"

if [ ! -f "${MAPPER_PY}" ]; then
  echo "[ensure_bfm_joy_mapper][error] 缺少 ${MAPPER_PY}" >&2
  exit 1
fi
chmod +x "${MAPPER_PY}" 2>/dev/null || true

# 只停进程，不动 install 二进制
pkill -f '/hightorque_midware/joy_mapper_node' 2>/dev/null || true
pkill -f 'joy_mapper_bfm_fix\.py' 2>/dev/null || true
sleep 0.4

nohup python3 "${MAPPER_PY}" >>"${LOG}" 2>&1 &
echo "[ensure_bfm_joy_mapper] 已启动 Bird_ws joy_mapper pid=$! log=${LOG}"
echo "[ensure_bfm_joy_mapper] 完成（未改动 hightorque_workspace）"
