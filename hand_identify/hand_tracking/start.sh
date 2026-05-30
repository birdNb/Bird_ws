#!/usr/bin/env bash
# 一键启动：五指(5)前后距离保持（目标约 50cm，发布 /cmd_vel.linear.x）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=/dev/null
source "${ROOT}/common/ros_env.sh"

cd "${SCRIPT_DIR}"
echo "[hand_tracking] 目录: ${SCRIPT_DIR}"
echo "[hand_tracking] 手势5距离保持（默认 --no-fsm，可加 --no-gui）"
exec python3 "${SCRIPT_DIR}/distance_hold.py" --no-fsm "$@"
