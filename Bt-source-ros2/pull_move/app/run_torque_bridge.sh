#!/usr/bin/env bash
# 肩/脖子力矩 → /cmd_vel 映射桥
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ -f "${HOME}/sim2real/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/install/setup.bash"
elif [[ -f "${HOME}/sim2real/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/devel/setup.bash"
fi

if [ -f "${ROOT}/torque_cmd_vel_bridge.py" ]; then
  exec python3 "${ROOT}/torque_cmd_vel_bridge.py" "$@"
fi
exec python3 "${ROOT}/torque_cmd_vel_bridge.pyc" "$@"
