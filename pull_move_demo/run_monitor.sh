#!/usr/bin/env bash
# 右手肩 pitch/yaw 力矩监控 demo
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

exec python3 "${ROOT}/monitor_r_shoulder_torque.py" "$@"
