#!/usr/bin/env bash
# 兼容旧入口 → run_torque_bridge.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${ROOT}/run_torque_bridge.sh" "$@"
