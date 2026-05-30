#!/usr/bin/env bash
# 工程根目录快捷入口 → hand_tracking/start.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${ROOT}/hand_tracking/start.sh" "$@"
