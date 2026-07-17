#!/bin/bash
# Bird BLE 源码树入口 → app/start.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PKG_DIR="${PKG_DIR:-${ROOT}}"
export BT_DIR="${BT_DIR:-${ROOT}/app}"
cd "${BT_DIR}"
exec ./start.sh "$@"
