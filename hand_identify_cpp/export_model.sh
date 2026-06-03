#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

pip install -r requirements-export.txt -q
python3 scripts/export_onnx.py
