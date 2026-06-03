#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

pip install -r requirements-export.txt -q
python3 scripts/export_onnx.py

FACE_MODEL="model/face_detection_yunet_2023mar.onnx"
if [ ! -f "${FACE_MODEL}" ]; then
  echo "Downloading YuNet face model..."
  curl -fsSL -o "${FACE_MODEL}" \
    "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
fi
echo "Face model OK: ${FACE_MODEL}"
