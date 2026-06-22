#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${HOME}/sim2real/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/devel/setup.bash"
elif [ -f "${HOME}/sim2real/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/install/setup.bash"
fi

MODEL_DIR="$(pwd)/model"
MODEL_FILE="${MODEL_DIR}/face_detection_yunet_2023mar.onnx"
MODEL_URL="https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"

mkdir -p "${MODEL_DIR}"
if [ ! -s "${MODEL_FILE}" ]; then
  echo "[model] 下载 YuNet: ${MODEL_URL}"
  if command -v curl >/dev/null; then
    curl -fsSL -o "${MODEL_FILE}" "${MODEL_URL}"
  elif command -v wget >/dev/null; then
    wget -q -O "${MODEL_FILE}" "${MODEL_URL}"
  else
    echo "[error] 需要 curl 或 wget 下载人脸模型"
    exit 1
  fi
fi

rm -rf build
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
echo "Build OK: $(pwd)/locate_face"
