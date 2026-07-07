#!/usr/bin/env bash
# D435i RGB 摄像头预览一键启动
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  echo "[D435i_demo] 警告: 不要用 sudo 运行，会装到 root 的 Python 且仍可能打不开相机"
  echo "[D435i_demo] 请改用: ./start.sh"
fi

if ! python3 -c "import cv2" 2>/dev/null; then
  echo "[D435i_demo] 安装 opencv / numpy ..."
  pip3 install --user opencv-python numpy
fi

if ! python3 -c "import pyrealsense2" 2>/dev/null; then
  echo "[D435i_demo] 未检测到 pyrealsense2（可选），尝试安装 ..."
  pip3 install --user -r requirements.txt || true
fi

exec python3 -u rgb_camera_demo.py "$@"
