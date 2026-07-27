#!/bin/bash
# 打包 locate_face_cpp 运行时（二进制 + 模型 + worker），不带源码/CMake 中间文件
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$(cd "${SRC_DIR}/.." && pwd)/locate_face_cpp_bag"
BIN="${SRC_DIR}/build/locate_face"
MODEL="${SRC_DIR}/model/face_detection_yunet_2023mar.onnx"

if [ ! -x "${BIN}" ]; then
  echo "[build] 未找到 ${BIN}，正在编译..."
  bash "${SRC_DIR}/build.sh"
fi

if [ ! -f "${MODEL}" ]; then
  echo "[error] 缺少模型: ${MODEL}"
  exit 1
fi

echo "[build] 组装 locate_face_cpp 运行时 → ${OUT_DIR}"
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}/build" "${OUT_DIR}/model" "${OUT_DIR}/scripts"

cp -a "${BIN}" "${OUT_DIR}/build/locate_face"
chmod +x "${OUT_DIR}/build/locate_face"
cp -a "${MODEL}" "${OUT_DIR}/model/"
cp -a "${SRC_DIR}/scripts/face_yunet_worker.py" "${OUT_DIR}/scripts/"
if [ -f "${SRC_DIR}/scripts/face_mediapipe_worker.py" ]; then
  cp -a "${SRC_DIR}/scripts/face_mediapipe_worker.py" "${OUT_DIR}/scripts/"
fi
cp -a "${SRC_DIR}/start.sh" "${OUT_DIR}/start.sh"
chmod +x "${OUT_DIR}/start.sh"

# 轻量说明，便于现场排查
cat >"${OUT_DIR}/README.md" <<'EOF'
# locate_face_cpp（运行时）

由 BLE `locate_face ON/OFF` 启停。需 ROS + OpenCV，相机默认 `/dev/video4`。

```bash
./start.sh          # 后台头追（无 GUI）
./start.sh --gui
```
EOF

echo "[ok] ${OUT_DIR}"
ls -la "${OUT_DIR}/build/locate_face" "${OUT_DIR}/model/" "${OUT_DIR}/scripts/"
