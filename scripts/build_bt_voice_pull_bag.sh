#!/bin/bash
# 组装四合一包：蓝牙 + 语音提醒 + 拖拽 + 头追
# 输出: Bt_voice_pull_bag_0728 / Bt_voice_pull_bag_0728.tar.gz
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
OUT_NAME="Bt_voice_pull_bag_0728"
OUT="${WS}/${OUT_NAME}"
VERSION="0728"

echo "[build] 重建 BLE / pull_move / locate_face_cpp 子包..."
bash "${WS}/BT_Control_0717/scripts/build_release.sh"
bash "${WS}/pull_move_demo/scripts/build_release.sh"
bash "${WS}/locate_face_cpp/scripts/build_release.sh"

echo "[build] 组装 ${OUT_NAME}（四合一）..."
rm -rf "${OUT}"
mkdir -p "${OUT}"
cp -a "${WS}/BT_Control_0717_bag" "${OUT}/ble"
cp -a "${WS}/pull_move_0717_bag" "${OUT}/pull_move"
cp -a "${WS}/locate_face_cpp_bag" "${OUT}/locate_face_cpp"
rsync -a --delete \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'generate_*.py' \
  --exclude 'import_voice_bag.py' \
  --exclude 'normalize_assets.py' \
  --exclude 'make_conv_code.py' \
  "${WS}/voice_remind/" "${OUT}/voice_remind/"

cp "${SCRIPTS}/Bt_voice_pull_bag_install.sh" "${OUT}/install.sh"
cp "${SCRIPTS}/Bt_voice_pull_bag_uninstall.sh" "${OUT}/uninstall.sh"
cp "${SCRIPTS}/Bt_voice_pull_bag_README.md" "${OUT}/README.md"
echo "${VERSION}" >"${OUT}/VERSION"
chmod +x "${OUT}/install.sh" "${OUT}/uninstall.sh"

# 校验关键运行文件
need=(
  "${OUT}/ble/app/ble_gatt_server.pyc"
  "${OUT}/pull_move/app/torque_cmd_vel_bridge.pyc"
  "${OUT}/locate_face_cpp/build/locate_face"
  "${OUT}/locate_face_cpp/model/face_detection_yunet_2023mar.onnx"
  "${OUT}/voice_remind/__init__.py"
)
for f in "${need[@]}"; do
  if [ ! -e "${f}" ]; then
    echo "[error] 组装缺文件: ${f}"
    exit 1
  fi
done
chmod +x "${OUT}/locate_face_cpp/build/locate_face" "${OUT}/locate_face_cpp/start.sh"

echo "[build] 清理临时子包 / 压缩包..."
rm -rf "${WS}/BT_Control_0717_bag" "${WS}/pull_move_0717_bag" "${WS}/locate_face_cpp_bag"
rm -f "${WS}/BT_Control_0717_bag.tar.gz" "${WS}/pull_move_0717_bag.tar.gz"

# 清理旧日期归档（保留正式装机目录 Bt_voice_pull_bag）；权限不足时跳过
rm -rf "${WS}/Bt_voice_pull_bag_0724" 2>/dev/null || \
  echo "[warn] 无法删除旧归档 Bt_voice_pull_bag_0724（可手动 sudo rm）"
rm -f "${WS}/Bt_voice_pull_bag_0724.tar.gz" 2>/dev/null || true

cd "${WS}"
rm -f "${OUT_NAME}.tar.gz"
tar czf "${OUT_NAME}.tar.gz" "${OUT_NAME}"

echo "[ok] ${OUT}"
echo "[ok] ${WS}/${OUT_NAME}.tar.gz"
du -sh "${OUT}" "${WS}/${OUT_NAME}.tar.gz"
echo "[ok] VERSION=$(cat "${OUT}/VERSION")  四合一: ble + voice_remind + pull_move + locate_face_cpp"
