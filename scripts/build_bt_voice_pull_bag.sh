#!/bin/bash
# 从源码重新组装 Bt_voice_pull_bag_0721，并清理 Bird_ws 下旧独立 bag
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
OUT_NAME="Bt_voice_pull_bag_0721"
OUT="${WS}/${OUT_NAME}"

echo "[build] 重建 BLE / pull_move 子包..."
bash "${WS}/BT_Control_0717/scripts/build_release.sh"
bash "${WS}/pull_move_demo/scripts/build_release.sh"

echo "[build] 组装 ${OUT_NAME}..."
rm -rf "${OUT}"
mkdir -p "${OUT}"
cp -a "${WS}/BT_Control_0717_bag" "${OUT}/ble"
cp -a "${WS}/pull_move_0717_bag" "${OUT}/pull_move"
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
echo 0721 >"${OUT}/VERSION"
chmod +x "${OUT}/install.sh" "${OUT}/uninstall.sh"

echo "[build] 清理旧独立 bag / 压缩包..."
rm -rf "${WS}/BT_Control_0717_bag" "${WS}/pull_move_0717_bag"
rm -f "${WS}/BT_Control_0717_bag.tar.gz" "${WS}/pull_move_0717_bag.tar.gz"

cd "${WS}"
tar czf "${OUT_NAME}.tar.gz" "${OUT_NAME}"

echo "[ok] ${OUT}"
echo "[ok] ${WS}/${OUT_NAME}.tar.gz"
du -sh "${OUT}" "${WS}/${OUT_NAME}.tar.gz"
