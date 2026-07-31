#!/bin/bash
# 组装加密发布包 Bt_voice_pull_bag_0730（RK3588 + Jetson Orin / ZED Mini 通用）
# - Python → 字节码（去掉 .py）
# - 含 locate_face_cpp 运行时（ZED Mini 并排双目自适应 + RK D435i）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
# 脚本放在 Bt_voice_pull_bag/scripts/ 时 ROOT=Bird_ws；也支持直接放在包旁调用
if [ -d "${ROOT}/Bt_voice_pull_bag/ble" ]; then
  SRC_LIVE="${ROOT}/Bt_voice_pull_bag"
  WS="${ROOT}"
elif [ -d "$(cd "$(dirname "$0")/.." && pwd)/ble" ]; then
  SRC_LIVE="$(cd "$(dirname "$0")/.." && pwd)"
  WS="$(cd "${SRC_LIVE}/.." && pwd)"
else
  echo "[error] 找不到 Bt_voice_pull_bag 源"
  exit 1
fi

VERSION="0730"
OUT_NAME="Bt_voice_pull_bag_${VERSION}"
OUT_DIR="${WS}/${OUT_NAME}"
LF_SRC="${WS}/locate_face_cpp"
BT_SRC="${WS}/BT_test"

echo "[build] 工作区: ${WS}"
echo "[build] 装机源: ${SRC_LIVE}"
echo "[build] 输出:   ${OUT_DIR}"

# ---------- 1) 重建头追（Orin ZED Mini + RK 自适应）----------
if [ -x "${LF_SRC}/build.sh" ]; then
  echo "[build] 编译 locate_face_cpp..."
  (cd "${LF_SRC}" && bash ./build.sh)
fi
if [ ! -x "${LF_SRC}/build/locate_face" ]; then
  echo "[error] 缺少 ${LF_SRC}/build/locate_face"
  exit 1
fi

# ---------- 2) 同步最新 BLE 源改动到装机目录（再加密）----------
sync_py_to_live() {
  local name="$1"
  if [ -f "${BT_SRC}/${name}" ]; then
    cp -a "${BT_SRC}/${name}" "${SRC_LIVE}/ble/app/${name}"
  fi
}
for f in ble_status_telemetry.py ble_status_hooks.py ble_gatt_boot.py ble_locate_face_manager.py; do
  sync_py_to_live "$f"
done

# ---------- 3) 组装输出目录 ----------
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

# 先整体拷贝当前可用装机树（含 ble/pull_move/voice_remind/docs/scripts）
rsync -a --delete \
  --exclude '.git' \
  --exclude '__pycache__' \
  --exclude '*.log' \
  --exclude 'locate_face_cpp/locate_face_ble.log' \
  "${SRC_LIVE}/" "${OUT_DIR}/"

# 覆盖头追运行时（干净结构）
rm -rf "${OUT_DIR}/locate_face_cpp"
mkdir -p "${OUT_DIR}/locate_face_cpp/build" \
         "${OUT_DIR}/locate_face_cpp/model" \
         "${OUT_DIR}/locate_face_cpp/scripts"
cp -a "${LF_SRC}/build/locate_face" "${OUT_DIR}/locate_face_cpp/build/locate_face"
chmod +x "${OUT_DIR}/locate_face_cpp/build/locate_face"
cp -a "${LF_SRC}/model/face_detection_yunet_2023mar.onnx" "${OUT_DIR}/locate_face_cpp/model/"
cp -a "${LF_SRC}/scripts/face_yunet_worker.py" "${OUT_DIR}/locate_face_cpp/scripts/"
cp -a "${LF_SRC}/scripts/face_mediapipe_worker.py" "${OUT_DIR}/locate_face_cpp/scripts/"
cp -a "${LF_SRC}/start.sh" "${OUT_DIR}/locate_face_cpp/start.sh"
chmod +x "${OUT_DIR}/locate_face_cpp/start.sh"
# 发布包 start.sh 不应尝试本地编译
sed -i 's|./build.sh|echo "[error] 发布包无源码，请使用预编译 build/locate_face"|g' \
  "${OUT_DIR}/locate_face_cpp/start.sh" || true

cat >"${OUT_DIR}/locate_face_cpp/README.md" <<'EOF'
# locate_face_cpp（运行时）

BLE `locate_face ON/OFF` 启停。适配：

- **Jetson Orin + ZED Mini**：自动选 `/dev/video0`，并排双目取左眼
- **RK3588 + D435i**：优先 `/dev/video4` 彩色流
- 覆盖：`export LOCATE_FACE_CAMERA=0`

```bash
./start.sh
./start.sh --gui
```
EOF

# ---------- 4) 加密：全部 .py → .pyc，再删源码 ----------
echo "[build] 编译字节码并移除 .py ..."
cd "${OUT_DIR}"
python3 -m compileall -b -q .

# 删除发布包内业务 .py（保留无同名依赖的纯脚本若有）
while IFS= read -r -d '' py; do
  base="${py%.py}"
  # 必须有对应 .pyc 才删
  if [ -f "${base}.pyc" ]; then
    rm -f "${py}"
  fi
done < <(find "${OUT_DIR}" -type f -name '*.py' -print0)

# 清理 __pycache__
find "${OUT_DIR}" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true

# ---------- 5) 入口脚本改为 .pyc ----------
# run_ble_with_ros：优先 boot.pyc
cat >"${OUT_DIR}/ble/app/run_ble_with_ros.sh" <<'EOF'
#!/bin/bash
# 在已 source ROS 的环境下启动 BLE 服务（供 sudo 调用）
set -eo pipefail
cd "$(dirname "$0")"

# shellcheck disable=SC1091
source "$(pwd)/platform_env.sh"

BLE_ARGS=("$@")
set --
# shellcheck disable=SC1091
source "$(pwd)/ros_env.sh"
set -- "${BLE_ARGS[@]}"

if ! python3 -c "import rospy" 2>/dev/null; then
  echo "[error] 无法 import rospy"
  echo "  请确认: source /opt/ros/noetic/setup.bash"
  exit 1
fi

if ! python3 -c "import sim2real_msg" 2>/dev/null; then
  echo "[warn] 无法 import sim2real_msg — FSM 模式(M_*) 需要此包"
  echo "  请确认: source ~/sim2real/install/setup.bash"
else
  echo "[ros] 环境 OK: rospy + sim2real_msg"
fi

# 先挂载功能状态遥测补丁，再进 GATT
if [ -f "$(pwd)/ble_gatt_boot.pyc" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.pyc" "$@"
fi
if [ -f "$(pwd)/ble_gatt_boot.py" ]; then
  exec python3 "$(pwd)/ble_gatt_boot.py" "$@"
fi
exec python3 "$(pwd)/ble_gatt_server.pyc" "$@"
EOF
chmod +x "${OUT_DIR}/ble/app/run_ble_with_ros.sh"

# platform_hw 通过 platform_detect（pyc）识别 RK/Orin — 保持不变

# ---------- 6) VERSION / README ----------
echo "${VERSION}" >"${OUT_DIR}/VERSION"

cat >"${OUT_DIR}/README.md" <<EOF
# Bt_voice_pull_bag（OTA / 手工通用）

蓝牙遥控 + 语音提醒 + 拖拽控制 + 头追 **四合一**安装包。
**RK3588 与 Jetson Orin 通用**；头追适配 **ZED Mini（Orin）** 与 **D435i（RK）**。

版本：\`${VERSION}\`（发布运行时，Python 已编译为字节码）

## 一键安装

\`\`\`bash
cd Bt_voice_pull_bag_${VERSION}
sudo ./install.sh
# OTA：ota-client 直接调用 ./install.sh（会自动提权）
\`\`\`

安装时会：

1. 按平台自动 sudo（RK 密码 \`ht\`，Orin 密码 \`nvidia\`）
2. 同步到 \`~/Bird_ws/Bt_voice_pull_bag\`
3. 清理旧 bird-ble / torque-cmd-vel / 头追残留
4. 安装并启动蓝牙；拖拽默认关闭（\`PULL ON\`）
5. 头追随包内 \`locate_face_cpp\`，小程序 \`locate_face ON/OFF\`
6. 写入 OTA：\`~/sim2real/version.json\` → \`ble-all-${VERSION}-<日期>\`

## 目录结构

\`\`\`
Bt_voice_pull_bag_${VERSION}/
  install.sh / uninstall.sh / VERSION / README.md
  ble/                 # 蓝牙 GATT（字节码）
  voice_remind/        # 语音提示（字节码 + wav）
  pull_move/           # 拖拽（字节码）
  locate_face_cpp/     # 头追运行时（二进制 + 模型 + worker 字节码）
\`\`\`

## 头追相机

| 平台 | 默认相机 | 说明 |
|------|----------|------|
| Orin + ZED Mini | \`/dev/video0\` | 并排双目自动裁左眼 |
| RK + D435i | \`/dev/video4\` | 彩色流 |
| 手动覆盖 | \`LOCATE_FACE_CAMERA=N\` | 安装环境或 shell 导出 |

需 FSM=\`EXEC_DEFAULT(5)\` 后才下发脖子目标。

## 服务

| 功能 | 触发 | 默认 |
|------|------|------|
| 蓝牙 + 语音 | \`bird-ble.service\` | 开机自启 |
| 拖拽 | \`torque-cmd-vel.service\` | \`PULL ON\` |
| 头追 | BLE \`locate_face ON/OFF\` | 关 |

\`\`\`bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
tail -f ~/Bird_ws/Bt_voice_pull_bag/locate_face_cpp/locate_face_ble.log
\`\`\`
EOF

# install.sh 默认版本回退文案
sed -i "s/echo 0724/echo ${VERSION}/g; s/echo 0728/echo ${VERSION}/g" "${OUT_DIR}/install.sh" || true

# ---------- 7) 校验 ----------
need=(
  ble/app/ble_gatt_server.pyc
  ble/app/ble_gatt_boot.pyc
  ble/app/ble_status_telemetry.pyc
  ble/app/ble_status_hooks.pyc
  ble/app/ble_locate_face_manager.pyc
  ble/app/platform_detect.pyc
  pull_move/app/torque_cmd_vel_bridge.pyc
  voice_remind/__init__.pyc
  voice_remind/player.pyc
  locate_face_cpp/build/locate_face
  locate_face_cpp/model/face_detection_yunet_2023mar.onnx
  locate_face_cpp/scripts/face_yunet_worker.pyc
  install.sh
)
for rel in "${need[@]}"; do
  if [ ! -e "${OUT_DIR}/${rel}" ]; then
    echo "[error] 缺少: ${rel}"
    exit 1
  fi
done

# 不应残留关键业务源码
leak=$(find "${OUT_DIR}/ble/app" "${OUT_DIR}/voice_remind" "${OUT_DIR}/pull_move" \
  "${OUT_DIR}/locate_face_cpp/scripts" -name '*.py' 2>/dev/null | head -20 || true)
if [ -n "${leak}" ]; then
  echo "[warn] 仍有 .py 残留:"
  echo "${leak}"
fi

# 导入冒烟（装机关键模块）
cd "${OUT_DIR}/ble/app"
python3 - <<'PY'
import ble_status_telemetry
import ble_status_hooks
import ble_locate_face_manager
import ble_gatt_boot
assert ble_status_telemetry.FSM_REPEAT == 2
assert ble_status_telemetry.MP_BURST_COUNT == 2
print("[ok] import smoke")
PY

# ---------- 8) tar.gz ----------
cd "${WS}"
tar czf "${OUT_NAME}.tar.gz" "${OUT_NAME}"
echo "[ok] 目录: ${OUT_DIR}"
echo "[ok] 压缩: ${WS}/${OUT_NAME}.tar.gz"
du -sh "${OUT_DIR}" "${WS}/${OUT_NAME}.tar.gz"
echo "[ok] locate_face: $(file "${OUT_DIR}/locate_face_cpp/build/locate_face")"
ls "${OUT_DIR}"
