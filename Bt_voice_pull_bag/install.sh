#!/bin/bash
# Bt_voice_pull_bag — 蓝牙 + 语音提醒 + 拖拽 + 头追 四合一安装（兼容 OTA / 手工）
# - RK / Orin 通用
# - OTA 以普通用户调用时，按平台自动 sudo 提权（RK: ht / Orin: nvidia）
# - 先同步到固定目录，再安装（OTA 删临时包后服务仍可用）
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

# ---------- 平台检测与提权（OTA 客户端非 root 调用）----------
_detect_board_kind() {
  local model=""
  if [ -r /proc/device-tree/model ]; then
    model="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)"
  fi
  model="$(printf '%s' "${model}" | tr '[:upper:]' '[:lower:]')"
  case "${model}" in
    *orin*|*jetson*)
      echo orin
      return
      ;;
    *rk3588*|*rk35*|*lubancat*|*rockchip*)
      echo rk
      return
      ;;
  esac
  if [ -f /etc/nv_tegra_release ] || lsmod 2>/dev/null | grep -qi tegra; then
    echo orin
    return
  fi
  echo rk
}

_elevate_if_needed() {
  if [ "$(id -u)" -eq 0 ]; then
    return 0
  fi
  if sudo -n true 2>/dev/null; then
    echo "[info] 使用免密 sudo 提权..."
    exec sudo -E bash "$0" "$@"
  fi
  local kind pass
  kind="$(_detect_board_kind)"
  if [ "${kind}" = "orin" ]; then
    pass="nvidia"
  else
    pass="ht"
  fi
  echo "[info] 检测到平台=${kind}，使用默认密码提权安装（OTA 兼容）..."
  # shellcheck disable=SC2024
  printf '%s\n' "${pass}" | sudo -S -E bash "$0" "$@"
  exit $?
}

_elevate_if_needed "$@"

# ---------- root 以下 ----------
# 安装用户：优先 sudo 调用者
INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
if [ "${INSTALL_USER}" = "root" ]; then
  INSTALL_USER="hightorque"
fi
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"

# 固定安装根目录（OTA 解包目录会被删除，必须拷到这里）
INSTALL_ROOT="${BIRD_INSTALL_ROOT:-${INSTALL_HOME}/Bird_ws/Bt_voice_pull_bag}"
BOARD_KIND="$(_detect_board_kind)"

sync_to_install_root() {
  echo "[sync] ${SRC_DIR} → ${INSTALL_ROOT}"
  mkdir -p "${INSTALL_ROOT}"
  # 不跟随把自身装进自己时的死循环：若已在目标目录则跳过 rsync 源=目标
  if [ "${SRC_DIR}" = "${INSTALL_ROOT}" ]; then
    echo "[sync] 已在正式目录，跳过拷贝"
    return 0
  fi
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete \
      --exclude '.git' \
      --exclude '__pycache__' \
      "${SRC_DIR}/" "${INSTALL_ROOT}/"
  else
    find "${INSTALL_ROOT}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
    cp -a "${SRC_DIR}/." "${INSTALL_ROOT}/"
  fi
  # 校验关键运行文件（防止同步丢 .pyc）
  if [ ! -f "${INSTALL_ROOT}/ble/app/ble_gatt_server.pyc" ] && [ ! -f "${INSTALL_ROOT}/ble/app/ble_gatt_server.py" ]; then
    echo "[error] 同步后缺少 ble_gatt_server，安装中止"
    exit 1
  fi
  if [ ! -f "${INSTALL_ROOT}/pull_move/app/torque_cmd_vel_bridge.pyc" ] && [ ! -f "${INSTALL_ROOT}/pull_move/app/torque_cmd_vel_bridge.py" ]; then
    echo "[error] 同步后缺少 torque_cmd_vel_bridge，安装中止"
    exit 1
  fi
  if [ ! -x "${INSTALL_ROOT}/locate_face_cpp/build/locate_face" ]; then
    echo "[error] 同步后缺少 locate_face_cpp/build/locate_face，安装中止"
    exit 1
  fi
  if [ ! -f "${INSTALL_ROOT}/locate_face_cpp/model/face_detection_yunet_2023mar.onnx" ]; then
    echo "[error] 同步后缺少 YuNet 模型，安装中止"
    exit 1
  fi
  chown -R "${INSTALL_USER}:${INSTALL_GID}" "${INSTALL_ROOT}"
  chmod +x "${INSTALL_ROOT}/install.sh" "${INSTALL_ROOT}/uninstall.sh" 2>/dev/null || true
  chmod +x "${INSTALL_ROOT}/locate_face_cpp/build/locate_face" \
            "${INSTALL_ROOT}/locate_face_cpp/start.sh" 2>/dev/null || true
  echo "[ok] 已同步到正式目录"
}

write_ota_version() {
  local ver_file="${SIM2REAL_WS}/version.json"
  local vernum date_tag raw
  vernum="$(tr -d '[:space:]' < "${INSTALL_ROOT}/VERSION" 2>/dev/null || echo 0728)"
  date_tag="$(date +%Y%m%d)"
  # OTA 匹配规则：名称-平台 前两段；RK/Orin 共用 ble-all
  raw="ble-all-${vernum}-${date_tag}"
  mkdir -p "$(dirname "${ver_file}")"
  python3 - "${ver_file}" "${raw}" <<'PY'
import json, sys
from pathlib import Path

path = Path(sys.argv[1])
raw = sys.argv[2]
items = []
if path.exists():
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, list):
            items = data
    except Exception:
        items = []
out = []
for x in items:
    if isinstance(x, str) and x.startswith("ble-all-"):
        continue
    out.append(x)
out.append(raw)
path.write_text(json.dumps(out, ensure_ascii=False) + "\n", encoding="utf-8")
print(f"[ok] OTA 本地版本: {path} -> {raw}")
PY
  chown "${INSTALL_USER}:${INSTALL_GID}" "${ver_file}" 2>/dev/null || true
}

cleanup_residuals() {
  echo "[0/5] 清理旧服务与残留..."
  local units=(
    bird-ble.service
    torque-cmd-vel.service
    bird-ble-boot.service
    bt-control.service
    ble-gatt.service
  )
  local u
  for u in "${units[@]}"; do
    systemctl stop "${u}" 2>/dev/null || true
    systemctl disable "${u}" 2>/dev/null || true
    rm -f "/etc/systemd/system/${u}"
    rm -f "/lib/systemd/system/${u}"
    rm -f "/usr/lib/systemd/system/${u}"
  done

  pkill -f 'ble_gatt_server\.pyc?' 2>/dev/null || true
  pkill -f 'torque_cmd_vel_bridge\.pyc?' 2>/dev/null || true
  pkill -f 'locate_face_cpp/build/locate_face' 2>/dev/null || true
  pkill -f '/locate_face_cpp/.*locate_face' 2>/dev/null || true

  rm -f /etc/default/bird-ble /etc/default/torque-cmd-vel

  systemctl daemon-reload 2>/dev/null || true
  systemctl reset-failed 2>/dev/null || true
  echo "[ok] 残留服务已清理"
}

sync_to_install_root

# 后续全部基于正式目录
PKG_DIR="${INSTALL_ROOT}"
BLE_DIR="${PKG_DIR}/ble"
PULL_DIR="${PKG_DIR}/pull_move"
VOICE_DIR="${PKG_DIR}/voice_remind"
FACE_DIR="${PKG_DIR}/locate_face_cpp"
BIRD_WS="${PKG_DIR}"
VERNUM="$(tr -d '[:space:]' < "${PKG_DIR}/VERSION" 2>/dev/null || echo 0728)"

echo "=========================================="
echo " Bt_voice_pull_bag 四合一安装（OTA 兼容）"
echo " 版本:     ${VERNUM}"
echo " 平台:     ${BOARD_KIND} (RK/Orin 通用包)"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " BIRD_WS:  ${BIRD_WS}"
echo " sim2real: ${SIM2REAL_WS}"
echo "=========================================="

for d in "${BLE_DIR}" "${PULL_DIR}" "${VOICE_DIR}" "${FACE_DIR}"; do
  if [ ! -d "${d}" ]; then
    echo "[error] 缺少子包: ${d}"
    exit 1
  fi
done
if [ ! -f "${VOICE_DIR}/__init__.py" ] && [ ! -f "${VOICE_DIR}/__init__.pyc" ]; then
  echo "[error] voice_remind 不完整: ${VOICE_DIR}"
  exit 1
fi
if [ ! -x "${FACE_DIR}/build/locate_face" ]; then
  echo "[error] locate_face_cpp 二进制缺失: ${FACE_DIR}/build/locate_face"
  exit 1
fi
if [ ! -f "${FACE_DIR}/model/face_detection_yunet_2023mar.onnx" ]; then
  echo "[error] locate_face_cpp 模型缺失"
  exit 1
fi

cleanup_residuals

echo "[1/6] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  pulseaudio-utils \
  rfkill \
  rsync

CONF="/etc/bluetooth/main.conf"
if [ -f "${CONF}" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "${CONF}" 2>/dev/null; then
  BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
  cp "${CONF}" "${BACKUP}"
  if grep -q '^\[General\]' "${CONF}"; then
    sed -i '/^\[General\]/a Experimental=true' "${CONF}"
  else
    printf '%s\n' '[General]' 'Experimental=true' >>"${CONF}"
  fi
  systemctl restart bluetooth
  sleep 2
  echo "[ok] BlueZ Experimental=true（备份: ${BACKUP}）"
fi

echo "[2/6] 安装蓝牙 BLE（含语音提醒 / 头追加载路径）..."
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
export BIRD_BLE_UID="${INSTALL_UID}"
export BIRD_WS="${BIRD_WS}"
export SIM2REAL_WS="${SIM2REAL_WS}"
export PKG_DIR="${BLE_DIR}"
export BT_DIR="${BLE_DIR}/app"

cat >/etc/default/bird-ble <<EOF
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
BIRD_BLE_UID=${INSTALL_UID}
BIRD_WS=${BIRD_WS}
SIM2REAL_WS=${SIM2REAL_WS}
BLE_DEVICE_NAME_FILE=/var/lib/bird-ble/ble_device_name.conf
EXTRA_ARGS=()
EOF

mkdir -p /var/lib/bird-ble
chmod 755 /var/lib/bird-ble
if [ ! -f /var/lib/bird-ble/ble_device_name.conf ] && [ -f "${BLE_DIR}/ble_device_name.conf" ]; then
  _bn="$(tr -d '[:space:]' < "${BLE_DIR}/ble_device_name.conf")"
  if [ -n "${_bn}" ]; then
    echo "${_bn}" >/var/lib/bird-ble/ble_device_name.conf
    chmod 644 /var/lib/bird-ble/ble_device_name.conf
    echo "[ok] 广播名持久化: ${_bn}"
  fi
fi
unset _bn

chown "${INSTALL_USER}:${INSTALL_GID}" "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${BLE_DIR}/app"/*.sh "${BLE_DIR}/app/scripts/"*.sh "${BLE_DIR}/app/systemd/"*.sh

# install-autostart 用环境变量 BIRD_WS / PKG_DIR(ble)
"${BLE_DIR}/app/install-autostart.sh"

echo "[3/6] 安装拖拽控制（默认不开机自启，由小程序 PULL ON 启动）..."
chmod +x "${PULL_DIR}/app"/*.sh "${PULL_DIR}/app/systemd/"*.sh
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
"${PULL_DIR}/app/install-autostart.sh"
systemctl disable torque-cmd-vel.service 2>/dev/null || true
systemctl stop torque-cmd-vel.service 2>/dev/null || true

echo "[4/6] 校验头追运行时（locate_face ON 启停，无独立 systemd）..."
chmod +x "${FACE_DIR}/build/locate_face" "${FACE_DIR}/start.sh" 2>/dev/null || true
chown -R "${INSTALL_USER}:${INSTALL_GID}" "${FACE_DIR}"
# 确保 BLE 侧路径可写日志
touch "${FACE_DIR}/locate_face_ble.log" 2>/dev/null || true
chown "${INSTALL_USER}:${INSTALL_GID}" "${FACE_DIR}/locate_face_ble.log" 2>/dev/null || true
echo "[ok] 头追: ${FACE_DIR}/build/locate_face"
echo "     小程序指令: locate_face ON / locate_face OFF"

echo "[5/6] 写入 OTA 本地版本（~/sim2real/version.json）..."
write_ota_version

echo "[6/6] 启动蓝牙服务并自检..."
systemctl daemon-reload
systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 5

set +u
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
[ -f "${SIM2REAL_WS}/install/setup.bash" ] && source "${SIM2REAL_WS}/install/setup.bash"
[ -f "${SIM2REAL_WS}/devel/setup.bash" ] && source "${SIM2REAL_WS}/devel/setup.bash"
set -u

python3 -c "import sim2real_msg" 2>/dev/null && echo "[ok] sim2real_msg" \
  || echo "[warn] 无法 import sim2real_msg — 检查 SIM2REAL_WS"
python3 -c "import sys; sys.path.insert(0, '${BIRD_WS}'); import voice_remind; print('[ok] voice_remind')" \
  || echo "[warn] voice_remind 导入失败"
if [ -x "${FACE_DIR}/build/locate_face" ]; then
  echo "[ok] locate_face_cpp"
else
  echo "[warn] locate_face_cpp 二进制不可执行"
fi

echo ""
echo "=========================================="
echo " 安装完成（四合一 ${VERNUM}）"
echo "=========================================="
echo " 正式目录: ${INSTALL_ROOT}"
echo " 蓝牙:     sudo systemctl status bird-ble"
echo " 拖拽:     sudo systemctl status torque-cmd-vel  # PULL ON 后运行"
echo " 头追:     locate_face ON/OFF  # ${FACE_DIR}"
echo " 语音:     ${VOICE_DIR}"
echo " OTA 组件: ble-all（RK/Orin 共用）"
echo " 日志:     journalctl -u bird-ble -f"
"${BLE_DIR}/app/scripts/check.sh" || true
