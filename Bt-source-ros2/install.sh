#!/bin/bash
# Bt-source-ros2 — 蓝牙 + 语音提醒（ROS2 Foxy /joy 桥）
# - RK / Orin 通用
# - 非 root 时按平台自动 sudo 提权（RK: ht / Orin: nvidia）
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

# ---------- 平台检测与提权 ----------
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
  echo "[info] 检测到平台=${kind}，使用默认密码提权安装..."
  # shellcheck disable=SC2024
  printf '%s\n' "${pass}" | sudo -S -E bash "$0" "$@"
  exit $?
}

_elevate_if_needed "$@"

# ---------- root 以下 ----------
INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
if [ "${INSTALL_USER}" = "root" ]; then
  INSTALL_USER="hightorque"
fi
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"
COLCON_WS="${COLCON_WS:-${INSTALL_HOME}/colcon_ws}"

INSTALL_ROOT="${BIRD_INSTALL_ROOT:-${INSTALL_HOME}/Bird_ws/Bt-source-ros2}"
BOARD_KIND="$(_detect_board_kind)"
PKG_VERSION="$(tr -d '[:space:]' < "${SRC_DIR}/VERSION" 2>/dev/null || echo Bt-source-ros2-1.0.0-260824)"

sync_to_install_root() {
  echo "[sync] ${SRC_DIR} → ${INSTALL_ROOT}"
  mkdir -p "${INSTALL_ROOT}"
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
  if [ ! -f "${INSTALL_ROOT}/ble/app/ble_gatt_server.py" ] && [ ! -f "${INSTALL_ROOT}/ble/app/ble_gatt_server.pyc" ]; then
    echo "[error] 同步后缺少 ble_gatt_server，安装中止"
    exit 1
  fi
  if [ ! -f "${INSTALL_ROOT}/pull_move/app/torque_cmd_vel_bridge.py" ] && [ ! -f "${INSTALL_ROOT}/pull_move/app/torque_cmd_vel_bridge.pyc" ]; then
    echo "[error] 同步后缺少 torque_cmd_vel_bridge，安装中止"
    exit 1
  fi
  if [ ! -x "${INSTALL_ROOT}/locate_face_cpp/build/locate_face" ]; then
    local ws_lf="${INSTALL_HOME}/Bird_ws/locate_face_cpp"
    if [ -x "${ws_lf}/build/locate_face" ]; then
      echo "[sync] 补齐 locate_face_cpp 运行时 ← ${ws_lf}"
      mkdir -p "${INSTALL_ROOT}/locate_face_cpp/build" \
               "${INSTALL_ROOT}/locate_face_cpp/model" \
               "${INSTALL_ROOT}/locate_face_cpp/scripts"
      cp -a "${ws_lf}/build/locate_face" "${INSTALL_ROOT}/locate_face_cpp/build/locate_face"
      chmod +x "${INSTALL_ROOT}/locate_face_cpp/build/locate_face"
      if [ -f "${ws_lf}/model/face_detection_yunet_2023mar.onnx" ]; then
        cp -a "${ws_lf}/model/face_detection_yunet_2023mar.onnx" \
          "${INSTALL_ROOT}/locate_face_cpp/model/"
      fi
      cp -a "${ws_lf}/scripts/face_yunet_worker.py" "${INSTALL_ROOT}/locate_face_cpp/scripts/" 2>/dev/null || true
      cp -a "${ws_lf}/scripts/face_mediapipe_worker.py" "${INSTALL_ROOT}/locate_face_cpp/scripts/" 2>/dev/null || true
      cp -a "${ws_lf}/start.sh" "${INSTALL_ROOT}/locate_face_cpp/start.sh" 2>/dev/null || true
      chmod +x "${INSTALL_ROOT}/locate_face_cpp/start.sh" 2>/dev/null || true
    else
      echo "[warn] 未找到 locate_face 二进制，BLE locate_face ON 将无法启动头追"
    fi
  fi
  chown -R "${INSTALL_USER}:${INSTALL_GID}" "${INSTALL_ROOT}"
  chmod +x "${INSTALL_ROOT}/install.sh" "${INSTALL_ROOT}/uninstall.sh" 2>/dev/null || true
  echo "[ok] 已同步到正式目录"
}

cleanup_residuals() {
  echo "[0/5] 清理旧服务与残留..."
  local units=(
    bird-ble.service
    ros2-bringup.service
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
  pkill -f 'ble_gatt_boot\.pyc?' 2>/dev/null || true
  pkill -f 'torque_cmd_vel_bridge\.pyc?' 2>/dev/null || true

  rm -f /etc/default/bird-ble /etc/default/torque-cmd-vel /etc/default/ros2-bringup

  systemctl daemon-reload 2>/dev/null || true
  systemctl reset-failed 2>/dev/null || true
  echo "[ok] 残留服务已清理"
}

sync_to_install_root

PKG_DIR="${INSTALL_ROOT}"
BLE_DIR="${PKG_DIR}/ble"
PULL_DIR="${PKG_DIR}/pull_move"
VOICE_DIR="${PKG_DIR}/voice_remind"
BIRD_WS="${PKG_DIR}"

echo "=========================================="
echo " Bt-source-ros2 一键安装（ROS2 Foxy）"
echo " 版本:     ${PKG_VERSION}"
echo " 平台:     ${BOARD_KIND} (RK/Orin 通用)"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " BIRD_WS:  ${BIRD_WS}"
echo " colcon:   ${COLCON_WS}"
echo "=========================================="

for d in "${BLE_DIR}" "${PULL_DIR}" "${VOICE_DIR}"; do
  if [ ! -d "${d}" ]; then
    echo "[error] 缺少子包: ${d}"
    exit 1
  fi
done
if [ ! -f "${VOICE_DIR}/__init__.py" ]; then
  echo "[error] voice_remind 不完整: ${VOICE_DIR}"
  exit 1
fi

cleanup_residuals

echo "[1/5] 安装系统依赖..."
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

echo "[2/5] 安装蓝牙 BLE（含语音提醒加载路径）..."
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
export BIRD_BLE_UID="${INSTALL_UID}"
export BIRD_WS="${BIRD_WS}"
export COLCON_WS="${COLCON_WS}"
if [ ! -f "${COLCON_WS}/install/setup.bash" ]; then
  for _cw in "${INSTALL_HOME}/hightorque_workspace" "${INSTALL_HOME}/colcon_ws"; do
    if [ -f "${_cw}/install/setup.bash" ]; then
      COLCON_WS="${_cw}"
      break
    fi
  done
fi
export COLCON_WS
unset _cw
export PKG_DIR="${BLE_DIR}"
export BT_DIR="${BLE_DIR}/app"

cat >/etc/default/bird-ble <<EOF
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
BIRD_BLE_UID=${INSTALL_UID}
BIRD_WS=${BIRD_WS}
COLCON_WS=${COLCON_WS}
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://${INSTALL_HOME}/cyclonedds.xml
BLE_DEVICE_NAME_FILE=/var/lib/bird-ble/ble_device_name.conf
EXTRA_ARGS=()
EOF

mkdir -p /var/lib/bird-ble
chmod 755 /var/lib/bird-ble
_NAME_SRC=""
if [ -f "${PKG_DIR}/ble_device_name.conf" ]; then
  _NAME_SRC="${PKG_DIR}/ble_device_name.conf"
elif [ -f "${BLE_DIR}/ble_device_name.conf" ]; then
  _NAME_SRC="${BLE_DIR}/ble_device_name.conf"
fi
if [ ! -f /var/lib/bird-ble/ble_device_name.conf ] && [ -n "${_NAME_SRC}" ]; then
  _bn="$(tr -d '[:space:]' < "${_NAME_SRC}")"
  if [ -n "${_bn}" ]; then
    echo "${_bn}" >/var/lib/bird-ble/ble_device_name.conf
    chmod 644 /var/lib/bird-ble/ble_device_name.conf
    echo "[ok] 广播名持久化: ${_bn}"
  fi
fi
unset _bn _NAME_SRC

chown "${INSTALL_USER}:${INSTALL_GID}" "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chown "${INSTALL_USER}:${INSTALL_GID}" "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${BLE_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod +x "${BLE_DIR}/app"/*.sh "${BLE_DIR}/app/scripts/"*.sh "${BLE_DIR}/app/systemd/"*.sh

"${BLE_DIR}/app/install-autostart.sh"

echo "[3/5] 安装拖拽控制（默认不开机自启，由小程序 PULL ON 启动）..."
chmod +x "${PULL_DIR}/app"/*.sh "${PULL_DIR}/app/systemd/"*.sh
export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
"${PULL_DIR}/app/install-autostart.sh"
systemctl disable torque-cmd-vel.service 2>/dev/null || true
systemctl stop torque-cmd-vel.service 2>/dev/null || true

echo "[4/5] 安装 ROS2 开机自启，关闭 ROS1 步态自启..."
chmod +x "${INSTALL_ROOT}/scripts/"*.sh 2>/dev/null || true
if [ -x "${INSTALL_ROOT}/scripts/install-ros2-autostart.sh" ]; then
  export COLCON_WS="${COLCON_WS}"
  export BIRD_USER="${INSTALL_USER}"
  "${INSTALL_ROOT}/scripts/install-ros2-autostart.sh"
else
  echo "[warn] 缺少 scripts/install-ros2-autostart.sh，跳过 ROS2 自启"
fi

echo "[5/5] 启动 ROS2 量产栈 → 蓝牙服务…"
systemctl daemon-reload
systemctl enable ros2-bringup.service 2>/dev/null || true
systemctl restart ros2-bringup.service 2>/dev/null || systemctl start ros2-bringup.service 2>/dev/null || true
echo "[info] 等待 ROS2 bringup 初始化（约 15s）…"
sleep 15
systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 5

set +u
[ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash
[ -f "${COLCON_WS}/install/setup.bash" ] && source "${COLCON_WS}/install/setup.bash"
set -u

python3 -c "import rclpy" 2>/dev/null && echo "[ok] rclpy" \
  || echo "[warn] 无法 import rclpy — 检查 ROS2 Foxy"
python3 -c "from sensor_msgs.msg import Joy; print('[ok] sensor_msgs/Joy')" 2>/dev/null \
  || echo "[warn] 无法 import sensor_msgs"
python3 -c "import sys; sys.path.insert(0, '${BIRD_WS}'); import voice_remind; print('[ok] voice_remind')" \
  || echo "[warn] voice_remind 导入失败"

echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 版本:     ${PKG_VERSION}"
echo " 正式目录: ${INSTALL_ROOT}"
echo " 蓝牙:     sudo systemctl status bird-ble"
echo " 拖拽:     sudo systemctl status torque-cmd-vel  # PULL ON 后运行"
echo " 语音:     ${VOICE_DIR}"
echo " 日志 BLE: journalctl -u bird-ble -f"
echo " 日志 ROS: journalctl -u ros2-bringup -f"
echo " ROS2自启: systemd ros2-bringup（仅此一路，桌面旧自启已删除）"
echo " ROS1自启: 已 Hidden（Pi_plus_start.desktop）"
"${BLE_DIR}/app/scripts/check.sh" || true
