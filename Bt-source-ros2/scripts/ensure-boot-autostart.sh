#!/bin/bash
# 一键确认/修复开机自启：ros2-bringup + bird-ble + IMU 串口
# 用法: sudo ~/Bird_ws/Bt-source-ros2/scripts/ensure-boot-autostart.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BLE_DIR="$(cd "${SCRIPT_DIR}/../ble" && pwd)"
INSTALL_USER="${SUDO_USER:-nvidia}"

echo "========== Bird 开机自启确认 =========="

chmod 0755 \
  "${SCRIPT_DIR}/ensure_imu_serial.sh" \
  "${SCRIPT_DIR}/fix_imu_perm.sh" \
  "${SCRIPT_DIR}/ros2-bringup-boot.sh" \
  "${SCRIPT_DIR}/prepare-cyclonedds-runtime.sh" \
  "${SCRIPT_DIR}/ros2_start.sh" \
  "${SCRIPT_DIR}/ros2-kill-old-stack.sh" \
  "${SCRIPT_DIR}/ensure-boot-autostart.sh" 2>/dev/null || true

# 1) IMU udev + 立即修复
if [ -f "${SCRIPT_DIR}/99-bird-yesense-imu.rules" ]; then
  install -m 0644 "${SCRIPT_DIR}/99-bird-yesense-imu.rules" /etc/udev/rules.d/99-bird-yesense-imu.rules
  udevadm control --reload-rules || true
  udevadm trigger --subsystem-match=tty || true
fi
usermod -aG dialout "${INSTALL_USER}" 2>/dev/null || true
if [ -L /dev/ttyUSB0 ] && [ ! -c /dev/ttyUSB0 ]; then
  rm -f /dev/ttyUSB0
fi
"${SCRIPT_DIR}/ensure_imu_serial.sh" || true

# 2) IMU oneshot（root，先于 ROS2）
install -m 0644 "${SCRIPT_DIR}/bird-imu-serial.service" /etc/systemd/system/bird-imu-serial.service

# 3) ROS2 bringup
install -m 0644 "${SCRIPT_DIR}/ros2-bringup.service" /etc/systemd/system/ros2-bringup.service

# 4) BLE：若尚未装过则跑 install；已装则保证 enable
if [ ! -f /etc/systemd/system/bird-ble.service ]; then
  echo "[info] 安装 bird-ble ..."
  (cd "${BLE_DIR}" && ./install.sh)
else
  # 刷新 unit（保持路径与当前树一致）
  if [ -x "${BLE_DIR}/app/install-autostart.sh" ]; then
    export BIRD_USER="${INSTALL_USER}"
    export BIRD_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
    export COLCON_WS="/home/nvidia/hightorque_workspace"
    export PKG_DIR="${BLE_DIR}"
    # install-autostart 会 enable+可能 restart；此处仅 enable 也可
    true
  fi
fi

systemctl daemon-reload
systemctl enable bird-imu-serial.service
systemctl enable ros2-bringup.service
systemctl enable bird-ble.service
systemctl enable bluetooth.service 2>/dev/null || true

# 拉起 IMU 修复（不强制此刻重启整栈，避免打断你正在跑的手动测试；重启后会按序启动）
systemctl start bird-imu-serial.service || true

echo ""
echo "========== 当前状态 =========="
printf "bird-imu-serial: %s / %s\n" "$(systemctl is-enabled bird-imu-serial 2>/dev/null || echo n/a)" "$(systemctl is-active bird-imu-serial 2>/dev/null || echo n/a)"
printf "ros2-bringup:    %s / %s\n" "$(systemctl is-enabled ros2-bringup 2>/dev/null || echo n/a)" "$(systemctl is-active ros2-bringup 2>/dev/null || echo n/a)"
printf "bird-ble:        %s / %s\n" "$(systemctl is-enabled bird-ble 2>/dev/null || echo n/a)" "$(systemctl is-active bird-ble 2>/dev/null || echo n/a)"
printf "bluetooth:       %s / %s\n" "$(systemctl is-enabled bluetooth 2>/dev/null || echo n/a)" "$(systemctl is-active bluetooth 2>/dev/null || echo n/a)"
echo ""
ls -la /dev/ttyUSB* /dev/yesense_imu 2>/dev/null || echo "[warn] 未见 IMU 串口"
echo ""
echo "开机顺序: bird-imu-serial → ros2-bringup → bird-ble"
echo "重启验证前建议先停掉手动 ./ros2_start.sh（Ctrl+C），再:"
echo "  sudo reboot"
echo ""
echo "重启后检查:"
echo "  systemctl is-active ros2-bringup bird-ble"
echo "  journalctl -u ros2-bringup -b --no-pager | tail -30"
echo "  journalctl -u bird-ble -b --no-pager | tail -30"
