#!/bin/bash
# 一键修复 Yesense 串口权限（需 sudo）——解决 GAIT ON 因无 /imu 被拒
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ "$(id -u)" -ne 0 ]; then
  echo "请执行: sudo $0" >&2
  exit 1
fi
install -m 0644 "${SCRIPT_DIR}/99-bird-yesense-imu.rules" /etc/udev/rules.d/99-bird-yesense-imu.rules
udevadm control --reload-rules || true
udevadm trigger --subsystem-match=tty || true
usermod -aG dialout "${SUDO_USER:-nvidia}" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/ensure_imu_serial.sh"
"${SCRIPT_DIR}/ensure_imu_serial.sh"
# 双保险
[ -c /dev/ttyUSB1 ] && chmod 0666 /dev/ttyUSB1
ln -sfn /dev/ttyUSB1 /dev/ttyUSB0 2>/dev/null || true
echo "==== 结果 ===="
ls -la /dev/ttyUSB* || true
python3 -c "open('/dev/ttyUSB0','rb').close(); print('open /dev/ttyUSB0 OK')"
echo "yesense 每 5s 重试；约 10s 后应有 /imu。再试小程序 GAIT ON（一般不必重启 ROS2）。"
