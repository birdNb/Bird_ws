#!/bin/bash
# 一键修复 Yesense 串口（需 sudo）——解决悬空 ttyUSB0 软链 / 权限导致无 /imu、GAIT ON 被拒
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ "$(id -u)" -ne 0 ]; then
  echo "请执行: sudo $0" >&2
  exit 1
fi

install -m 0644 "${SCRIPT_DIR}/99-bird-yesense-imu.rules" /etc/udev/rules.d/99-bird-yesense-imu.rules
udevadm control --reload-rules || true

# 先拆掉可能挡住真设备的悬空链
if [ -L /dev/ttyUSB0 ]; then
  echo "[fix] 删除软链 /dev/ttyUSB0 -> $(readlink /dev/ttyUSB0)"
  rm -f /dev/ttyUSB0
fi

udevadm trigger --subsystem-match=tty || true
sleep 1

# 若仍无字符设备，尝试重绑 CP210x
if [ ! -c /dev/ttyUSB0 ] && [ ! -c /dev/ttyUSB1 ] && [ ! -c /dev/yesense_imu ]; then
  for d in /sys/bus/usb/drivers/cp210x/*; do
    [ -e "$d" ] || continue
    name="$(basename "$d")"
    case "${name}" in
      bind|unbind|module|uevent) continue ;;
    esac
    echo "[fix] rebind cp210x ${name}"
    echo "${name}" > /sys/bus/usb/drivers/cp210x/unbind 2>/dev/null || true
    sleep 0.3
    echo "${name}" > /sys/bus/usb/drivers/cp210x/bind 2>/dev/null || true
  done
  sleep 1
  udevadm trigger --subsystem-match=tty || true
  sleep 0.5
fi

usermod -aG dialout "${SUDO_USER:-nvidia}" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/ensure_imu_serial.sh"
"${SCRIPT_DIR}/ensure_imu_serial.sh"

echo "==== 结果 ===="
ls -la /dev/ttyUSB* /dev/yesense_imu 2>/dev/null || true
python3 - <<'PY'
import os
for p in ("/dev/ttyUSB0", "/dev/yesense_imu", "/dev/ttyUSB1"):
    try:
        open(p, "rb").close()
        print("open OK", p, "->", os.path.realpath(p))
        break
    except Exception as e:
        print("open fail", p, e)
else:
    raise SystemExit(1)
PY
echo "请重新 ./ros2_start.sh，确认不再刷 Unable to open serial port，再试 GAIT ON。"
