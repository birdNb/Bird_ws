#!/bin/bash
# 保证 Yesense IMU 可用在量产配置的 /dev/ttyUSB0（不改 hightorque 源码）。
#
# 常见坑：
# 1) CP2102 枚举成 ttyUSB1 → 需软链到 ttyUSB0
# 2) 旧软链 ttyUSB0→ttyUSB1 在设备重枚举成 ttyUSB0 后变成悬空链，盖住真节点 → 无 /imu
# 3) 权限 660 + 用户不在 dialout → Permission denied
set -euo pipefail

TARGET="${IMU_TTY_TARGET:-/dev/ttyUSB0}"
BY_ID_ROOT="/dev/serial/by-id"
ALIAS="/dev/yesense_imu"

_yesense_by_id() {
  if [ -d "${BY_ID_ROOT}" ]; then
    ls -1 "${BY_ID_ROOT}"/usb-Silicon_Labs_CP2102* 2>/dev/null || true
    ls -1 "${BY_ID_ROOT}"/usb-*Yesense* 2>/dev/null || true
    ls -1 "${BY_ID_ROOT}"/usb-*YESENSE* 2>/dev/null || true
  fi
}

_resolve_real() {
  local p="$1"
  if [ -L "$p" ]; then
    readlink -f "$p" 2>/dev/null || realpath "$p" 2>/dev/null || true
  else
    echo "$p"
  fi
}

_is_usable_chr() {
  [ -n "${1:-}" ] && [ -c "$1" ] && [ ! -L "$1" ]
}

_relax_perm() {
  local real="$1"
  _is_usable_chr "${real}" || return 0
  chmod 0666 "${real}" 2>/dev/null || true
  echo "[ensure_imu_serial] 权限 $(stat -c '%a %U:%G' "${real}" 2>/dev/null || echo '?') ${real}"
}

_rebind_hint() {
  echo "[ensure_imu_serial][warn] 未找到可用 CP2102 字符设备。可试:" >&2
  echo "  sudo rm -f /dev/ttyUSB0; sudo udevadm trigger --subsystem-match=tty; sleep 1; $0" >&2
}

# 1) 清掉悬空 / 错误软链（否则会挡住内核新建的真 ttyUSB0）
if [ -L "${TARGET}" ]; then
  real="$(_resolve_real "${TARGET}" || true)"
  if ! _is_usable_chr "${real}"; then
    echo "[ensure_imu_serial] 删除悬空软链 ${TARGET} (-> ${real:-missing})"
    rm -f "${TARGET}"
  fi
fi

# 2) 找真设备：by-id → yesense_imu → ttyUSB*
SRC=""
for c in $(_yesense_by_id); do
  real="$(_resolve_real "$c")"
  if _is_usable_chr "${real}"; then
    SRC="${real}"
    break
  fi
done

if [ -z "${SRC}" ] && _is_usable_chr "${ALIAS}"; then
  SRC="${ALIAS}"
fi

if [ -z "${SRC}" ]; then
  for n in 0 1 2 3; do
    if _is_usable_chr "/dev/ttyUSB${n}"; then
      # 多个时优先选 Silicon Labs 已匹配的；此处仅回退
      SRC="/dev/ttyUSB${n}"
      break
    fi
  done
fi

# 仍没有：尝试 udev 触发后再找一次
if [ -z "${SRC}" ]; then
  udevadm trigger --subsystem-match=tty 2>/dev/null || true
  sleep 0.8
  for c in $(_yesense_by_id); do
    real="$(_resolve_real "$c")"
    if _is_usable_chr "${real}"; then
      SRC="${real}"
      break
    fi
  done
fi

if [ -z "${SRC}" ]; then
  _rebind_hint
  exit 0
fi

_relax_perm "${SRC}"

# 3) TARGET 已是真节点且就是 SRC：完成
if _is_usable_chr "${TARGET}" && [ "$(_resolve_real "${TARGET}")" = "$(_resolve_real "${SRC}")" ]; then
  echo "[ensure_imu_serial] ${TARGET} 已是可用串口 (${SRC})"
  exit 0
fi

# 4) TARGET 已是其它真串口：不覆盖
if _is_usable_chr "${TARGET}" && [ "${TARGET}" != "${SRC}" ]; then
  echo "[ensure_imu_serial][warn] ${TARGET} 已是其它字符设备，不覆盖；IMU 在 ${SRC}" >&2
  # 若量产写死 ttyUSB0 而设备在 USB1，仍需要链——仅当 TARGET 不存在时链
  exit 0
fi

# 5) 需要 TARGET 指向 SRC
if [ -L "${TARGET}" ] || [ -e "${TARGET}" ]; then
  rm -f "${TARGET}"
fi

if [ "${TARGET}" = "${SRC}" ]; then
  echo "[ensure_imu_serial] 使用 ${SRC}"
  exit 0
fi

ln -sfn "${SRC}" "${TARGET}"
echo "[ensure_imu_serial] 已链接 ${TARGET} -> ${SRC}"
_relax_perm "${SRC}"
