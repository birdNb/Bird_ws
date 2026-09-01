#!/bin/bash
# 保证 Yesense IMU 落在量产配置的 /dev/ttyUSB0（不改 hightorque 源码）。
# CP2102N 常被枚举成 ttyUSB1；且默认 660 root:dialout，未进 dialout 的用户会
# Permission denied → 无 /imu → GAIT ON 被拒。
set -euo pipefail

TARGET="${IMU_TTY_TARGET:-/dev/ttyUSB0}"
BY_ID_ROOT="/dev/serial/by-id"

_yesense_candidates() {
  if [ -d "${BY_ID_ROOT}" ]; then
    ls -1 "${BY_ID_ROOT}"/usb-Silicon_Labs_CP2102* 2>/dev/null || true
    ls -1 "${BY_ID_ROOT}"/usb-*Yesense* 2>/dev/null || true
    ls -1 "${BY_ID_ROOT}"/usb-*YESENSE* 2>/dev/null || true
  fi
}

_resolve_real() {
  local p="$1"
  if [ -L "$p" ]; then
    readlink -f "$p" 2>/dev/null || realpath "$p" 2>/dev/null || echo "$p"
  else
    echo "$p"
  fi
}

_relax_perm() {
  local real="$1"
  [ -c "${real}" ] || return 0
  chmod 0666 "${real}" 2>/dev/null || true
  echo "[ensure_imu_serial] 权限 $(stat -c '%a %U:%G' "${real}" 2>/dev/null || echo '?') ${real}"
}

SRC=""
for c in $(_yesense_candidates); do
  real="$(_resolve_real "$c")"
  if [ -c "${real}" ]; then
    SRC="${real}"
    break
  fi
done

if [ -z "${SRC}" ] && [ -c /dev/ttyUSB1 ]; then
  SRC="/dev/ttyUSB1"
fi
if [ -z "${SRC}" ] && [ -c /dev/ttyUSB0 ] && [ ! -L /dev/ttyUSB0 ]; then
  SRC="/dev/ttyUSB0"
fi

if [ -z "${SRC}" ]; then
  echo "[ensure_imu_serial][warn] 未找到 Yesense/CP2102 串口" >&2
  exit 0
fi

_relax_perm "${SRC}"

if [ -c "${TARGET}" ] && [ ! -L "${TARGET}" ]; then
  if [ "${TARGET}" = "${SRC}" ] || [ "$(_resolve_real "${TARGET}")" = "$(_resolve_real "${SRC}")" ]; then
    echo "[ensure_imu_serial] ${TARGET} 已是真实串口"
    exit 0
  fi
  echo "[ensure_imu_serial][warn] ${TARGET} 已是其它真实串口，不覆盖；已放宽 ${SRC}" >&2
  exit 0
fi

if [ -L "${TARGET}" ]; then
  cur="$(_resolve_real "${TARGET}")"
  if [ "${cur}" = "$(_resolve_real "${SRC}")" ]; then
    echo "[ensure_imu_serial] ${TARGET} -> ${SRC} 已正确"
    exit 0
  fi
  rm -f "${TARGET}"
fi

if [ "${TARGET}" != "${SRC}" ]; then
  ln -sfn "${SRC}" "${TARGET}"
  echo "[ensure_imu_serial] 已链接 ${TARGET} -> ${SRC}"
else
  echo "[ensure_imu_serial] 使用 ${SRC}"
fi
