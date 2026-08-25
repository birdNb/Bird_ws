#!/bin/bash
# 卸载 Bt-source-ros2 相关 systemd 服务（默认不删正式安装目录）
set -euo pipefail

_detect_board_kind() {
  local model=""
  if [ -r /proc/device-tree/model ]; then
    model="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)"
  fi
  model="$(printf '%s' "${model}" | tr '[:upper:]' '[:lower:]')"
  case "${model}" in
    *orin*|*jetson*) echo orin; return ;;
    *rk3588*|*rk35*|*lubancat*|*rockchip*) echo rk; return ;;
  esac
  if [ -f /etc/nv_tegra_release ] || lsmod 2>/dev/null | grep -qi tegra; then
    echo orin
    return
  fi
  echo rk
}

if [ "$(id -u)" -ne 0 ]; then
  if sudo -n true 2>/dev/null; then
    exec sudo -E bash "$0" "$@"
  fi
  kind="$(_detect_board_kind)"
  if [ "${kind}" = "orin" ]; then
    pass="nvidia"
  else
    pass="ht"
  fi
  printf '%s\n' "${pass}" | sudo -S -E bash "$0" "$@"
  exit $?
fi

for u in bird-ble.service torque-cmd-vel.service; do
  systemctl stop "${u}" 2>/dev/null || true
  systemctl disable "${u}" 2>/dev/null || true
  rm -f "/etc/systemd/system/${u}"
done

pkill -f 'ble_gatt_server\.pyc?' 2>/dev/null || true
pkill -f 'torque_cmd_vel_bridge\.pyc?' 2>/dev/null || true

systemctl daemon-reload
systemctl reset-failed 2>/dev/null || true

ROS2_DESKTOP="/home/${SUDO_USER:-hightorque}/.config/autostart/Pi_plus_ros2.desktop"
if [ -f "${ROS2_DESKTOP}" ]; then
  if grep -q '^Hidden=' "${ROS2_DESKTOP}"; then
    sed -i 's/^Hidden=.*/Hidden=true/' "${ROS2_DESKTOP}"
  else
    printf '\nHidden=true\n' >>"${ROS2_DESKTOP}"
  fi
  echo "[ok] 已隐藏 ROS2 步态自启: ${ROS2_DESKTOP}"
fi

echo "[ok] 已移除 bird-ble / torque-cmd-vel 开机服务"
echo "    正式目录默认保留: ~/Bird_ws/Bt-source-ros2"
echo "    若需删除: rm -rf ~/Bird_ws/Bt-source-ros2"
