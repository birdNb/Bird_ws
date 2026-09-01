#!/bin/bash
# ROS2 自启诊断（仅 systemd ros2-bringup）
set -uo pipefail

WS=/home/nvidia/hightorque_workspace

echo "========== ROS2 自启诊断 =========="
[ -f "${WS}/install/setup.bash" ] && echo "[OK] ${WS}/install/setup.bash" || echo "[!!] 缺 setup.bash"

echo ""
echo "--- 桌面自启（应已删除）---"
for f in Pi_plus_ros2.desktop hightorque_ros2_bfm.desktop; do
  p="/home/nvidia/.config/autostart/${f}"
  if [ -f "${p}" ]; then
    echo "[!!] 仍存在: ${p}"
    grep -E '^Exec=|^Hidden=' "${p}" || true
  else
    echo "[OK] 已删除: ${f}"
  fi
done

echo ""
echo "--- systemd ros2-bringup ---"
if [ -f /etc/systemd/system/ros2-bringup.service ]; then
  systemctl is-enabled ros2-bringup.service 2>/dev/null || echo "[!!] 未 enable"
  systemctl is-active ros2-bringup.service 2>/dev/null || echo "[!!] 未 active"
  grep -E '^ExecStart=' /etc/systemd/system/ros2-bringup.service || true
else
  echo "[!!] 未安装单元，请: sudo ~/Bird_ws/Bt-source-ros2/scripts/install-ros2-autostart.sh"
fi

echo ""
echo "--- 进程 ---"
pgrep -af 'bfm_real.launch.py|hightorque_controller_node|hightorque_midware_node' 2>/dev/null | head -8 \
  || echo "[!!] 无 bfm_real / controller / midware"

echo ""
echo "--- journal ---"
journalctl -u ros2-bringup -n 8 --no-pager 2>/dev/null || true
