#!/bin/bash
# 卸载 Bt_voice_pull_bag_0721 相关 systemd 服务（不删除本目录）
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./uninstall.sh"
  exit 1
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

echo "[ok] 已移除 bird-ble / torque-cmd-vel 开机服务"
echo "    安装目录未删除；需要时可手动 rm -rf 本目录"
