#!/bin/bash
# 检查板子 BLE 广播是否在发（小程序扫的是 BLE，不是系统蓝牙配对列表）
set -euo pipefail

echo "========== 蓝牙适配器 =========="
if ! hciconfig hci0 2>/dev/null | grep -q "hci0"; then
  echo "[!!] hci0 不存在 — 系统无蓝牙控制器"
  echo "     运行: sudo ./scripts/recover.sh  或  sudo reboot"
  echo ""
fi
bluetoothctl show 2>/dev/null | grep -E 'Controller|Name:|Alias:|Powered|Discoverable|Discovering' || true
echo ""
echo "========== LE 广播状态 (btmgmt) =========="
if command -v btmgmt >/dev/null; then
  btmgmt info 2>/dev/null | grep -E 'current settings|name |addr ' || true
  if btmgmt info 2>/dev/null | grep -q 'advertising'; then
    echo "[OK] LE advertising 已开启（小程序可扫描）"
  else
    echo "[!!] LE advertising 未开启 — 请先运行: cd $(dirname "$0") && ./start.sh"
  fi
else
  echo "未安装 btmgmt，跳过"
fi
echo ""
echo "========== 说明 =========="
echo "1. EDIFIER BLE 是附近耳机/音箱，不是你的板子，可忽略"
echo "2. 板子 MAC 一般是: 00:19:86:00:2E:AF（以 bluetoothctl show 为准）"
echo "3. 小程序用 services=[FFE0] 扫描，见 BLE_PROTOCOL.md"
echo "4. ./start.sh 必须保持运行；只在小程序连接，勿系统蓝牙配对"
echo "5. Discoverable/Pairable 应为 off，否则手机会一直弹连接框"
