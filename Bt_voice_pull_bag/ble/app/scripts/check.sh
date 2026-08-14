#!/bin/bash
# 检查 BLE 广播状态
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../platform_hw.sh"
cd "${BT_DIR}"
cd "${BT_DIR}"

echo "========== 平台 =========="
echo "主控: ${BLE_PLATFORM} | ${BLE_BOARD_NAME}"
echo "蓝牙: ${BLE_HW_DESC} (${BLE_CHIP} → ${BLE_HCI_DEV})"
echo "广播模式: ${BLE_ADV_MODE}"
echo "工作目录: ${BT_DIR}"
echo ""

if ! hciconfig "${BLE_HCI_DEV}" 2>/dev/null | grep -q "${BLE_HCI_DEV}"; then
  echo "[!!] ${BLE_HCI_DEV} 不存在"
  echo "     sudo ${BT_DIR}/scripts/recover.sh"
  echo ""
fi
bluetoothctl show 2>/dev/null | grep -E 'Controller|Name:|Alias:|Powered|Discoverable' || true
MAC=$(bluetoothctl show 2>/dev/null | awk '/Controller/ {print $2}' | head -1)
echo ""

echo "========== BLE 服务 =========="
# 自启入口为 ble_gatt_boot；手工 start.sh 也可能直接跑 ble_gatt_server
if pgrep -f 'ble_gatt_boot\.py[c]?|ble_gatt_server\.py[c]?' >/dev/null; then
  echo "[OK] BLE GATT 进程运行中"
elif systemctl is-active --quiet bird-ble.service 2>/dev/null; then
  echo "[OK] bird-ble.service active"
else
  echo "[!!] 未运行 — sudo systemctl start bird-ble  或  cd ${BT_DIR} && ./start.sh"
fi
echo ""
echo "板子 MAC: ${MAC:-见 bluetoothctl show}"
echo "广播名: $(python3 -c "from ble_device_name import load_ble_name; print(load_ble_name())" 2>/dev/null || echo HT_88888888)"
echo ""
echo "说明:"
echo "  - Orin: 外接 USB 蓝牙模块"
echo "  - RK3588: 板载 RTL8822CE 一体网卡（非 USB 棒）"
echo "  - 小程序内扫描 FFE0，勿用手机系统蓝牙配对"
echo "  - Discovering 只读告警可忽略"
