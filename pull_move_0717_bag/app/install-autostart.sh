#!/bin/bash
# 安装力矩→速度桥 开机自启动（systemd）
# 运行时仅 FSM=EXEC_DEFAULT(5) 行走模式才发布 /cmd_vel
set -euo pipefail

DEMO_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_NAME="torque-cmd-vel.service"
UNIT_SRC="${DEMO_DIR}/systemd/torque-cmd-vel.service"
BOOT_SRC="${DEMO_DIR}/systemd/torque-cmd-vel-boot.sh"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_ENV="/etc/default/torque-cmd-vel"

BIRD_USER="${BIRD_USER:-$(stat -c '%U' "${DEMO_DIR}" 2>/dev/null || echo hightorque)}"
if [ "${BIRD_USER}" = "root" ]; then
  BIRD_USER="hightorque"
fi
BIRD_HOME="${BIRD_HOME:-/home/${BIRD_USER}}"
BIRD_GROUP="${BIRD_GROUP:-$(id -gn "${BIRD_USER}" 2>/dev/null || echo "${BIRD_USER}")}"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo $0"
  exit 1
fi

chmod +x \
  "${BOOT_SRC}" \
  "${DEMO_DIR}/run_torque_bridge.sh" \
  "${DEMO_DIR}/run_pitch_bridge.sh" \
  "${DEMO_DIR}/torque_cmd_vel_bridge.pyc"

sed \
  -e "s|@DEMO_DIR@|${DEMO_DIR}|g" \
  -e "s|@BIRD_USER@|${BIRD_USER}|g" \
  -e "s|@BIRD_GROUP@|${BIRD_GROUP}|g" \
  -e "s|@BIRD_HOME@|${BIRD_HOME}|g" \
  "${UNIT_SRC}" >"${UNIT_DST}"

if [ ! -f "${DEFAULT_ENV}" ]; then
  cat >"${DEFAULT_ENV}" <<'EOF'
# torque-cmd-vel 额外启动参数（传给 torque_cmd_vel_bridge.pyc）
# 示例：仅用脖子 / 仅用右手
# EXTRA_ARGS=(--no-arms)
# EXTRA_ARGS=(--side right)
# 注意：不要加 --no-fsm；开机服务会强制保留 FSM 行走模式守门
EXTRA_ARGS=()
EOF
  echo "[ok] 已创建 ${DEFAULT_ENV}"
fi

systemctl daemon-reload
systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
systemctl stop "${SERVICE_NAME}" 2>/dev/null || true

echo ""
echo "=========================================="
echo " torque-cmd-vel 已安装（默认不开机自启）"
echo " 服务名: ${SERVICE_NAME}"
echo " FSM: 仅 EXEC_DEFAULT(5)=行走模式 发 /cmd_vel"
echo " 开启: 小程序 PULL ON 或 sudo systemctl start"
echo "=========================================="
echo ""
echo "常用命令:"
echo "  sudo systemctl start ${SERVICE_NAME}    # 立即启动"
echo "  sudo systemctl stop ${SERVICE_NAME}     # 停止"
echo "  sudo systemctl status ${SERVICE_NAME}   # 状态"
echo "  journalctl -u ${SERVICE_NAME} -f        # 日志"
echo ""
echo "修改参数: 编辑 ${DEFAULT_ENV} 后"
echo "  sudo systemctl daemon-reload"
echo "  sudo systemctl restart ${SERVICE_NAME}"
