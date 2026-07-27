#!/bin/bash
# 将 pull_move_demo 打成可移植发布包（字节码，不含 .py 源码）
# 输出目录: ../pull_move_0717_bag
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="$(tr -d '[:space:]' < "${SRC_DIR}/VERSION" 2>/dev/null || echo 0717)"
OUT_NAME="pull_move_${VERSION}_bag"
OUT_DIR="$(cd "${SRC_DIR}/.." && pwd)/${OUT_NAME}"
PY_FILES=(
  torque_cmd_vel_bridge.py
  monitor_r_shoulder_torque.py
  monitor_neck_torque.py
)

echo "[build] 源目录: ${SRC_DIR}"
echo "[build] 输出:   ${OUT_DIR}"
echo "[build] 版本:   ${VERSION}"

rm -rf "${OUT_DIR}"
APP_DIR="${OUT_DIR}/app"
mkdir -p "${APP_DIR}/systemd"

for f in run_torque_bridge.sh run_pitch_bridge.sh run_monitor.sh run_neck_monitor.sh install-autostart.sh; do
  cp "${SRC_DIR}/${f}" "${APP_DIR}/"
done
cp "${SRC_DIR}/systemd/"* "${APP_DIR}/systemd/"

for py in "${PY_FILES[@]}"; do
  if [ ! -f "${SRC_DIR}/${py}" ]; then
    echo "[warn] 跳过缺失: ${py}"
    continue
  fi
  cp "${SRC_DIR}/${py}" "${APP_DIR}/"
done

cd "${APP_DIR}"
python3 -m compileall -b -q .
for py in "${PY_FILES[@]}"; do
  rm -f "${py}"
done

for sh in run_torque_bridge.sh run_monitor.sh run_neck_monitor.sh systemd/torque-cmd-vel-boot.sh; do
  sed -i 's/\.py"/.pyc"/g; s/\.py /.pyc /g; s/\.py$/.pyc/' "${sh}" 2>/dev/null || true
done
sed -i 's/torque_cmd_vel_bridge\.py/torque_cmd_vel_bridge.pyc/g' install-autostart.sh
# chmod 对缺失文件不失败（兼容 .py / .pyc）
sed -i '/torque_cmd_vel_bridge\.pyc"/d' install-autostart.sh
if ! grep -q 'torque_cmd_vel_bridge.pyc 2>/dev/null' install-autostart.sh; then
  sed -i '/run_pitch_bridge\.sh"/a chmod +x "${DEMO_DIR}"/torque_cmd_vel_bridge.pyc 2>/dev/null || true\nchmod +x "${DEMO_DIR}"/torque_cmd_vel_bridge.py 2>/dev/null || true' install-autostart.sh
fi

chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/systemd/"*.sh

cat >"${OUT_DIR}/install.sh" <<'INSTALL_EOF'
#!/bin/bash
# 力矩拖拽控制 一键安装：systemd 开机自启
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="${PKG_DIR}/app"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"

echo "=========================================="
echo " pull_move 力矩→cmd_vel 安装包"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " ROS 工作空间: ${SIM2REAL_WS}"
echo "=========================================="

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "[warn] 未检测到 ROS Noetic，请先安装 ROS 与 sim2real"
else
  echo "[ok] ROS Noetic 已安装"
fi

export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"

chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/systemd/"*.sh

echo "[1/2] 安装 systemd 开机自启..."
"${APP_DIR}/install-autostart.sh"

echo "[2/2] 检查 sim2real..."
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/install"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/devel"
else
  echo "[warn] 未找到 sim2real（${SIM2REAL_WS}）"
fi

systemctl restart torque-cmd-vel.service || systemctl start torque-cmd-vel.service || true
sleep 3

echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 状态: sudo systemctl status torque-cmd-vel"
echo " 日志: journalctl -u torque-cmd-vel -f"
echo " 手动: cd ${APP_DIR} && ./run_torque_bridge.sh"
echo ""
echo " 仅 FSM=行走模式(EXEC_DEFAULT) 时发布 /cmd_vel"
INSTALL_EOF

cat >"${OUT_DIR}/uninstall.sh" <<'UNINSTALL_EOF'
#!/bin/bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./uninstall.sh"
  exit 1
fi
systemctl stop torque-cmd-vel.service 2>/dev/null || true
systemctl disable torque-cmd-vel.service 2>/dev/null || true
rm -f /etc/systemd/system/torque-cmd-vel.service
systemctl daemon-reload
echo "[ok] 已移除 torque-cmd-vel 开机自启（安装目录未删除）"
UNINSTALL_EOF

chmod +x "${OUT_DIR}/install.sh" "${OUT_DIR}/uninstall.sh"

cat >"${OUT_DIR}/README.md" <<README_EOF
# pull_move 力矩拖拽控制安装包 (pull_move_${VERSION}_bag)

肩/脖子力矩 → \`/cmd_vel\` 映射桥。仅在 **FSM=行走模式(EXEC_DEFAULT)** 时发布速度。

本包为**发布运行时**（Python 字节码，不含 .py 源码）。

## 一键安装

\`\`\`bash
cd pull_move_${VERSION}_bag
sudo ./install.sh
\`\`\`

## 前置条件

- Ubuntu 20.04 + ROS Noetic
- 已编译 \`sim2real\` 工作空间
- 机器人主控已运行 \`roscore\` / \`sim2real_master\`

## 目录结构

\`\`\`
pull_move_${VERSION}_bag/
  install.sh / uninstall.sh
  README.md / VERSION
  app/                    # 运行时（字节码 + 脚本）
\`\`\`

## 常用命令

\`\`\`bash
sudo systemctl status torque-cmd-vel
journalctl -u torque-cmd-vel -f
cd app && ./run_torque_bridge.sh          # 前台调试
cd app && ./run_torque_bridge.sh --dry-run
sudo ./uninstall.sh
\`\`\`

## 可选参数

编辑 \`/etc/default/torque-cmd-vel\`：

\`\`\`bash
EXTRA_ARGS=(--no-arms)    # 仅脖子
EXTRA_ARGS=(--side right)
\`\`\`

然后 \`sudo systemctl restart torque-cmd-vel\`
README_EOF

echo "${VERSION}" >"${OUT_DIR}/VERSION"

cd "$(dirname "${OUT_DIR}")"
tar czf "${OUT_NAME}.tar.gz" "$(basename "${OUT_DIR}")"
echo "[build] 完成: ${OUT_DIR}"
echo "[build] 压缩包: $(dirname "${OUT_DIR}")/${OUT_NAME}.tar.gz"
if find "${APP_DIR}" -name '*.py' | grep -q .; then
  echo "[error] app/ 仍含 .py" >&2
  exit 1
fi
echo "[ok] app/ 无 .py 源码"
ls -la "${OUT_DIR}"
