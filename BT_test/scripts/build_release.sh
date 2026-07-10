#!/bin/bash
# 将 BT_test 打成可移植发布包（字节码，不含 .py 源码）
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$(cd "${SRC_DIR}/.." && pwd)/BT_Control_0710"
VERSION="0710"
PY_FILES=(
  ble_gatt_server.py
  ble_ros_bridge.py
  ble_command_dispatcher.py
  ble_legacy_adv.py
  ble_status_telemetry.py
  ble_neck_bridge.py
  ble_motor_power_manager.py
  ble_locate_face_manager.py
  ble_volume_manager.py
  ble_log.py
  ble_device_name.py
  ble_advertise.py
  platform_detect.py
  neck_smooth_home.py
)

echo "[build] 源目录: ${SRC_DIR}"
echo "[build] 输出:   ${OUT_DIR}"

rm -rf "${OUT_DIR}"
APP_DIR="${OUT_DIR}/app"
mkdir -p "${APP_DIR}/systemd" "${APP_DIR}/scripts" "${OUT_DIR}/docs"

# 包根目录：仅配置与入口（用户可编辑蓝牙名）
cp "${SRC_DIR}/ble_device_name.conf" "${OUT_DIR}/"
chmod 664 "${OUT_DIR}/ble_device_name.conf" 2>/dev/null || true
if [ -n "${SUDO_USER:-}" ]; then
  chown "${SUDO_USER}:$(id -g "${SUDO_USER}" 2>/dev/null || echo 1000)" "${OUT_DIR}/ble_device_name.conf" 2>/dev/null || true
elif [ "$(id -u)" -ne 0 ]; then
  chown "$(id -un):$(id -gn)" "${OUT_DIR}/ble_device_name.conf" 2>/dev/null || true
fi

# 运行时收进 app/
for f in start.sh run_ble_with_ros.sh ros_env.sh platform_env.sh platform_hw.sh install-autostart.sh; do
  cp "${SRC_DIR}/${f}" "${APP_DIR}/"
done
cp "${SRC_DIR}/systemd/"* "${APP_DIR}/systemd/"
cp "${SRC_DIR}/scripts/check.sh" "${SRC_DIR}/scripts/recover.sh" "${APP_DIR}/scripts/"
cp "${SRC_DIR}/BLE_PROTOCOL.md" "${OUT_DIR}/docs/"
cp "${SRC_DIR}/docs/miniprogram_ble_snippet.js" "${OUT_DIR}/docs/" 2>/dev/null || true

# Python → 字节码（放入 app/）
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

# 入口脚本改为调用 .pyc
sed -i 's/ble_gatt_server\.py/ble_gatt_server.pyc/g' run_ble_with_ros.sh
sed -i 's/ble_gatt_server\.py/ble_gatt_server.pyc/g' scripts/check.sh

chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/scripts/"*.sh "${APP_DIR}/systemd/"*.sh

# 安装/卸载脚本（由模板写入）
cat >"${OUT_DIR}/install.sh" <<'INSTALL_EOF'
#!/bin/bash
# Bird BLE 一键安装：系统依赖 + 开机自启
set -euo pipefail

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="${PKG_DIR}/app"
cd "${PKG_DIR}"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./install.sh"
  exit 1
fi

# 检测运行用户（安装 sudo 的登录用户）
INSTALL_USER="${SUDO_USER:-${BIRD_USER:-hightorque}}"
INSTALL_HOME="$(getent passwd "${INSTALL_USER}" | cut -d: -f6)"
INSTALL_UID="$(id -u "${INSTALL_USER}" 2>/dev/null || echo 1000)"
SIM2REAL_WS="${SIM2REAL_WS:-${INSTALL_HOME}/sim2real}"
BIRD_WS="${BIRD_WS:-${INSTALL_HOME}/Bird_ws}"
INSTALL_GID="$(id -g "${INSTALL_USER}" 2>/dev/null || echo 1000)"

echo "=========================================="
echo " Bird BLE 遥控安装包"
echo " 安装目录: ${PKG_DIR}"
echo " 运行用户: ${INSTALL_USER}"
echo " ROS 工作空间: ${SIM2REAL_WS}"
echo "=========================================="

echo "[1/4] 安装系统依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  bluez bluez-tools \
  python3 python3-dbus python3-gi \
  rfkill

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "[warn] 未检测到 ROS Noetic，请先安装 ROS 与 sim2real_msg"
  echo "       BLE 可安装，但模式/动作控制需 ROS 环境"
else
  echo "[ok] ROS Noetic 已安装"
fi

echo "[2/4] 配置 BlueZ Experimental..."
CONF="/etc/bluetooth/main.conf"
if [ -f "${CONF}" ] && ! grep -qE '^[[:space:]]*Experimental[[:space:]]*=[[:space:]]*true' "${CONF}" 2>/dev/null; then
  BACKUP="${CONF}.bak.$(date +%Y%m%d%H%M%S)"
  cp "${CONF}" "${BACKUP}"
  if grep -q '^\[General\]' "${CONF}"; then
    sed -i '/^\[General\]/a Experimental=true' "${CONF}"
  else
    printf '%s\n' '[General]' 'Experimental=true' >>"${CONF}"
  fi
  systemctl restart bluetooth
  sleep 2
  echo "[ok] 已启用 Experimental=true（备份: ${BACKUP}）"
fi

echo "[3/4] 写入环境 /etc/default/bird-ble ..."
cat >/etc/default/bird-ble <<EOF
# Bird BLE 环境（由 install.sh 生成）
BIRD_USER=${INSTALL_USER}
BIRD_HOME=${INSTALL_HOME}
BIRD_BLE_UID=${INSTALL_UID}
BIRD_WS=${BIRD_WS}
SIM2REAL_WS=${SIM2REAL_WS}
EXTRA_ARGS=()
EOF

export BIRD_USER="${INSTALL_USER}"
export BIRD_HOME="${INSTALL_HOME}"
export BIRD_BLE_UID="${INSTALL_UID}"
export BIRD_WS="${BIRD_WS}"
export SIM2REAL_WS="${SIM2REAL_WS}"

chown "${INSTALL_USER}:${INSTALL_GID}" "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true
chmod 664 "${PKG_DIR}/ble_device_name.conf" 2>/dev/null || true

chmod +x "${APP_DIR}"/*.sh "${APP_DIR}/scripts/"*.sh "${APP_DIR}/systemd/"*.sh

echo "[4/5] 安装 systemd 开机自启..."
"${APP_DIR}/install-autostart.sh"

echo "[5/5] 检查 ROS / sim2real 环境..."
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/install"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  echo "[ok] sim2real: ${SIM2REAL_WS}/devel"
else
  echo "[warn] 未找到 sim2real 工作空间（${SIM2REAL_WS}）"
  echo "       模式/动作/摇杆指令需要 sim2real_msg，请先编译 sim2real"
fi

set +u
if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/install/setup.bash"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SIM2REAL_WS}/devel/setup.bash"
fi
set -u

if python3 -c "import sim2real_msg" 2>/dev/null; then
  echo "[ok] sim2real_msg 可导入"
else
  echo "[warn] 无法 import sim2real_msg — 检查 SIM2REAL_WS 路径"
fi

systemctl restart bird-ble.service || systemctl start bird-ble.service || true
sleep 5

if rostopic list >/dev/null 2>&1; then
  echo "[ok] roscore 已运行，bird-ble 已重启并连接 ROS"
else
  echo "[tip] roscore 尚未运行。请先启动 sim2real_master，再执行:"
  echo "      sudo systemctl restart bird-ble"
fi

echo ""
echo "=========================================="
echo " 安装完成"
echo "=========================================="
echo " 状态: sudo systemctl status bird-ble"
echo " 日志: journalctl -u bird-ble -f"
echo " 手动: cd ${APP_DIR} && ./start.sh"
echo " 诊断: ${APP_DIR}/scripts/check.sh"
echo ""
echo " 指令无效时: 确认日志含「ROS 控制桥接已启动」"
echo "            sudo systemctl restart bird-ble"
echo ""
"${APP_DIR}/scripts/check.sh" || true
INSTALL_EOF

cat >"${OUT_DIR}/uninstall.sh" <<'UNINSTALL_EOF'
#!/bin/bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行: sudo ./uninstall.sh"
  exit 1
fi
systemctl stop bird-ble.service 2>/dev/null || true
systemctl disable bird-ble.service 2>/dev/null || true
rm -f /etc/systemd/system/bird-ble.service
systemctl daemon-reload
echo "[ok] 已移除 bird-ble 开机自启（安装目录未删除）"
UNINSTALL_EOF

chmod +x "${OUT_DIR}/install.sh" "${OUT_DIR}/uninstall.sh"

cat >"${OUT_DIR}/README.md" <<'README_EOF'
# Bird BLE 遥控安装包 (BT_Control_0710)

微信小程序 BLE 从机，适配 **Jetson Orin（USB 蓝牙）** 与 **RK3588s（板载 RTL8822CE）**。

本包为**发布运行时**（Python 字节码，不含 .py 源码）。

## 一键安装

```bash
cd BT_Control_0710
sudo ./install.sh
```

安装内容：
- 系统依赖（bluez、python3-dbus、python3-gi）
- BlueZ `Experimental=true`
- systemd 服务 `bird-ble` 开机自启

## 前置条件

- Ubuntu 20.04 + **ROS Noetic**
- 已编译 `sim2real` 工作空间（`~/sim2real/install` 或 `~/sim2real/devel`）
- 机器人主控已启动 `roscore` / `sim2real_master`（**必须先于或同步于 BLE 服务**）

> BLE 能连接但指令无效？通常是 roscore 未就绪。先启动主控，再执行  
> `sudo systemctl restart bird-ble`，日志应出现 `ROS 控制桥接已启动`。

可选功能（需目标机 `Bird_ws` 内额外组件）：
- `locate_face_cpp` — `locate_face ON/OFF`
- `sound_demo` — 语音 `sound ON/OFF`

## 目录结构

```
BT_Control_0710/
  install.sh / uninstall.sh   # 一键安装
  README.md / VERSION
  ble_device_name.conf        # 蓝牙广播名
  app/                        # 运行时（字节码 + 脚本）
  docs/                       # 协议与小程序参考
```

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
./app/scripts/check.sh
sudo ./app/scripts/recover.sh
cd app && ./start.sh          # 前台调试
sudo ./uninstall.sh           # 移除自启
```

## 小程序参数

- 广播名：见 `ble_device_name.conf`（默认 `HT_88888888`）
- 服务 FFE0 / 写入 FFE1 / 通知 FFE2

协议详见 `docs/BLE_PROTOCOL.md`。

## 平台

自动识别 Orin / RK3588s，无需改配置。手动覆盖：

```bash
export BLE_PLATFORM=orin      # 或 rk3588s
```

## 环境变量（/etc/default/bird-ble）

安装后可在该文件修改 `SIM2REAL_WS`、`BIRD_USER`、`EXTRA_ARGS`，然后：

```bash
sudo systemctl daemon-reload
sudo systemctl restart bird-ble
```
README_EOF

echo "${VERSION}" >"${OUT_DIR}/VERSION"

# 打 tar 包
cd "$(dirname "${OUT_DIR}")"
tar czf "BT_Control_${VERSION}.tar.gz" "$(basename "${OUT_DIR}")"
echo "[build] 完成: ${OUT_DIR}"
echo "[build] 压缩包: $(dirname "${OUT_DIR}")/BT_Control_${VERSION}.tar.gz"
ls -la "${OUT_DIR}"
echo "[build] app 内容:"
ls -la "${APP_DIR}" | head -20
