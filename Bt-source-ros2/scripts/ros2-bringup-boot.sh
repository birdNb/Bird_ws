#!/bin/bash
# ROS2 量产栈开机启动（严格按用户指定命令）
# 关键：必须等 wlan0 具备 IPv4 且 CycloneDDS 能建域，否则节点会全部秒崩：
#   "wlan0: does not match an available interface supporting udp"
# 另外：子节点全死后 launch 父进程仍会挂着，systemd 不会 Restart —— 此处用看门狗兜底。
set -eo pipefail

WS="/home/nvidia/hightorque_workspace"
LOG_DIR="${WS}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ros2-bringup.service.log"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export HOME="/home/nvidia"
export BIRD_HOME="${BIRD_HOME:-/home/nvidia}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RCUTILS_COLORIZED_OUTPUT=0
unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP

log() { echo "[ros2-bringup] $*" | tee -a "${LOG_FILE}"; }

DDS_IFACE="wlan0"
if [ -f "${HOME}/cyclonedds.xml" ]; then
  _if="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${HOME}/cyclonedds.xml" 2>/dev/null | head -1 || true)"
  DDS_IFACE="${_if:-wlan0}"
fi

# 1) 等网卡 UP + IPv4（仅 link UP 不够，DDS 会拒绝）
log "等待 ${DDS_IFACE} IPv4..."
_local_ip=""
for _ in $(seq 1 120); do
  if ip link show "${DDS_IFACE}" 2>/dev/null | grep -q "UP"; then
    _local_ip="$(ip -4 -o addr show "${DDS_IFACE}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1 || true)"
    if [ -n "${_local_ip}" ]; then
      log "${DDS_IFACE} 已有 IPv4: ${_local_ip}"
      break
    fi
  fi
  sleep 0.5
done
if [ -z "${_local_ip}" ]; then
  log "警告: ${DDS_IFACE} 长时间无 IPv4，仍尝试启动（可能失败）"
fi
# DHCP/驱动再留一点稳定时间
sleep 3

# 2) 生成运行时 DDS（本机 IP 写入 Peers）
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/prepare-cyclonedds-runtime.sh"
log "使用 CYCLONEDDS_URI=${CYCLONEDDS_URI}"

# 3) IMU（root oneshot 应已跑；此处再补一次）
if [ -x "${SCRIPT_DIR}/ensure_imu_serial.sh" ]; then
  "${SCRIPT_DIR}/ensure_imu_serial.sh" >>"${LOG_FILE}" 2>&1 || true
fi

cd "${WS}"
set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

# 4) 探测 CycloneDDS 能否真正建域（与量产节点同一环境）
log "探测 CycloneDDS / rclpy ..."
_dds_ok=0
for i in $(seq 1 40); do
  if CYCLONEDDS_URI="${CYCLONEDDS_URI}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" python3 - <<'PY' >>"${LOG_FILE}" 2>&1
import os, sys
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
try:
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    n = Node("bird_dds_boot_probe")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0)
except Exception as e:
    print("dds_probe_fail:", e)
    try:
        import rclpy
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(1)
PY
  then
    _dds_ok=1
    log "DDS 探测成功 (attempt ${i})"
    break
  fi
  log "DDS 未就绪，重试 ${i}/40 ..."
  sleep 1
done
if [ "${_dds_ok}" != "1" ]; then
  log "错误: DDS 探测失败，放弃启动（避免空 launch 占坑）"
  exit 1
fi

log "$(date -Iseconds) start bfm_real"
# 不用 exec：父脚本需看门狗；子节点全死后必须退出以触发 systemd Restart
set +e
ros2 launch hightorque_bringup bfm_real.launch.py \
  auto_stand:=true \
  auto_start_bfm:=false \
  enable_gamepad_commands:=true \
  enable_fall_detector:=false \
  enable_auto_fall_recovery:=false &
_launch_pid=$!
set -e

_controller_ok=0
for i in $(seq 1 90); do
  if ! kill -0 "${_launch_pid}" 2>/dev/null; then
    wait "${_launch_pid}" || true
    log "错误: launch 提前退出 (pid=${_launch_pid})"
    exit 1
  fi
  if pgrep -f 'hightorque_controller_node' >/dev/null 2>&1 \
    && pgrep -f 'hightorque_midware_node' >/dev/null 2>&1; then
    _controller_ok=1
    log "控制器/中间件已起来 (wait ${i}s)"
    break
  fi
  sleep 1
done

if [ "${_controller_ok}" != "1" ]; then
  log "错误: 90s 内未见 controller/midware，杀掉空 launch 以便 systemd 重试"
  kill "${_launch_pid}" 2>/dev/null || true
  sleep 2
  kill -9 "${_launch_pid}" 2>/dev/null || true
  pkill -f 'ros2 service call /hightorque_controller/change_state' 2>/dev/null || true
  exit 1
fi

wait "${_launch_pid}"
_rc=$?
log "launch 退出 code=${_rc}"
exit "${_rc}"
