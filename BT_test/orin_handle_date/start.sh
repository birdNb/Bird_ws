#!/bin/bash
# 2.4G 无线手柄接收器指令监听（ROS /joy）
set -euo pipefail
cd "$(dirname "$0")"

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项...]

监听 2.4G USB 接收器经 joy_node 发布的指令，终端打印摇杆/按键变化。

硬件: 接收器插入后一般为 /dev/input/js0
ROS:  sim2real 启动后话题为 /joy 或 /joy_input

选项（传给 orin_handle_date.py）:
  --joy-only        只监听 /joy、/joy_input
  --topic /joy      指定主话题
  --watch-cmd-vel   同时打印 /cmd_vel
  --watch-joy-msg   同时打印 /joy_msg
  --rate-hz 0       关闭空闲心跳
  --no-log          不写日志文件
  -h, --help        帮助

日志: 默认写入本目录 orin_handle_YYYYMMDD_HHMMSS.log（jsonl 原始数据）

示例:
  ./start.sh
  ./start.sh --joy-only
  ./start.sh --watch-cmd-vel --watch-joy-msg

前置: roscore + joy_node 或 sim2real_master 已运行
EOF
}

for arg in "$@"; do
  case "$arg" in
    -h|--help) show_help; exit 0 ;;
  esac
done

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${HOME}/sim2real/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/devel/setup.bash"
elif [ -f "${HOME}/sim2real/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/install/setup.bash"
fi

chmod +x orin_handle_date.py

if ! ls /dev/input/js* >/dev/null 2>&1; then
  echo "[warn] 未检测到 /dev/input/js*，请插入 2.4G 接收器"
fi

if ! rostopic list 2>/dev/null | grep -qE '^/joy$|^/joy_input$'; then
  echo "[warn] 当前无 /joy 或 /joy_input 话题"
  echo "       请先启动 sim2real 或: rosrun joy joy_node"
  echo "       本脚本仍会等待订阅..."
fi

LOG_FILE="$(pwd)/orin_handle_$(date +%Y%m%d_%H%M%S).log"

echo "========================================"
echo " 2.4G 手柄监听 orin_handle_date"
echo " 话题: /joy  /joy_input  (+/cmd_vel /joy_msg)"
echo " 日志: ${LOG_FILE}"
echo "========================================"

exec python3 orin_handle_date.py --log-file "${LOG_FILE}" "$@"
