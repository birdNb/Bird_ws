#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

show_help() {
  cat <<'EOF'
用法: ./start.sh [选项...]

  默认后台运行（无 OpenCV 窗口）
  --gui       显示全屏预览
  --no-gui    显式关闭预览（默认）
  --no-fsm    跳过 FSM 守门（谨慎）
  -h, --help  显示帮助

示例:
  ./start.sh              # 后台头追
  ./start.sh --gui        # 带全屏预览
  ./start.sh --no-fsm     # 调试：不等 FSM=5
EOF
}

LF_ARGS=()
HAS_GUI=0
for arg in "$@"; do
  case "$arg" in
    -h|--help)
      show_help
      exit 0
      ;;
    --gui|--no-gui)
      HAS_GUI=1
      LF_ARGS+=("$arg")
      ;;
    *)
      LF_ARGS+=("$arg")
      ;;
  esac
done

if [ "$HAS_GUI" -eq 0 ]; then
  LF_ARGS+=(--no-gui)
fi

BIN="./build/locate_face"
if [ ! -x "${BIN}" ]; then
  echo "[build] 未找到 ${BIN}，正在编译..."
  echo "[error] 发布包无源码，请使用预编译 build/locate_face"
fi

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

export LOCATE_FACE_CPP_ROOT="$(pwd)"
export DISPLAY="${DISPLAY:-:0}"
[ -f "${HOME}/.Xauthority" ] && export XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}"

pkill -f 'locate_face_cpp/build/locate_face' 2>/dev/null || true
pkill -f 'locate_face\.py' 2>/dev/null || true
sleep 0.2

echo "Starting: ${BIN} ${LF_ARGS[*]}"
exec "${BIN}" "${LF_ARGS[@]}"
