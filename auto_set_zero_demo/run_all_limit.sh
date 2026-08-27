#!/bin/bash
# 限位扫描一键串行：右手 → 头+腰 → 右腿
# 各组结果分别写入 standing_pose.yaml 对应段落。
#
# 用法:
#   ./run_all_limit.sh                 # 默认带 --takeover（停控制器拿电机）
#   ./run_all_limit.sh --no-takeover
#   ./run_all_limit.sh --from waist_neck  # 从腰头开始（跳过右手）
#   ./run_all_limit.sh --only right_arm   # 只跑一组
#   ./run_all_limit.sh --dry-run
#
# 其余参数原样传给 cw_limit.py（如 --flip-waist / --flip-hip-roll）
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

title_of() {
  case "$1" in
    right_arm) echo "右手 4 轴" ;;
    waist_neck) echo "头 + 腰" ;;
    right_leg) echo "右腿" ;;
    *) echo "未知" ;;
  esac
}

STAGES=(right_arm waist_neck right_leg)
TAKEOVER=1
FROM=""
ONLY=""
PASSTHRU=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-takeover)
      TAKEOVER=0
      shift
      ;;
    --from)
      FROM="${2:-}"
      if [[ -z "$FROM" ]]; then
        echo "用法: --from right_arm|waist_neck|right_leg" >&2
        exit 2
      fi
      shift 2
      ;;
    --only)
      ONLY="${2:-}"
      if [[ -z "$ONLY" ]]; then
        echo "用法: --only right_arm|waist_neck|right_leg" >&2
        exit 2
      fi
      shift 2
      ;;
    *)
      PASSTHRU+=("$1")
      shift
      ;;
  esac
done

if [[ -n "$ONLY" ]]; then
  case "$ONLY" in
    right_arm|waist_neck|right_leg) ;;
    *)
      echo "未知 --only $ONLY" >&2
      exit 2
      ;;
  esac
  STAGES=("$ONLY")
elif [[ -n "$FROM" ]]; then
  found=0
  filtered=()
  for g in "${STAGES[@]}"; do
    if [[ "$g" == "$FROM" ]]; then
      found=1
    fi
    if [[ $found -eq 1 ]]; then
      filtered+=("$g")
    fi
  done
  if [[ $found -eq 0 ]]; then
    echo "未知 --from $FROM，可选: right_arm | waist_neck | right_leg" >&2
    exit 2
  fi
  STAGES=("${filtered[@]}")
fi

echo "=========================================="
echo "  限位串行: ${STAGES[*]}"
echo "=========================================="

first=1
for g in "${STAGES[@]}"; do
  echo
  echo ">>>>>>>> [${g}] $(title_of "$g") <<<<<<<<"
  args=()
  if [[ ${#PASSTHRU[@]} -gt 0 ]]; then
    args+=("${PASSTHRU[@]}")
  fi
  if [[ $TAKEOVER -eq 1 && $first -eq 1 ]]; then
    args+=(--takeover)
  fi
  first=0
  python3 "${ROOT}/cw_limit.py" --group "$g" "${args[@]}"
  echo "<<<<<<<< [${g}] 完成 >>>>>>>>"
  sleep 1
done

echo
echo "全部完成。结果见: ${ROOT}/standing_pose.yaml"
echo "  right_arm_limit_q / waist_neck_limit_q / right_leg_limit_q"
