#!/bin/bash
# 限位并行标定（推荐）:
#   1) 腰转 90°
#   2) 手 / 头 / 腿 三车道同时寻限位并各自恢复
#   3) 再标定腰（从当前位置继续转到限位）
#   4) 全身恢复启动姿态
#
# 用法:
#   ./run_all_limit.sh                 # 并行 + --takeover
#   ./run_all_limit.sh --serial        # 旧版串行: 臂→腰头→腿
#   ./run_all_limit.sh --no-takeover
#   ./run_all_limit.sh --dry-run
#
# 其余参数原样传给 cw_limit.py
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

set +u
source /opt/ros/foxy/setup.bash
source /home/nvidia/hightorque_workspace/install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

MODE=parallel
TAKEOVER=1
FROM=""
ONLY=""
PASSTHRU=()
HAS_RESTORE_VEL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      MODE=serial
      shift
      ;;
    --parallel)
      MODE=parallel
      shift
      ;;
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
      MODE=serial
      shift 2
      ;;
    --only)
      ONLY="${2:-}"
      if [[ -z "$ONLY" ]]; then
        echo "用法: --only right_arm|waist_neck|right_leg" >&2
        exit 2
      fi
      MODE=serial
      shift 2
      ;;
    --restore-vel)
      HAS_RESTORE_VEL=1
      PASSTHRU+=("$1" "$2")
      shift 2
      ;;
    *)
      PASSTHRU+=("$1")
      shift
      ;;
  esac
done

args=()
if [[ ${#PASSTHRU[@]} -gt 0 ]]; then
  args+=("${PASSTHRU[@]}")
fi
if [[ $HAS_RESTORE_VEL -eq 0 ]]; then
  args+=(--restore-vel 2.5 --restore-sec 1.0)
fi
if [[ $TAKEOVER -eq 1 ]]; then
  args+=(--takeover)
fi

if [[ "$MODE" == "parallel" ]]; then
  echo "=========================================="
  echo "  限位并行: 腰90° → 手/头/腿同时 → 腰 → 恢复"
  echo "=========================================="
  python3 "${ROOT}/cw_limit.py" --parallel "${args[@]}"
else
  STAGES=(right_arm waist_neck right_leg)
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
      echo "未知 --from $FROM" >&2
      exit 2
    fi
    STAGES=("${filtered[@]}")
  fi
  echo "=========================================="
  echo "  限位串行(同进程): ${STAGES[*]}"
  echo "=========================================="
  python3 "${ROOT}/cw_limit.py" --groups "${STAGES[@]}" "${args[@]}"
fi

echo
echo "全部完成。结果见: ${ROOT}/standing_pose.yaml"
echo "  right_arm_limit_q / waist_neck_limit_q / right_leg_limit_q"
