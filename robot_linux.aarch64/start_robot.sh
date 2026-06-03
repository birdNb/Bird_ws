#!/bin/bash
# 机器人端启动脚本（支持conda环境）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
cd "$PROJECT_ROOT"

CONDA_ENV_NAME="robot_control"
DAEMON_MODE=false

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--daemon)
            DAEMON_MODE=true
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [-d|--daemon]"
            exit 1
            ;;
    esac
done

echo "=============================================="
echo "       机器人控制系统 - 启动"
echo "=============================================="
echo ""

echo "[1/3] 检查Python环境..."

# 检查当前Python版本
CURRENT_PYTHON_VERSION=""
if command -v python3 &> /dev/null; then
    CURRENT_PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
    echo "  当前Python版本: $CURRENT_PYTHON_VERSION"
fi

# 如果是Python 3.10，直接使用系统Python
if [ "$CURRENT_PYTHON_VERSION" = "3.10" ]; then
    echo "  ✓ Python版本符合要求，使用系统Python"
    USE_CONDA=false
else
    echo "  ⚠️  需要Python 3.10环境"
    USE_CONDA=true
fi

echo ""

# 如果需要conda环境
if [ "$USE_CONDA" = true ]; then
    echo "[2/3] 检查conda环境..."

    # 检查conda是否可用
    check_conda() {
        # 尝试多个可能的conda路径
        local conda_paths=(
            "$HOME/miniconda3/etc/profile.d/conda.sh"
            "$HOME/anaconda3/etc/profile.d/conda.sh"
            "/opt/conda/etc/profile.d/conda.sh"
        )

        for conda_sh in "${conda_paths[@]}"; do
            if [ -f "$conda_sh" ]; then
                source "$conda_sh"
                return 0
            fi
        done

        # 检查conda命令是否直接可用
        if command -v conda &> /dev/null; then
            return 0
        fi

        return 1
    }

    # 检查conda环境是否存在
    check_conda_env() {
        if conda env list | grep -q "^${CONDA_ENV_NAME} "; then
            return 0
        else
            return 1
        fi
    }

    if check_conda; then
        echo "  ✓ conda已安装"

        # 检查robot环境是否存在
        if check_conda_env; then
            echo "  ✓ conda环境 '$CONDA_ENV_NAME' 已存在"
            conda activate "$CONDA_ENV_NAME" || {
                echo "  ✗ 错误: 无法激活conda环境"
                exit 1
            }
            echo "  ✓ 环境已激活"
        else
            echo "  ⚠️  conda环境 '$CONDA_ENV_NAME' 不存在"
            echo "  需要先配置环境，是否现在执行? (y/N)"
            read -r response
            if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
                echo ""
                bash setup_env.sh || {
                    echo "  ✗ 环境配置失败，请手动执行: ./setup_env.sh"
                    exit 1
                }
                # 重新加载conda并激活环境
                check_conda
                conda activate "$CONDA_ENV_NAME"
            else
                echo ""
                echo "  请先执行环境配置: ./setup_env.sh"
                exit 1
            fi
        fi
    else
        echo "  ⚠️  未检测到conda，需要先配置环境"
        echo "  是否现在执行环境配置? (y/N)"
        read -r response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
            echo ""
            bash setup_env.sh || {
                echo "  ✗ 环境配置失败，请手动执行: ./setup_env.sh"
                exit 1
            }
            # 重新加载conda并激活环境
            check_conda
            conda activate "$CONDA_ENV_NAME"
        else
            echo ""
            echo "  请先执行环境配置: ./setup_env.sh"
            exit 1
        fi
    fi

    echo ""
else
    echo "[2/3] 跳过conda环境检查（Python版本已满足）"
    echo ""
fi

# 启动机器人端
echo "[3/3] 启动机器人端..."
echo ""

if [ "$DAEMON_MODE" = true ]; then
    echo "后台模式启动..."
    nohup python3 robot/robot_main.py > robot.log 2>&1 &
    PID=$!
    echo $PID > robot.pid
    echo "✓ 机器人端已在后台启动 (PID: $PID)"
    echo "  日志文件: robot.log"
    echo "  停止服务: ./robot/stop_robot.sh 或 kill $PID"
else
    echo "前台模式启动..."
    python3 robot/robot_main.py
fi
