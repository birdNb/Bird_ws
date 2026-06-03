#!/bin/bash
# 安装systemd服务，实现开机自启动

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

SERVICE_NAME="robot-control"
SERVICE_FILE="${SERVICE_NAME}.service"
SYSTEMD_DIR="$HOME/.config/systemd/user"

echo "========================================="
echo "  安装机器人控制系统 - Systemd服务"
echo "========================================="
echo ""

# 检查Python环境
echo "[1/5] 检查Python环境..."

PYTHON_CMD=$(which python3)
if [ -z "$PYTHON_CMD" ]; then
    echo "✗ 错误: 未找到python3"
    echo "请先安装Python 3: sudo apt install python3"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
echo "✓ Python版本: $PYTHON_VERSION"
echo "✓ Python路径: $PYTHON_CMD"

# 检查conda环境
CONDA_ENV_NAME="robot_control"
CONDA_PYTHON=""
USE_CONDA=false

# 尝试查找conda
check_conda() {
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

    if command -v conda &> /dev/null; then
        return 0
    fi

    return 1
}

if check_conda; then
    if conda env list | grep -q "^${CONDA_ENV_NAME} "; then
        echo "✓ 找到conda环境: $CONDA_ENV_NAME"
        # 获取conda环境的Python路径
        CONDA_ENV_PATH=$(conda env list | grep "^${CONDA_ENV_NAME} " | awk '{print $NF}')
        CONDA_PYTHON="$CONDA_ENV_PATH/bin/python3"
        if [ -f "$CONDA_PYTHON" ]; then
            echo "✓ 将使用conda环境的Python: $CONDA_PYTHON"
            PYTHON_CMD="$CONDA_PYTHON"
            USE_CONDA=true
        fi
    fi
fi

echo ""

# 检查ROS环境
echo "[2/5] 检查ROS环境..."
ROS_SETUP_PATHS=(
    "/opt/ros/noetic/setup.bash"
    "/opt/ros/melodic/setup.bash"
    "/opt/ros/humble/setup.bash"
    "/opt/ros/foxy/setup.bash"
)

ROS_DISTRO=""
ROS_PYTHON_PATH=""
for ros_setup in "${ROS_SETUP_PATHS[@]}"; do
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
        ROS_DISTRO="$ROS_DISTRO"
        ROS_PYTHON_PATH="/opt/ros/$ROS_DISTRO/lib/python3/dist-packages"
        echo "✓ 找到ROS: $ROS_DISTRO"
        break
    fi
done

if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  警告: 未找到ROS环境"
    echo "   系统将在模拟模式下运行"
    ROS_PYTHON_PATH=""
fi

echo ""

# 创建systemd用户目录
echo "[3/5] 准备systemd目录..."
mkdir -p "$SYSTEMD_DIR"
echo "✓ systemd目录: $SYSTEMD_DIR"

echo ""

# 生成服务文件
echo "[4/5] 生成服务配置..."

# 读取模板
SERVICE_CONTENT=$(cat "$SERVICE_FILE")

# 替换变量
SERVICE_CONTENT="${SERVICE_CONTENT//\%ROBOT_DIR%/$SCRIPT_DIR}"
SERVICE_CONTENT="${SERVICE_CONTENT//\%ROS_PYTHON_PATH%/$ROS_PYTHON_PATH}"
SERVICE_CONTENT="${SERVICE_CONTENT//\%PYTHON_CMD%/$PYTHON_CMD}"

# 写入服务文件
TARGET_SERVICE="$SYSTEMD_DIR/$SERVICE_FILE"
echo "$SERVICE_CONTENT" > "$TARGET_SERVICE"
echo "✓ 服务文件已生成: $TARGET_SERVICE"

echo ""

# 重载systemd
echo "[5/5] 重载systemd配置..."
systemctl --user daemon-reload
echo "✓ systemd配置已重载"

echo ""

# 启用服务
systemctl --user enable "$SERVICE_NAME"
echo "✓ 服务已启用（开机自启动）"

echo ""
echo "========================================="
echo "✓ 安装完成"
echo "========================================="
echo ""

# 检查并启用用户lingering
echo "检查用户lingering状态..."
if ! loginctl show-user "$USER" | grep -q "Linger=yes"; then
    echo "⚠️  用户lingering未启用，需要启用以确保开机自启动"
    echo "执行: sudo loginctl enable-linger $USER"
    echo ""
    echo "是否现在启用? (需要sudo权限) (Y/n)"
    read -r linger_response
    if [[ ! "$linger_response" =~ ^([nN][oO]|[nN])$ ]]; then
        sudo loginctl enable-linger "$USER" && echo "✓ 用户lingering已启用"
    else
        echo "跳过，请稍后手动执行: sudo loginctl enable-linger $USER"
    fi
else
    echo "✓ 用户lingering已启用"
fi

echo ""
echo "服务管理命令:"
echo "  启动服务: systemctl --user start $SERVICE_NAME"
echo "  停止服务: systemctl --user stop $SERVICE_NAME"
echo "  重启服务: systemctl --user restart $SERVICE_NAME"
echo "  查看状态: systemctl --user status $SERVICE_NAME"
echo "  查看日志: journalctl --user -u $SERVICE_NAME -f"
echo "  禁用自启: systemctl --user disable $SERVICE_NAME"
echo ""
echo "服务日志文件: $SCRIPT_DIR/robot.log"
echo ""
echo "是否现在启动服务? (Y/n)"
read -r response
if [[ "$response" =~ ^([nN][oO]|[nN])$ ]]; then
    echo "跳过启动，可稍后手动启动: systemctl --user start $SERVICE_NAME"
else
    echo ""
    echo "启动服务..."
    systemctl --user start "$SERVICE_NAME"
    sleep 2
    systemctl --user status "$SERVICE_NAME" --no-pager
fi

echo ""
