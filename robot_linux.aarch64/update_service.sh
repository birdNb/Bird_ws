#!/bin/bash
# 更新systemd服务配置（修改配置文件后使用）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

SERVICE_NAME="robot-control"
SERVICE_FILE="${SERVICE_NAME}.service"
SYSTEMD_DIR="$HOME/.config/systemd/user"
TARGET_SERVICE="$SYSTEMD_DIR/$SERVICE_FILE"

echo "========================================="
echo "  更新机器人控制系统服务配置"
echo "========================================="
echo ""

if [ ! -f "$TARGET_SERVICE" ]; then
    echo "✗ 服务未安装，请先执行: ./install_service.sh"
    exit 1
fi

echo "[1/4] 检查服务状态..."
SERVICE_RUNNING=false
if systemctl --user is-active --quiet "$SERVICE_NAME"; then
    SERVICE_RUNNING=true
    echo "✓ 服务正在运行"
else
    echo "○ 服务未运行"
fi

echo ""

# 停止服务
if [ "$SERVICE_RUNNING" = true ]; then
    echo "[2/4] 停止服务..."
    systemctl --user stop "$SERVICE_NAME"
    echo "✓ 服务已停止"
else
    echo "[2/4] 跳过停止（服务未运行）"
fi

echo ""

# 重新生成配置
echo "[3/4] 重新生成服务配置..."

# 检查Python
PYTHON_CMD=$(which python3)
if [ -z "$PYTHON_CMD" ]; then
    echo "✗ 错误: 未找到python3"
    exit 1
fi

# 检查conda环境
CONDA_ENV_NAME="robot_control"
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
        CONDA_ENV_PATH=$(conda env list | grep "^${CONDA_ENV_NAME} " | awk '{print $NF}')
        CONDA_PYTHON="$CONDA_ENV_PATH/bin/python3"
        if [ -f "$CONDA_PYTHON" ]; then
            PYTHON_CMD="$CONDA_PYTHON"
        fi
    fi
fi

# 检查ROS
ROS_PYTHON_PATH=""
for ros_setup in "/opt/ros/noetic/setup.bash" \
                 "/opt/ros/melodic/setup.bash" \
                 "/opt/ros/humble/setup.bash" \
                 "/opt/ros/foxy/setup.bash"; do
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
        ROS_PYTHON_PATH="/opt/ros/$ROS_DISTRO/lib/python3/dist-packages"
        break
    fi
done

# 读取模板并替换变量
SERVICE_CONTENT=$(cat "$SERVICE_FILE")
SERVICE_CONTENT="${SERVICE_CONTENT//\%ROBOT_DIR%/$SCRIPT_DIR}"
SERVICE_CONTENT="${SERVICE_CONTENT//\%ROS_PYTHON_PATH%/$ROS_PYTHON_PATH}"
SERVICE_CONTENT="${SERVICE_CONTENT//\%PYTHON_CMD%/$PYTHON_CMD}"

# 写入服务文件
echo "$SERVICE_CONTENT" > "$TARGET_SERVICE"
echo "✓ 服务配置已更新"

# 重载systemd
systemctl --user daemon-reload
echo "✓ systemd配置已重载"

echo ""

# 重启服务
if [ "$SERVICE_RUNNING" = true ]; then
    echo "[4/4] 重启服务..."
    systemctl --user start "$SERVICE_NAME"
    sleep 2
    if systemctl --user is-active --quiet "$SERVICE_NAME"; then
        echo "✓ 服务已重启"
    else
        echo "✗ 服务启动失败"
        systemctl --user status "$SERVICE_NAME" --no-pager
        exit 1
    fi
else
    echo "[4/4] 启动服务..."
    systemctl --user start "$SERVICE_NAME"
    sleep 2
    if systemctl --user is-active --quiet "$SERVICE_NAME"; then
        echo "✓ 服务已启动"
    else
        echo "✗ 服务启动失败"
        systemctl --user status "$SERVICE_NAME" --no-pager
        exit 1
    fi
fi

echo ""
echo "========================================="
echo "✓ 更新完成"
echo "========================================="
echo ""
echo "查看服务状态: systemctl --user status $SERVICE_NAME"
echo "查看实时日志: journalctl --user -u $SERVICE_NAME -f"
echo ""
