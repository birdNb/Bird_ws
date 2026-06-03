#!/bin/bash
# 机器人控制系统环境部署
# 支持 ARM64 Ubuntu
# 直接执行: ./setup_env.sh

set -e  # 遇到错误立即退出

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

CONDA_ENV_NAME="robot_control"
PYTHON_VERSION="3.10"

echo "=============================================="
echo "       机器人控制系统环境部署"
echo "=============================================="
echo ""

# 获取用户家目录
USER_HOME="$HOME"
echo "用户家目录: $USER_HOME"
echo ""

# 0. 检查当前系统Python版本
echo "[1/6] 检查当前Python环境..."
CURRENT_PYTHON_VERSION=""
USE_SYSTEM_PYTHON=false

if command -v python3 &> /dev/null; then
    CURRENT_PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
    echo "当前Python版本: $CURRENT_PYTHON_VERSION"

    if [ "$CURRENT_PYTHON_VERSION" = "$PYTHON_VERSION" ]; then
        echo "✓ 当前系统Python版本符合要求 (Python $PYTHON_VERSION)"
        echo "将使用系统Python，无需安装conda"
        USE_SYSTEM_PYTHON=true
    else
        echo "⚠️  当前Python版本不符合要求，需要Python $PYTHON_VERSION"
        echo "将安装conda并创建独立环境"
    fi
else
    echo "⚠️  未找到python3，将安装conda"
fi

echo ""

# 如果使用系统Python，跳过conda安装
if [ "$USE_SYSTEM_PYTHON" = true ]; then
    echo "[2/6] 跳过Miniconda安装（使用系统Python）"
    echo ""
    echo "[3/6] 跳过Conda环境创建（使用系统Python）"
    echo ""
    echo "[4/6] 跳过环境激活（使用系统Python）"
    echo ""
    echo "[5/6] 安装Python依赖..."
    pip3 install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple || {
        echo -e "\033[91m错误: 依赖安装失败\033[0m"
        exit 1
    }

    echo ""
    echo "=============================================="
    echo -e "\033[92m✓ 环境配置完成\033[0m"
    echo "=============================================="
    echo ""
    echo "环境信息:"
    echo "  使用系统Python: $(which python3)"
    echo "  Python版本: $(python3 --version)"
    echo "  pip版本: $(pip3 --version | cut -d' ' -f1-2)"
    echo ""
    echo ""

    # 跳过systemd服务安装询问，直接退出
    echo "=============================================="
    echo "快速启动命令:"
    echo "=============================================="
    echo "  机器人端: ./start_robot.sh"
    echo "  控制端:   ./start_controller.sh"
    echo ""
    exit 0
fi

# 1. 安装Miniconda (ARM64)
echo "[2/6] 检查Miniconda..."
CONDA_DIR="$USER_HOME/miniconda3"

if [ ! -d "$CONDA_DIR" ]; then
    echo "Miniconda未安装，开始安装..."

    # 检查系统架构
    ARCH=$(uname -m)
    if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
        echo "检测到 ARM64 架构"
        MINICONDA_INSTALLER="Miniconda3-latest-Linux-aarch64.sh"
    else
        echo "检测到 x86_64 架构"
        MINICONDA_INSTALLER="Miniconda3-latest-Linux-x86_64.sh"
    fi

    # 下载 Miniconda
    if [ ! -f "$MINICONDA_INSTALLER" ]; then
        echo "下载 Miniconda..."
        wget "https://repo.anaconda.com/miniconda/$MINICONDA_INSTALLER" -O "$MINICONDA_INSTALLER" || {
            echo -e "\033[91m错误: Miniconda下载失败\033[0m" >&2
            echo "请手动下载 $MINICONDA_INSTALLER 到当前目录"
            exit 1
        }
    fi

    bash "$MINICONDA_INSTALLER" -b -p "$CONDA_DIR"
    echo -e "\033[92m✓ Miniconda安装完成\033[0m"
else
    echo "✓ Miniconda已安装"
fi

# 2. 初始化Conda环境
echo ""
echo "[3/6] 初始化Conda环境..."
CONDA_SH="$CONDA_DIR/etc/profile.d/conda.sh"
if [ ! -f "$CONDA_SH" ]; then
    echo -e "\033[91m错误: 找不到conda初始化文件\033[0m" >&2
    exit 1
fi
source "$CONDA_SH"
echo "✓ Conda环境已加载"

# 3. 检查并创建Conda环境
echo ""
echo "[4/6] 检查Conda环境..."
if conda env list | grep -q "^${CONDA_ENV_NAME} "; then
    echo "✓ conda环境 '$CONDA_ENV_NAME' 已存在"
else
    echo "创建新的 '$CONDA_ENV_NAME' 环境 (Python $PYTHON_VERSION)..."

    # 配置conda
    conda config --set auto_activate_base false 2>/dev/null || true

    # 创建环境
    conda create -n "$CONDA_ENV_NAME" python=$PYTHON_VERSION -y || {
        echo -e "\033[91m错误: 创建环境失败\033[0m"
        exit 1
    }
    echo -e "\033[92m✓ conda环境创建完成\033[0m"
fi

# 4. 激活环境
echo ""
echo "[5/6] 激活环境..."
conda activate "$CONDA_ENV_NAME" || {
    echo -e "\033[91m错误: 激活环境失败\033[0m"
    exit 1
}
echo "✓ 环境已激活: $CONDA_ENV_NAME"

# 5. 安装基本Python依赖
echo ""
echo "[6/6] 安装Python依赖..."
pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple || {
    echo -e "\033[91m错误: 依赖安装失败\033[0m"
    exit 1
}

echo ""
echo "=============================================="
echo -e "\033[92m✓ 环境配置完成\033[0m"
echo "=============================================="
echo ""
echo "环境信息:"
echo "  conda环境: $CONDA_ENV_NAME"
echo "  Python版本: $(python --version)"
echo "  pip版本: $(pip --version | cut -d' ' -f1-2)"
echo ""
echo ""

# ============================================
# 询问是否安装systemd服务（开机自启动）
# ============================================
echo "是否安装systemd服务（开机自启动）? (y/N)"
read -r response

if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    echo ""
    echo "=============================================="
    echo "安装systemd服务..."
    echo "=============================================="
    echo ""

    bash install_service.sh || {
        echo -e "\033[91m✗ systemd服务安装失败\033[0m"
        echo "可稍后手动安装: ./install_service.sh"
    }
else
    echo ""
    echo "跳过systemd服务安装"
    echo "如需开机自启动，可稍后执行: ./install_service.sh"
    echo ""
fi

echo ""
echo "=============================================="
echo "快速启动命令:"
echo "=============================================="
echo "  机器人端: ./start_robot.sh"
echo "  控制端:   ./start_controller.sh"
echo ""