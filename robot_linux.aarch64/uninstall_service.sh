#!/bin/bash
# 卸载systemd服务

SERVICE_NAME="robot-control"
SYSTEMD_DIR="$HOME/.config/systemd/user"
SERVICE_FILE="$SYSTEMD_DIR/${SERVICE_NAME}.service"

echo "========================================="
echo "  卸载机器人控制系统 - Systemd服务"
echo "========================================="
echo ""

if [ ! -f "$SERVICE_FILE" ]; then
    echo "✗ 服务未安装"
    exit 0
fi

echo "正在卸载服务..."
echo ""

# 停止服务
echo "[1/4] 停止服务..."
systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
echo "✓ 服务已停止"

echo ""

# 禁用服务
echo "[2/4] 禁用自启动..."
systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true
echo "✓ 自启动已禁用"

echo ""

# 删除服务文件
echo "[3/4] 删除服务文件..."
rm -f "$SERVICE_FILE"
echo "✓ 服务文件已删除"

echo ""

# 重载systemd
echo "[4/4] 重载systemd配置..."
systemctl --user daemon-reload
echo "✓ systemd配置已重载"

echo ""
echo "========================================="
echo "✓ 卸载完成"
echo "========================================="
echo ""
