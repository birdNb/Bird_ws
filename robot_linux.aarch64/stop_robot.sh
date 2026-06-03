#!/bin/bash
# 停止机器人端

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

PID_FILE="robot.pid"

if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if ps -p $PID > /dev/null 2>&1; then
        echo "停止机器人端 (PID: $PID)..."
        kill $PID
        sleep 2
        if ps -p $PID > /dev/null 2>&1; then
            echo "强制停止..."
            kill -9 $PID
        fi
        rm -f "$PID_FILE"
        echo "✓ 机器人端已停止"
    else
        echo "进程不存在 (PID: $PID)"
        rm -f "$PID_FILE"
    fi
else
    echo "未找到PID文件，尝试查找进程..."
    PIDS=$(pgrep -f "python3 robot_main.py")
    if [ -n "$PIDS" ]; then
        echo "找到进程: $PIDS"
        kill $PIDS
        echo "✓ 机器人端已停止"
    else
        echo "未找到运行中的机器人端进程"
    fi
fi
