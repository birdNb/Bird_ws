# gesture_recognition — 手势识别与动作

ZED + MediaPipe 识别手势 **0~5**、手掌 3D 位置与移动方向；手势 **1~4** 稳定 2s 后触发机器人动作。

## 一键启动

```bash
cd ~/Bird_ws/hand_identify/gesture_recognition
chmod +x start.sh
./start.sh
```

或从工程根目录：

```bash
./start_gesture_recognition.sh
```

## 常用参数

| 参数 | 说明 |
|------|------|
| `--preview` | 仅识别与日志，不初始化 ROS |
| `--no-gui` | 无窗口 |
| `--no-coquette` | 禁用手势 1 撒娇扭腰 |
| `--no-actions` | 禁用全部动作 |
| `--no-fsm` | 不等待 FSM=5 |
| `--gesture-hold-sec 2` | 触发前稳定时长 |

终端 **Ctrl+C** 可强制退出（`start.sh` 会转发信号；连按两次可 SIGKILL）。

## 文件

| 文件 | 说明 |
|------|------|
| `start.sh` | 一键启动 |
| `zed_gesture_recognition.py` | 主程序 |
| `gesture_motion.py` | ROS 动作调度 |
| `motion/hand_action_library.py` | 手势 2~4 → `/joy_msg` |
| `motion/waist_coquette_*.py` | 手势 1 撒娇扭腰 |

公共模块见 `../common/`。
