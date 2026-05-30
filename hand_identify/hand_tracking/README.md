# hand_tracking — 手部跟踪

基于 `hand_perception` 的 ZED 感知，手势 **5** 时根据掌心深度做前后距离保持，发布 `/cmd_vel.linear.x`。

## 一键启动

```bash
cd ~/Bird_ws/hand_identify/hand_tracking
chmod +x start.sh
./start.sh
```

或从工程根目录：

```bash
./start_hand_tracking.sh
```

默认 `--no-fsm`；需要 FSM 守门时去掉该参数或编辑 `start.sh`。

## 常用参数

| 参数 | 说明 |
|------|------|
| `--no-gui` | 无窗口 |
| `--no-fsm` | 不等待 FSM=EXEC_DEFAULT |
| `--dist-min 0.2` | 最近有效距离 |
| `--dist-max 2.0` | 最远有效距离 |

## 文件

| 文件 | 说明 |
|------|------|
| `start.sh` | 一键启动 → `distance_hold.py` |
| `hand_perception.py` | 感知库（可单独 `python3 hand_perception.py` 预览） |
| `distance_hold.py` | 手势 5 距离保持主程序 |
| `locomotion.py` | 全轴跟手备份（未接入 start.sh） |

公共模块见 `../common/`（`ros_control.py` 提供 FSM 监听）。
