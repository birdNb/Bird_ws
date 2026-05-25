# hand_identify — ZED Mini 手势与 3D 跟踪

## 功能

- MediaPipe Hands 检测单手 21 关键点
- 手势数字识别 **0~5**（伸直手指数）
- ZED 深度/点云计算手掌中心 **3D 坐标**（米）
- 帧间位移判断移动方向：**前/后/左/右/上/下**
- OpenCV 画面叠加 + 终端彩色日志

## 依赖

1. 安装 [ZED SDK](https://www.stereolabs.com/developers/release/)（建议 4.2+）
2. Python 包：

```bash
pip install -r requirements.txt
# ZED SDK 安装后:
pip install pyzed
```

## 运行

```bash
cd hand_identify
python3 zed_gesture_recognition.py
```

无界面（仅终端日志）：

```bash
python3 zed_gesture_recognition.py --no-gui
```

按 **q** 退出。

## 参数

| 参数 | 说明 |
|------|------|
| `--no-gui` | 不弹窗 |
| `--move-threshold 0.03` | 移动判定阈值(米)，默认 0.02 |

## 调试

- 深度不准：在脚本里把 `DEPTH_MODE.QUALITY` 改为 `ULTRA`
- 手势不稳：提高 `min_detection_confidence`（默认 0.7）
- 方向太敏感：增大 `--move-threshold`
- 检测距离：修改 `depth_maximum_distance`（默认 3.0m）
