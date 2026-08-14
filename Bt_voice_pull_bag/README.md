# Bt_voice_pull_bag（源码树）

蓝牙遥控 + 语音提醒 + 拖拽控制 + 头追 **四合一**源码目录。

## 版本

当前版本见 `VERSION`：

```
Bt-source-1.0.0-260730
```

命名约定：

| 版本名 | 含义 |
|--------|------|
| `Bt-source-1.0.0-260730` | 源码版（本目录，可含 `.py`） |
| `Bt-build-1.0.0-260730` | 编译版（仅 `.pyc` + 二进制，隐藏源码） |

后缀 `260730` 表示功能基线日期（2026-07-30）。

## 一键安装

```bash
cd ~/Bird_ws/Bt_voice_pull_bag
sudo ./install.sh
```

## 功能默认

| 功能 | 默认 | 开启 |
|------|------|------|
| 语音提示 `sound` | 开 | `sound OFF` 可关 |
| 人脸追踪 | 关 | `locate_face ON` |
| 拖拽 | 关 | `PULL ON` |
| 蓝牙 GATT | 开机自启 | `sudo systemctl status bird-ble` |

## 平台

- **RK3588**：板载蓝牙 + 头追优先 D435i `/dev/video4`
- **Jetson Orin**：USB 蓝牙 + 头追优先 ZED Mini `/dev/video0`（并排裁左眼）

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
cd ble/app && ./scripts/check.sh
```
