# Bt_voice_pull_bag_0721

蓝牙遥控 + 语音提醒 + 拖拽控制 **一体化安装包**。

## 一键安装

```bash
cd Bt_voice_pull_bag_0721
sudo ./install.sh
```

安装前会自动：

- 停止并禁用旧 `bird-ble` / `torque-cmd-vel` 等残留服务
- 删除旧 unit 文件与 `/etc/default/bird-ble`、`torque-cmd-vel`
- 清理残留进程

然后依次安装蓝牙（含语音加载）、拖拽控制单元。

## 目录结构

```
Bt_voice_pull_bag_0721/
  install.sh / uninstall.sh
  VERSION
  ble/              # 蓝牙 GATT 遥控（字节码）
  voice_remind/     # 语音提示（由 bird-ble 加载）
  pull_move/        # 力矩拖拽 → /cmd_vel
```

## 服务说明

| 功能 | 服务 | 默认 |
|------|------|------|
| 蓝牙 + 语音 | `bird-ble.service` | 开机自启 |
| 拖拽控制 | `torque-cmd-vel.service` | 默认关闭，小程序 `PULL ON` 启动 |

语音路径：`BIRD_WS=本包根目录`，即 `voice_remind/`。

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
sudo systemctl status torque-cmd-vel
sudo ./uninstall.sh
```

## 前置条件

- Ubuntu 20.04 + ROS Noetic
- 已编译 `~/sim2real`
- PulseAudio（语音播放）
