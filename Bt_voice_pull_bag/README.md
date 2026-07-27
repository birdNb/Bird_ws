# Bt_voice_pull_bag（OTA / 手工通用）

蓝牙遥控 + 语音提醒 + 拖拽控制 + 头追 **四合一**安装包。**RK3588 与 Jetson Orin 通用。**

版本：`0728`

## 一键安装

```bash
# 手工
cd Bt_voice_pull_bag_0728   # 或解压后的目录名
sudo ./install.sh

# OTA：由 ota-client 直接调用 ./install.sh（普通用户即可，脚本会自动提权）
```

安装时会：

1. 按平台自动 `sudo`（RK 密码 `ht`，Orin 密码 `nvidia`；已配置免密则优先免密）
2. 同步到固定目录 `~/Bird_ws/Bt_voice_pull_bag`（OTA 删临时包后仍可用）
3. 清理旧 `bird-ble` / `torque-cmd-vel` / 头追残留进程
4. 安装并启动蓝牙；拖拽默认不开机自启（`PULL ON`）
5. 头追随包内 `locate_face_cpp`，由小程序 `locate_face ON/OFF` 启停
6. 写入 OTA 本地版本：`~/sim2real/version.json` → `ble-all-<VERSION>-<日期>`

## OTA 发布

包格式已符合组件升级包约定：

```text
Bt_voice_pull_bag_0728/
  install.sh
  ...
```

服务端算法组件 **code 前两段**须为：`ble-all`（例如本地 `ble-all-0728-20260728`）。

## 目录结构

```
Bt_voice_pull_bag_0728/     # 发布归档内顶层目录
  install.sh / uninstall.sh
  VERSION
  ble/
  voice_remind/
  pull_move/
  locate_face_cpp/          # 头追运行时（二进制+模型）
```

装机后正式路径：

```text
~/Bird_ws/Bt_voice_pull_bag/
```

## 服务 / 功能

| 功能 | 触发 | 默认 |
|------|------|------|
| 蓝牙 + 语音 | `bird-ble.service` | 开机自启 |
| 拖拽 | `torque-cmd-vel.service` | 默认关闭，`PULL ON` 启动 |
| 头追 | 无独立 systemd | `locate_face ON` 启动 / `OFF` 停止并回中 |

头追说明：需相机（默认 `/dev/video4`），且 FSM 在行走模式（`EXEC_DEFAULT=5`）后才会视觉伺服。

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
# 头追日志（ON 之后）
tail -f ~/Bird_ws/Bt_voice_pull_bag/locate_face_cpp/locate_face_ble.log
./uninstall.sh
```
