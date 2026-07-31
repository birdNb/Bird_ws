# Bt_voice_pull_bag（OTA / 手工通用 · 加密发布包）

蓝牙遥控 + 语音提醒 + 拖拽控制 + 头追 **四合一**安装包。  
**本目录为加密发布包（字节码）**；日常开发请改源码树 `Bt_voice_pull_bag/`（无日期后缀）。

**RK3588 与 Jetson Orin 通用**；头追适配 **ZED Mini（Orin）** 与 **D435i（RK）**。

版本：`0730`（发布运行时，Python 已编译为字节码）

## 一键安装

```bash
cd Bt_voice_pull_bag_0730
sudo ./install.sh
# OTA：ota-client 直接调用 ./install.sh（会自动提权）
```

安装时会：

1. 按平台自动 sudo（RK 密码 `ht`，Orin 密码 `nvidia`）
2. 同步到 `~/Bird_ws/Bt_voice_pull_bag`
3. 清理旧 bird-ble / torque-cmd-vel / 头追残留
4. 安装并启动蓝牙；拖拽默认关闭（`PULL ON`）
5. 头追随包内 `locate_face_cpp`，小程序 `locate_face ON/OFF`
6. 写入 OTA：`~/sim2real/version.json` → `ble-all-0730-<日期>`

## 目录结构

```
Bt_voice_pull_bag_0730/
  ble_device_name.conf # 蓝牙广播名（默认 HT_88888888）
  install.sh / uninstall.sh / VERSION / README.md
  ble/                 # 蓝牙 GATT（字节码）
  voice_remind/        # 语音提示（字节码 + wav）
  pull_move/           # 拖拽（字节码）
  locate_face_cpp/     # 头追运行时（二进制 + 模型 + worker 字节码）
```

## 头追相机

| 平台 | 默认相机 | 说明 |
|------|----------|------|
| Orin + ZED Mini | `/dev/video0` | 并排双目自动裁左眼 |
| RK + D435i | `/dev/video4` | 彩色流 |
| 手动覆盖 | `LOCATE_FACE_CAMERA=N` | 安装环境或 shell 导出 |

需 FSM=`EXEC_DEFAULT(5)` 后才下发脖子目标。

## 服务 / 功能默认

| 功能 | 触发 | 默认 |
|------|------|------|
| 语音提示 | `sound ON/OFF` | **开** |
| 拖拽 | `torque-cmd-vel.service` / `PULL ON` | **关** |
| 头追 | BLE `locate_face ON/OFF` | **关** |
| 蓝牙 GATT | `bird-ble.service` | 开机自启 |

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
tail -f ~/Bird_ws/Bt_voice_pull_bag/locate_face_cpp/locate_face_ble.log
```
