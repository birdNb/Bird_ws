# Bird BLE 遥控安装包 (BT_Control_0710)

微信小程序 BLE 从机，适配 **Jetson Orin（USB 蓝牙）** 与 **RK3588s（板载 RTL8822CE）**。

本包为**发布运行时**（Python 字节码，不含 .py 源码）。

## 一键安装

```bash
cd BT_Control_0710
sudo ./install.sh
```

安装内容：
- 系统依赖（bluez、python3-dbus、python3-gi）
- BlueZ `Experimental=true`
- systemd 服务 `bird-ble` 开机自启

## 前置条件

- Ubuntu 20.04 + **ROS Noetic**
- 已编译 `sim2real` 工作空间（`~/sim2real/install` 或 `~/sim2real/devel`）
- 机器人主控已启动 `roscore` / `sim2real_master`（**必须先于或同步于 BLE 服务**）

> BLE 能连接但指令无效？通常是 roscore 未就绪。先启动主控，再执行  
> `sudo systemctl restart bird-ble`，日志应出现 `ROS 控制桥接已启动`。

可选功能（需目标机 `Bird_ws` 内额外组件）：
- `locate_face_cpp` — `locate_face ON/OFF`
- `sound_demo` — 语音 `sound ON/OFF`

## 目录结构

```
BT_Control_0710/
  install.sh / uninstall.sh   # 一键安装
  README.md / VERSION
  ble_device_name.conf        # 蓝牙广播名
  app/                        # 运行时（字节码 + 脚本）
  docs/                       # 协议与小程序参考
```

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
./app/scripts/check.sh
sudo ./app/scripts/recover.sh
cd app && ./start.sh          # 前台调试
sudo ./uninstall.sh           # 移除自启
```

## 小程序参数

- 广播名：见 `ble_device_name.conf`（默认 `HT_88888888`）
- 服务 FFE0 / 写入 FFE1 / 通知 FFE2

协议详见 `docs/BLE_PROTOCOL.md`。

## 平台

自动识别 Orin / RK3588s，无需改配置。手动覆盖：

```bash
export BLE_PLATFORM=orin      # 或 rk3588s
```

## 环境变量（/etc/default/bird-ble）

安装后可在该文件修改 `SIM2REAL_WS`、`BIRD_USER`、`EXTRA_ARGS`，然后：

```bash
sudo systemctl daemon-reload
sudo systemctl restart bird-ble
```
