# Bird BLE 遥控服务 (BT_Control_0717)

微信小程序 BLE 从机，接收指令并映射为 ROS 话题。

## 目录结构

```
BT_Control_0717/
  README.md / VERSION
  ble_device_name.conf     # 蓝牙广播名（对外配置）
  start.sh                 # 入口
  docs/                    # 协议 / 小程序参考
  app/                     # 源码与运行脚本
  scripts/build_release.sh # 打无源码安装包
```

顶层只保留对外需要的配置与入口；实现全部在 `app/`。

## 快速启动

```bash
cd ~/Bird_ws/BT_Control_0717
./start.sh
```

## 打发布包（无源码）

```bash
./scripts/build_release.sh
# → ../BT_Control_0717_bag/  与  ../BT_Control_0717_bag.tar.gz
```

目标机：

```bash
cd BT_Control_0717_bag && sudo ./install.sh
```

## 小程序参数

- 广播名：`ble_device_name.conf`（默认 `HT_88888888`）
- 服务 FFE0 / 写入 FFE1 / 通知 FFE2
- 主广播含 16-bit FFE0（安卓 `services` 过滤可用）

详见 [docs/BLE_PROTOCOL.md](docs/BLE_PROTOCOL.md)。

## 平台

| 主控 | 蓝牙 | 广播 |
|------|------|------|
| Jetson Orin | USB 模块 | btmgmt |
| RK3588s | 板载 RTL8822CE | Legacy HCI |

覆盖：`export BLE_PLATFORM=orin` 或 `rk3588s`
