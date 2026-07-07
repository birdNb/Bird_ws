# Bird BLE 遥控服务

微信小程序 BLE 从机，接收指令并映射为 ROS 话题。

## 平台适配

| 主控 | 蓝牙硬件 | 广播方式 |
|------|----------|----------|
| **Jetson Orin** | 外接 **USB 蓝牙模块** | btmgmt |
| **RK3588s LubanCat** | **板载** RTL8822CE WiFi+蓝牙一体网卡 | Legacy HCI |

自动检测：`platform_detect.py` / `platform_hw.sh`  
手动覆盖：`export BLE_PLATFORM=orin` 或 `export BLE_PLATFORM=rk3588s`

## 快速启动

```bash
cd ~/Bird_ws/BT_test
./start.sh
```

仅测试广播外发：

```bash
sudo python3 ble_advertise.py
```

## 小程序参数

- 广播名：`HT_88888888`（`ble_device_name.conf`）
- 服务 FFE0 / 写入 FFE1 / 通知 FFE2

详见 [BLE_PROTOCOL.md](BLE_PROTOCOL.md)。

## 运维

```bash
./scripts/check.sh
sudo ./scripts/recover.sh
```

RK 板载网卡上 `Discovering is not writable` 告警可忽略。
