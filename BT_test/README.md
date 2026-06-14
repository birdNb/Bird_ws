# Bird BLE 遥控服务

Orin 板载蓝牙 BLE 从机，接收微信小程序指令，映射为实体手柄等效 ROS 话题，零侵入复用 `sim2real_master` 控制链路。

## 目录结构

```
BT_test/
├── README.md                 # 本说明
├── BLE_PROTOCOL.md           # 小程序对接协议（指令/UUID/流程）
├── start.sh                  # 启动入口
├── run_ble_with_ros.sh       # sudo 下加载 ROS 并启动 GATT
├── ros_env.sh                # ROS 环境
├── ble_gatt_server.py        # GATT 从机 + 连接
├── ble_command_dispatcher.py   # 指令分发
├── ble_ros_bridge.py           # ROS 桥接
├── ble_status_telemetry.py     # IP/电量/FSM → FFE2
├── ble_log.py                  # RX红 TX绿 日志
├── scripts/
│   ├── check.sh              # 检查 BLE 广播状态
│   └── recover.sh            # 蓝牙适配器恢复
└── docs/
    └── miniprogram_ble_snippet.js  # 小程序参考实现
```

## 快速启动

```bash
cd ~/Bird_ws/BT_test
./start.sh
```

首次若 GATT 注册失败：`./start.sh --setup`

## 依赖

- `bluez` `python3-dbus` `python3-gi`
- ROS Noetic + `sim2real_msg`（`~/sim2real/install`）

## 小程序连接

| 项 | 值 |
|----|-----|
| 广播名 | `Bird_BLE_Test` |
| 服务 | `0000FFE0-0000-1000-8000-00805F9B34FB` |
| 写入 | `0000FFE1-...` |
| 通知 | `0000FFE2-...` |

详细协议见 [BLE_PROTOCOL.md](BLE_PROTOCOL.md)。

## 运维

```bash
./scripts/check.sh      # 检查广播是否在发
sudo ./scripts/recover.sh  # hci0 丢失时恢复
```

**注意**：仅在微信小程序内连接，勿在手机系统设置里配对。
