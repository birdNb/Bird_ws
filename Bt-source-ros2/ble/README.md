# Bird BLE（ROS2 源码）

蓝牙 GATT 遥控模块，属于 `Bt-source-ros2`。控制桥对接量产 `hightorque_controller`（默认 AMP），摇杆发 ROS2 `/joy`。

版本见上级 `../VERSION` 与本目录 `VERSION`。

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
cd app && ./scripts/check.sh
cd app && ./start.sh
```

协议说明：`docs/BLE_PROTOCOL.md`
