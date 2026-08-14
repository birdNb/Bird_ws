# Bird BLE（源码）

蓝牙 GATT 遥控模块，属于 `Bt_voice_pull_bag` 源码树。

版本见上级 `../VERSION` 与本目录 `VERSION`。

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
cd app && ./scripts/check.sh
cd app && ./start.sh
```

协议说明：`docs/BLE_PROTOCOL.md`
