# Bird BLE 遥控安装包 (BT_Control_0717_bag)

发布运行时（Python 字节码，不含 .py 源码）。

## 一键安装

```bash
cd BT_Control_0717_bag
sudo ./install.sh
```

## 目录结构

```
BT_Control_0717_bag/
  install.sh / uninstall.sh
  README.md / VERSION
  ble_device_name.conf        # 蓝牙广播名（可改）
  app/                        # 运行时
  docs/                       # 协议与小程序参考
```

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
./app/scripts/check.sh
cd app && ./start.sh
```
