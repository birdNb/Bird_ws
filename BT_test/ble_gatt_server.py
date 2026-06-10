#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE GATT 从机测试：手机/微信小程序连接后向可写特征发数据，本脚本打印收到的内容。

小程序侧建议 UUID（与 start.sh 输出一致）：
  服务 serviceId:  0000FFF0-0000-1000-8000-00805F9B34FB
  写入 write UUID:  0000FFF1-0000-1000-8000-00805F9B34FB  (write / writeNoResponse)
  通知 notify UUID: 0000FFF2-0000-1000-8000-00805F9B34FB  (可选，收板子回显)

依赖：sudo apt install bluez python3-dbus python3-gi
需蓝牙服务运行；若注册 GATT 失败，在 /etc/bluetooth/main.conf [General] 加 Experimental=true 后重启 bluetooth。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from typing import List, Optional

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# ----- 与微信小程序常见的 FFF0 自定义服务对齐 -----
SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
WRITE_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
NOTIFY_CHAR_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

DEVICE_NAME = "Bird_BLE_Test"
# 小程序也可按 MAC 连接（系统设置里看到的蓝牙地址）
BOARD_BDADDR = "00:19:86:00:2E:AF"
ADAPTER_PATH = "/org/bluez/hci0"
BLUEZ_SERVICE = "org.bluez"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADV_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"
LE_ADV_IFACE = "org.bluez.LEAdvertisement1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHAR_IFACE = "org.bluez.GattCharacteristic1"
GATT_DESC_IFACE = "org.bluez.GattDescriptor1"
DEVICE_IFACE = "org.bluez.Device1"
ADAPTER_IFACE = "org.bluez.Adapter1"


def log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")
    sys.stdout.flush()


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


class Application(dbus.service.Object):
    def __init__(self, bus: dbus.SystemBus):
        self.path = "/org/bird/ble_test"
        self.services: List[dbus.service.Object] = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def add_service(self, service: dbus.service.Object) -> None:
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.get_characteristics():
                response[chrc.get_path()] = chrc.get_properties()
                for desc in chrc.get_descriptors():
                    response[desc.get_path()] = desc.get_properties()
        return response


class Service(dbus.service.Object):
    PATH_BASE = "/org/bird/ble_test/service"

    def __init__(self, bus: dbus.SystemBus, index: int, uuid: str, primary: bool):
        self.path = f"{self.PATH_BASE}{index}"
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics: List[dbus.service.Object] = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, chrc: dbus.service.Object) -> None:
        self.characteristics.append(chrc)

    def get_characteristics(self) -> List[dbus.service.Object]:
        return self.characteristics

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": self.primary,
                "Characteristics": dbus.Array(
                    [c.get_path() for c in self.characteristics], signature="o"
                ),
            }
        }


class Characteristic(dbus.service.Object):
    PATH_BASE = "/org/bird/ble_test/char"

    def __init__(
        self,
        bus: dbus.SystemBus,
        index: int,
        uuid: str,
        flags: List[str],
        service: Service,
        on_write=None,
        on_notify_start=None,
        on_notify_stop=None,
    ):
        self.path = f"{self.PATH_BASE}{index}"
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors: List[dbus.service.Object] = []
        self._value = bytearray(b"Bird BLE ready")
        self._on_write = on_write
        self._on_notify_start = on_notify_start
        self._on_notify_stop = on_notify_stop
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self) -> str:
        return dbus.ObjectPath(self.path)

    def get_descriptors(self) -> List[dbus.service.Object]:
        return self.descriptors

    def get_properties(self):
        return {
            GATT_CHAR_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": self.flags,
                "Value": dbus.Array(self._value, signature="y"),
            }
        }

    @dbus.service.method(GATT_CHAR_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        if self._on_write:
            self._on_write(bytes(self._value), {"event": "read"})
        return self._value

    @dbus.service.method(GATT_CHAR_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        data = bytes(value)
        self._value = bytearray(data)
        if self._on_write:
            self._on_write(data, options)

    @dbus.service.method(GATT_CHAR_IFACE)
    def StartNotify(self):
        if self._on_notify_start:
            self._on_notify_start()

    @dbus.service.method(GATT_CHAR_IFACE)
    def StopNotify(self):
        if self._on_notify_stop:
            self._on_notify_stop()

    def notify(self, data: bytes) -> None:
        if "notify" not in self.flags and "indicate" not in self.flags:
            return
        self._value = bytearray(data)
        self.PropertiesChanged(
            GATT_CHAR_IFACE,
            {"Value": dbus.Array(self._value, signature="y")},
            [],
        )

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/bird/ble_test/advertisement"

    def __init__(self, bus: dbus.SystemBus, index: int, local_name: str):
        self.path = f"{self.PATH_BASE}{index}"
        self.local_name = local_name
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        # 广播包仅 31 字节：用 16 位 UUID + 名称，避免名称被挤掉
        return {
            LE_ADV_IFACE: {
                "Type": "peripheral",
                "LocalName": self.local_name,
                "ServiceUUIDs": dbus.Array(["fff0"], signature="s"),
                "Discoverable": dbus.Boolean(True),
                "IncludeTxPower": False,
                "MinInterval": dbus.UInt16(0x0020),
                "MaxInterval": dbus.UInt16(0x0040),
            }
        }

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(LE_ADV_IFACE, in_signature="", out_signature="")
    def Release(self):
        log("BLE 广播已释放")


def decode_payload(data: bytes) -> str:
    for enc in ("utf-8", "gbk", "latin-1"):
        try:
            return data.decode(enc)
        except UnicodeDecodeError:
            continue
    return repr(data)


class BleTestServer:
    def __init__(self, adapter: str, name: str, echo: bool):
        self.adapter = adapter
        self.name = name
        self.echo = echo
        self._notify_chrc: Optional[Characteristic] = None
        self._write_chrc: Optional[Characteristic] = None
        self._msg_count = 0
        self._connected_devices: set = set()
        self._adapter_path = ""

        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.mainloop = GLib.MainLoop()

    def _device_label(self, path: str) -> str:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, path), DBUS_PROP_IFACE
            )
            values = props.GetAll(DEVICE_IFACE)
            name = str(values.get("Name", "") or values.get("Alias", "") or "未知设备")
            addr = str(values.get("Address", path))
            return f"{name} ({addr})"
        except dbus.exceptions.DBusException:
            return path

    def _on_device_props_changed(
        self, interface: str, changed, invalidated, path: str = ""
    ) -> None:
        if interface != DEVICE_IFACE or "Connected" not in changed:
            return
        if not str(path).startswith(self._adapter_path):
            return
        connected = bool(changed["Connected"])
        label = self._device_label(path)
        if connected:
            if path not in self._connected_devices:
                self._connected_devices.add(path)
                log(f"*** 手机已连接: {label} ***")
                log("    等待小程序向 FFF1 特征写入数据...")
        else:
            if path in self._connected_devices:
                self._connected_devices.discard(path)
                log(f"--- 手机已断开: {label} ---")

    def _on_interfaces_added(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        if not str(path).startswith(self._adapter_path):
            return
        dev = interfaces[DEVICE_IFACE]
        if dev.get("Connected", False):
            if path not in self._connected_devices:
                self._connected_devices.add(path)
                label = self._device_label(path)
                log(f"*** 手机已连接: {label} ***")

    def _on_interfaces_removed(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        if path in self._connected_devices:
            label = self._device_label(path)
            self._connected_devices.discard(path)
            log(f"--- 手机已断开: {label} ---")

    def _watch_devices(self) -> None:
        self.bus.add_signal_receiver(
            self._on_interfaces_added,
            dbus_interface=DBUS_OM_IFACE,
            signal_name="InterfacesAdded",
        )
        self.bus.add_signal_receiver(
            self._on_interfaces_removed,
            dbus_interface=DBUS_OM_IFACE,
            signal_name="InterfacesRemoved",
        )
        self.bus.add_signal_receiver(
            self._on_device_props_changed,
            dbus_interface=DBUS_PROP_IFACE,
            signal_name="PropertiesChanged",
            path_keyword="path",
        )

    def _on_write(self, data: bytes, options) -> None:
        opt = dict(options) if options else {}
        if opt.get("event") == "read":
            log(f"手机读取特征值 ({len(data)} bytes)")
            return

        self._msg_count += 1
        text = decode_payload(data)
        log("=" * 56)
        log(f">>> 收到手机消息 #{self._msg_count}")
        log(f"    文本: {text}")
        log(f"    HEX:  {data.hex(' ')}")
        log(f"    长度: {len(data)} bytes")
        if opt:
            log(f"    选项: {opt}")
        log("=" * 56)

        if self.echo and self._notify_chrc is not None:
            reply = f"ACK:{text}".encode("utf-8", errors="replace")[:180]
            self._notify_chrc.notify(reply)
            log(f"    已回显 ACK 到 notify 特征 ({len(reply)} bytes)")

    def _on_notify_start(self) -> None:
        log("手机已订阅 notify 特征 (FFF2)，可接收板子回显")

    def _on_notify_stop(self) -> None:
        log("手机已取消 notify 订阅")

    def _find_adapter(self) -> str:
        om = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, "/"), DBUS_OM_IFACE
        )
        objects = om.GetManagedObjects()
        for path, ifaces in objects.items():
            if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces:
                if self.adapter == "hci0" and str(path).endswith("hci0"):
                    return str(path)
                if str(path) == self.adapter:
                    return str(path)
        for path, ifaces in objects.items():
            if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces:
                return str(path)
        raise RuntimeError("未找到支持 GATT/广播 的蓝牙适配器，请确认 bluetoothd 已启动")

    def _read_adapter_address(self, adapter_path: str) -> str:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
            )
            return str(props.Get(ADAPTER_IFACE, "Address"))
        except dbus.exceptions.DBusException:
            return BOARD_BDADDR

    def _run_btmgmt(self, *args: str) -> None:
        try:
            subprocess.run(
                ["btmgmt", "--index", "0", *args],
                check=False,
                capture_output=True,
                timeout=3,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
            log(f"[warn] btmgmt {' '.join(args)}: {e}")

    def _enter_ble_only_mode(self) -> None:
        """仅 BLE 广播，关闭经典蓝牙可发现/可配对，避免手机系统反复弹连接框。"""
        try:
            subprocess.run(
                ["hciconfig", "hci0", "noscan"],
                check=False,
                capture_output=True,
                timeout=3,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            pass
        self._run_btmgmt("le", "on")
        self._run_btmgmt("connectable", "on")
        self._run_btmgmt("discov", "off")
        self._run_btmgmt("pairable", "off")
        self._run_btmgmt("bondable", "off")
        log("已切换 BLE-only：经典蓝牙不可配对（避免手机系统弹窗）")

    def _set_adapter_props(self, adapter_path: str) -> None:
        props = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
        )
        try:
            props.Set(ADAPTER_IFACE, "Alias", self.name)
            log(f"BLE 广播名: {self.name}")
        except dbus.exceptions.DBusException as e:
            log(f"[warn] 设置 Alias 失败: {e}")
        try:
            props.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
            # 勿开经典蓝牙可发现/可配对，否则手机系统会一直弹「连接/配对」
            props.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "DiscoverableTimeout", dbus.UInt32(0))
            props.Set(ADAPTER_IFACE, "PairableTimeout", dbus.UInt32(0))
            props.Set(ADAPTER_IFACE, "Discovering", dbus.Boolean(False))
        except dbus.exceptions.DBusException as e:
            log(f"[warn] 适配器属性: {e}")
        self._enter_ble_only_mode()

    def _verify_le_advertising(self, adapter_path: str) -> None:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
            )
            alias = str(props.Get(ADAPTER_IFACE, "Alias"))
            addr = str(props.Get(ADAPTER_IFACE, "Address"))
            log(f"当前适配器 MAC: {addr}  Alias: {alias}")
            log("小程序请用 services=[FFF0] 扫描，或按此 MAC 连接")
        except dbus.exceptions.DBusException as e:
            log(f"[warn] 读取适配器信息失败: {e}")

    def run(self) -> int:
        adapter_path = self._find_adapter()
        self._adapter_path = adapter_path
        log(f"使用适配器: {adapter_path}")
        self._set_adapter_props(adapter_path)
        self._watch_devices()

        app = Application(self.bus)
        service = Service(self.bus, 0, SERVICE_UUID, True)
        self._write_chrc = Characteristic(
            self.bus,
            0,
            WRITE_CHAR_UUID,
            ["write", "write-without-response", "read"],
            service,
            on_write=self._on_write,
        )
        self._notify_chrc = Characteristic(
            self.bus,
            1,
            NOTIFY_CHAR_UUID,
            ["read", "notify"],
            service,
            on_notify_start=self._on_notify_start,
            on_notify_stop=self._on_notify_stop,
        )
        write_chrc = self._write_chrc
        service.add_characteristic(write_chrc)
        service.add_characteristic(self._notify_chrc)
        app.add_service(service)

        adv = Advertisement(self.bus, 0, self.name)
        adv_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), LE_ADV_MANAGER_IFACE
        )
        gatt_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), GATT_MANAGER_IFACE
        )

        def register_done() -> None:
            log("GATT 服务已注册，等待手机连接")

        def register_failed(error: dbus.DBusException) -> None:
            log(f"[error] GATT 注册失败: {error}")
            log(
                "  尝试: ./start.sh --setup  "
                "或在 /etc/bluetooth/main.conf 加 Experimental=true"
            )
            self.mainloop.quit()

        def adv_done() -> None:
            log(f"BLE 广播已开启，小程序应能扫到名称: {self.name}")
            log(f"    或按服务 UUID 扫描: {SERVICE_UUID}")
            self._verify_le_advertising(adapter_path)

        def adv_failed(error: dbus.DBusException) -> None:
            log(f"[error] 广播注册失败: {error}")
            self.mainloop.quit()

        gatt_manager.RegisterApplication(
            app.get_path(),
            {},
            reply_handler=register_done,
            error_handler=register_failed,
        )
        adv_manager.RegisterAdvertisement(
            adv.get_path(),
            {},
            reply_handler=adv_done,
            error_handler=adv_failed,
        )

        print()
        addr = self._read_adapter_address(adapter_path)
        log(">>> BLE 测试服务运行中 <<<")
        log("    请在【微信小程序】里连接，勿在手机系统设置里点配对")
        log("    若曾系统配对，请在手机里「忽略/取消配对」此设备")
        log("    必须保持本脚本运行，小程序才看得到 BLE 广播")
        log(f"    广播名: {self.name}")
        log(f"    MAC:    {addr}")
        log(f"    服务 UUID: {SERVICE_UUID}")
        log(f"    写入 UUID: {WRITE_CHAR_UUID}")
        log(f"    通知 UUID: {NOTIFY_CHAR_UUID}")
        log("    连接成功 / 收到消息 均会在本终端打印")
        log("    Ctrl+C 退出")
        print()

        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            log("退出")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Bird BLE GATT 接收测试")
    parser.add_argument(
        "--name",
        default=DEVICE_NAME,
        help=f"BLE 广播设备名 (默认 {DEVICE_NAME})",
    )
    parser.add_argument("--adapter", default="hci0", help="hci0 或 dbus 路径")
    parser.add_argument(
        "--no-echo",
        action="store_true",
        help="不向 notify 特征回显 ACK",
    )
    args = parser.parse_args()
    server = BleTestServer(args.adapter, args.name, echo=not args.no_echo)
    return server.run()


if __name__ == "__main__":
    sys.exit(main())
