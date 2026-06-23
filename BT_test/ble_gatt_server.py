#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bird BLE GATT 从机：微信小程序连接，FFE1 收指令，FFE2 回 ACK。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from typing import List, Optional

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# ----- 量产 BLE 服务 FFE0 -----
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
WRITE_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
NOTIFY_CHAR_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

# 语音 FFE3（sound_demo）；仅 --enable-voice 时注册
_BIRD_WS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _BIRD_WS not in sys.path:
    sys.path.insert(0, _BIRD_WS)

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


from ble_log import log_info, log_rx, log_tx, log_warn


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
        # ServiceUUIDs 必须是完整 128-bit，否则 BlueZ 可能不广播
        return {
            LE_ADV_IFACE: {
                "Type": "peripheral",
                "LocalName": self.local_name,
                "ServiceUUIDs": dbus.Array(
                    ["0000ffe0-0000-1000-8000-00805f9b34fb"], signature="s"
                ),
                "IncludeTxPower": dbus.Boolean(False),
                "MinInterval": dbus.UInt16(0x00A0),
                "MaxInterval": dbus.UInt16(0x00C0),
            }
        }

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(LE_ADV_IFACE, in_signature="", out_signature="")
    def Release(self):
        log_info("BLE 广播已释放")
        # BlueZ 主动 Release 时标记为未注册，断连恢复逻辑会重新注册
        server = getattr(self, "_server", None)
        if server is not None:
            server._adv_registered = False



class BleGattServer:
    def __init__(
        self,
        adapter: str,
        name: str,
        echo: bool,
        ros_control: bool,
        enable_voice: bool = False,
    ):
        self.adapter = adapter
        self.name = name
        self.echo = echo
        self.ros_control = ros_control
        self.enable_voice = enable_voice
        self._notify_chrc: Optional[Characteristic] = None
        self._write_chrc: Optional[Characteristic] = None
        self._audio_chrc: Optional[Characteristic] = None
        self._msg_count = 0
        self._connected_devices: set = set()
        self._adapter_path = ""
        self._ros_bridge = None
        self._locate_face = None
        self._hand_identify = None
        self._volume = None
        self._voice = None
        self._dispatcher = None
        self._telemetry = None
        self._connect_hint_ids: List[int] = []
        self._msg_count_at_connect = 0
        self._adv: Optional[Advertisement] = None
        self._adv_manager = None
        self._adv_reregister_pending = False
        self._adv_registered = False

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

    def _schedule_connect_hint(self) -> None:
        self._msg_count_at_connect = self._msg_count
        hint_id = GLib.timeout_add(5000, self._connect_no_data_hint)
        self._connect_hint_ids.append(hint_id)

    def _connect_no_data_hint(self) -> bool:
        if self._msg_count == self._msg_count_at_connect:
            log_warn("[tip] 已连接 5s 仍无 FFE1 写入 — 见 BLE_PROTOCOL.md")
            log_info("      进入遥控页须 write: M_default")
            log_info(f"      写入 UUID: {WRITE_CHAR_UUID}")
        return False

    def _on_phone_connected(self, path: str) -> None:
        if path in self._connected_devices:
            return
        self._connected_devices.add(path)
        label = self._device_label(path)
        log_info(f"*** 手机已连接: {label} ***")
        log_info("    等待 FFE1 写入（握手 M_default）")
        log_info(f"    写入 UUID: {WRITE_CHAR_UUID}")
        self._schedule_connect_hint()

    def _on_phone_disconnected(self, path: str) -> None:
        if path not in self._connected_devices:
            return
        label = self._device_label(path)
        self._connected_devices.discard(path)
        log_info(f"--- 手机已断开: {label} ---")
        if self._dispatcher is not None:
            self._dispatcher.on_disconnect()
        if self._voice is not None:
            self._voice.on_disconnect()
        if self._ros_bridge is not None:
            self._ros_bridge.on_disconnect()
        if not self._connected_devices:
            self._schedule_restart_advertising()

    @staticmethod
    def _dbus_error_text(error: dbus.DBusException) -> str:
        try:
            return f"{error.get_dbus_name()}: {error}"
        except Exception:
            return str(error)

    @staticmethod
    def _is_adv_already_exists(error: dbus.DBusException) -> bool:
        text = BleGattServer._dbus_error_text(error)
        return "AlreadyExists" in text or "Already Exists" in text

    def _refresh_advertising_after_disconnect(self, hci_dev: str = "hci0") -> None:
        """断连后恢复可扫描：广播 D-Bus 对象通常仍注册，重开 advertising 并刷新广播包。"""
        self._run_btmgmt("le", "on")
        self._run_btmgmt("connectable", "on")
        self._run_btmgmt("advertising", "on")

        def _worker() -> None:
            try:
                self._patch_le_adv_data(hci_dev=hci_dev)
                log_info(f"断连后 BLE 广播已恢复: {self.name}")
            except Exception as e:
                log_warn(f"断连后刷新广播包失败: {e}")

        threading.Thread(target=_worker, daemon=True).start()

    def _schedule_restart_advertising(self) -> None:
        """断连后延迟恢复 BLE 广播，便于小程序重新扫描连接。"""
        if self._connected_devices or self._adv is None:
            return
        if self._adv_reregister_pending:
            return
        self._adv_reregister_pending = True
        GLib.timeout_add(400, self._restart_advertising_cb)

    def _restart_advertising_cb(self) -> bool:
        self._adv_reregister_pending = False
        if self._connected_devices or self._adv is None:
            return False

        hci_dev = self._adapter_path.split("/")[-1] if self._adapter_path else "hci0"

        # 常见情况：断连后 BlueZ 仍保留 Advertisement 注册 → 直接 Register 会 AlreadyExists
        if self._adv_registered or self._adv_manager is None:
            log_info("断连后恢复 BLE 可扫描状态...")
            self._refresh_advertising_after_disconnect(hci_dev)
            return False

        def adv_done() -> None:
            self._adv_registered = True
            log_info(f"断连后 BLE 广播已重新注册: {self.name}")
            self._refresh_advertising_after_disconnect(hci_dev)

        def adv_failed(error: dbus.DBusException) -> None:
            if self._is_adv_already_exists(error):
                self._adv_registered = True
                log_info("断连后 BLE 广播仍注册，刷新广播包")
                self._refresh_advertising_after_disconnect(hci_dev)
                return
            log_warn(f"断连后广播恢复失败: {self._dbus_error_text(error)}，尝试注销后重注册")
            self._reregister_advertisement(hci_dev)

        try:
            self._adv_manager.RegisterAdvertisement(
                self._adv.get_path(),
                {},
                reply_handler=adv_done,
                error_handler=adv_failed,
            )
        except dbus.exceptions.DBusException as e:
            if self._is_adv_already_exists(e):
                self._adv_registered = True
                log_info("断连后 BLE 广播仍注册，刷新广播包")
                self._refresh_advertising_after_disconnect(hci_dev)
            else:
                log_warn(f"断连后 RegisterAdvertisement 异常: {self._dbus_error_text(e)}")
                self._reregister_advertisement(hci_dev)
        return False

    def _reregister_advertisement(self, hci_dev: str) -> None:
        if self._adv is None or self._adv_manager is None:
            return

        def register_again() -> None:
            def on_registered() -> None:
                self._adv_registered = True
                log_info(f"断连后 BLE 广播已重新注册: {self.name}")
                self._refresh_advertising_after_disconnect(hci_dev)

            def on_register_failed(err: dbus.DBusException) -> None:
                log_warn(
                    f"断连后重注册广播仍失败: {self._dbus_error_text(err)}"
                )
                GLib.timeout_add(3000, self._restart_advertising_cb)

            try:
                self._adv_manager.RegisterAdvertisement(
                    self._adv.get_path(),
                    {},
                    reply_handler=on_registered,
                    error_handler=on_register_failed,
                )
            except dbus.exceptions.DBusException as e:
                log_warn(
                    f"断连后 RegisterAdvertisement 异常: {self._dbus_error_text(e)}"
                )
                GLib.timeout_add(3000, self._restart_advertising_cb)

        def unregister_failed(error: dbus.DBusException) -> None:
            err = self._dbus_error_text(error)
            if "DoesNotExist" in err or "NotFound" in err:
                register_again()
                return
            log_warn(f"断连后注销广播失败: {err}")
            GLib.timeout_add(3000, self._restart_advertising_cb)

        try:
            self._adv_manager.UnregisterAdvertisement(
                self._adv.get_path(),
                reply_handler=register_again,
                error_handler=unregister_failed,
            )
            self._adv_registered = False
        except dbus.exceptions.DBusException as e:
            err = self._dbus_error_text(e)
            if "DoesNotExist" in err or "NotFound" in err:
                register_again()
            else:
                log_warn(f"断连后 UnregisterAdvertisement 异常: {err}")
                GLib.timeout_add(3000, self._restart_advertising_cb)

    def _on_device_props_changed(
        self, interface: str, changed, invalidated, path: str = ""
    ) -> None:
        if interface != DEVICE_IFACE or "Connected" not in changed:
            return
        if not str(path).startswith(self._adapter_path):
            return
        if bool(changed["Connected"]):
            self._on_phone_connected(path)
        else:
            self._on_phone_disconnected(path)

    def _on_interfaces_added(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        if not str(path).startswith(self._adapter_path):
            return
        dev = interfaces[DEVICE_IFACE]
        if dev.get("Connected", False):
            self._on_phone_connected(path)

    def _on_interfaces_removed(self, path, interfaces) -> None:
        if DEVICE_IFACE not in interfaces:
            return
        self._on_phone_disconnected(path)

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

    def _on_audio_write(self, data: bytes, options) -> None:
        opt = dict(options) if options else {}
        if opt.get("event") == "read":
            return
        if self._voice is not None:
            self._voice.on_audio_write(data)

    def _on_write(self, data: bytes, options) -> None:
        opt = dict(options) if options else {}
        if opt.get("event") == "read":
            log_info(f"手机读取 FFE1 ({len(data)} bytes) — 须 write")
            return

        self._msg_count += 1

        # 小程序语音：FFE1 二进制 [0x0B, seq_hi, seq_lo, pcm...]
        if len(data) >= 3 and data[0] == 0x0B:
            if self._voice is not None:
                self._voice.on_audio_write(data)
            else:
                log_warn(f"[sound] 音频包 {len(data)}B 但语音模块未加载")
            return

        if self._dispatcher is not None:
            self._dispatcher.dispatch(data)

    def _notify_on_main_thread(self, data: bytes) -> None:
        """FFE2 notify 必须在 GLib 主线程发送，避免跨线程 D-Bus 导致断联。"""
        GLib.idle_add(self._do_notify, data)

    def _do_notify(self, data: bytes) -> bool:
        if self._notify_chrc is not None:
            self._notify_chrc.notify(data)
        return False

    def _send_ack(self, wire: str) -> None:
        if not self.echo or self._notify_chrc is None:
            return
        from ble_command_dispatcher import make_notify_reply

        payload = make_notify_reply(wire)
        self._notify_on_main_thread(payload)
        log_tx(f"ACK:{wire}")

    def _send_command_echo(self, wire: str) -> None:
        """步态/电源：回传与上行相同的原文（无 ACK: 前缀）。"""
        if not self.echo or self._notify_chrc is None:
            return
        payload = wire.encode("utf-8", errors="replace")[:180]
        self._notify_on_main_thread(payload)
        log_tx(wire)

    def _handle_dispatched(self, kind, payload: str) -> None:
        if kind.value == "locate_face":
            if self._locate_face is not None:
                self._locate_face.handle(payload)
            return
        if kind.value == "hi":
            if self._hand_identify is not None:
                self._hand_identify.handle(payload)
            return
        if kind.value == "volume":
            if self._volume is not None:
                self._volume.handle(payload)
            return
        if kind.value == "sound":
            if self._voice is not None:
                wire = self._voice.on_sound_command(payload)
                if wire:
                    self._send_command_echo(wire)
            else:
                log_warn("语音模块未加载（检查 sound_demo）")
            return
        if self._ros_bridge is not None and self.ros_control:
            self._ros_bridge.handle_command(kind.value, payload)

    def _on_notify_start(self) -> None:
        log_info("手机已订阅 FFE2 notify")
        if self._telemetry is not None:
            self._telemetry.on_subscribed()

    def _on_notify_stop(self) -> None:
        log_info("手机已取消 notify 订阅")
        if self._telemetry is not None:
            self._telemetry.on_unsubscribed()

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
            log_warn(f"btmgmt {' '.join(args)}: {e}")

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
        log_info("已切换 BLE-only：经典蓝牙不可配对（避免手机系统弹窗）")

    def _set_adapter_props(self, adapter_path: str) -> None:
        props = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
        )
        try:
            props.Set(ADAPTER_IFACE, "Alias", self.name)
            log_info(f"BLE 广播名: {self.name}")
        except dbus.exceptions.DBusException as e:
            log_warn(f"设置 Alias 失败: {e}")
        try:
            props.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
            # 勿开经典蓝牙可发现/可配对，否则手机系统会一直弹「连接/配对」
            props.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(False))
            props.Set(ADAPTER_IFACE, "DiscoverableTimeout", dbus.UInt32(0))
            props.Set(ADAPTER_IFACE, "PairableTimeout", dbus.UInt32(0))
            props.Set(ADAPTER_IFACE, "Discovering", dbus.Boolean(False))
        except dbus.exceptions.DBusException as e:
            log_warn(f"适配器属性: {e}")
        self._enter_ble_only_mode()

    def _verify_le_advertising(self, adapter_path: str) -> None:
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE, adapter_path), DBUS_PROP_IFACE
            )
            alias = str(props.Get(ADAPTER_IFACE, "Alias"))
            addr = str(props.Get(ADAPTER_IFACE, "Address"))
            log_info(f"当前适配器 MAC: {addr}  Alias: {alias}")
            log_info("小程序请用 services=[FFE0] 扫描，或按此 MAC 连接")
        except dbus.exceptions.DBusException as e:
            log_warn(f"读取适配器信息失败: {e}")

    def _patch_le_adv_data(self, hci_dev: str = "hci0") -> None:
        """
        BlueZ 5.53 + BT 5.1 adapter workaround：

        D-Bus RegisterAdvertisement → MGMT Add Advertising (Flags=0, data=empty)
        → 内核使用 Extended Advertising (0x0037) 广播**空数据包**（无 Name/UUID）
        → Legacy hcitool cmd (0x0008/0x0009) 被 Extended Advertising 忽略

        正确做法：用 btmgmt add-adv 覆盖 instance 1，写入 UUID+Name。
        fallback：直接写 LE Set Extended Advertising Data (0x0037) HCI 命令。
        """
        time.sleep(0.4)  # 等 BlueZ MGMT 流程完成

        hci_idx = hci_dev.replace("hci", "")  # "hci0" → "0"

        # ── 主路径：btmgmt add-adv 覆盖 instance 1 ──────────────────
        # -u ffe0  : 将 0xFFE0 写入 Complete 16-bit UUID AD
        # -n name  : 将设备名写入 Complete Local Name AD
        # -g       : General Discoverable (Flags 0x02)
        # -b       : BR/EDR Not Supported (Flags 0x04)
        # 1        : Instance 1（覆盖 D-Bus 注册的空 instance 1）
        try:
            # -u ffe0 : Service UUID 0xFFE0
            # -n      : 将适配器 Alias 写入 scan-rsp-local-name（无需带值）
            # -g      : General Discoverable flag
            # -m      : managed-flags（内核自动管理 Flags AD type）
            # -c      : connectable
            # 1       : instance 1（覆盖 D-Bus 注册的空 instance）
            r = subprocess.run(
                ["btmgmt", "--index", hci_idx, "add-adv",
                 "-u", "ffe0", "-n", "-g", "-m", "-c", "1"],
                capture_output=True, timeout=6,
            )
            out = r.stdout.decode("utf-8", errors="replace").strip()
            err = r.stderr.decode("utf-8", errors="replace").strip()
            if r.returncode == 0:
                log_info(f"[adv] LE 广播包已写入: UUID=FFE0 + Name={self.name}")
                return
            log_warn(f"[adv] btmgmt add-adv 失败(rc={r.returncode}): {out} {err}")
        except Exception as exc:
            log_warn(f"[adv] btmgmt add-adv 异常: {exc}")

        # ── fallback：直接写 LE Set Extended Advertising Data (0x0037) ──
        # 针对 BT 5.0+ 控制器，Extended Advertising handle=1
        # HCI LE Set Extended Advertising Data:
        #   Advertising_Handle (1B) | Operation (1B=0x03完整) |
        #   Fragment_Preference (1B=0x01) | Data_Length (1B) | Data
        name_b = self.name.encode("utf-8")[:25]  # 留余量
        # ADV data: Flags(3B) + 16-bit UUID(4B) + LocalName(2+N B)
        adv_payload = (
            bytes([0x02, 0x01, 0x06])                        # Flags
            + bytes([0x03, 0x03, 0xE0, 0xFF])                # 16-bit UUID
            + bytes([len(name_b) + 1, 0x09]) + name_b        # Complete Local Name
        )
        if len(adv_payload) > 31:
            adv_payload = adv_payload[:31]
        ext_args = (
            ["0x01", "0x03", "0x01", f"0x{len(adv_payload):02x}"]
            + [f"0x{b:02x}" for b in adv_payload]
        )
        try:
            r2 = subprocess.run(
                ["hcitool", "-i", hci_dev, "cmd", "0x08", "0x0037"] + ext_args,
                capture_output=True, timeout=4,
            )
            if r2.returncode == 0:
                log_info(f"[adv] LE Ext 广播包已写入 (fallback): Name={self.name}")
            else:
                log_warn(f"[adv] LE Ext 广播写入失败: rc={r2.returncode}")
        except Exception as exc2:
            log_warn(f"[adv] LE Ext fallback 异常: {exc2}")

    def _start_volume_manager(self) -> None:
        try:
            from ble_volume_manager import VolumeController
        except ImportError as e:
            log_warn(f"无法加载 ble_volume_manager: {e}")
            return
        self._volume = VolumeController(log=log_info)

    def _start_hand_identify_manager(self) -> None:
        try:
            from ble_hand_identify_manager import HandIdentifyManager
        except ImportError as e:
            log_warn(f"无法加载 ble_hand_identify_manager: {e}")
            return
        self._hand_identify = HandIdentifyManager(log=log_info)
        log_info("手势管理器已就绪（HI ON/OFF）")

    def _start_locate_face_manager(self) -> None:
        try:
            from ble_locate_face_manager import LocateFaceManager
        except ImportError as e:
            log_warn(f"无法加载 ble_locate_face_manager: {e}")
            return
        self._locate_face = LocateFaceManager(log=log_info)
        log_info("locate_face 管理器已就绪（locate_face ON/OFF）")

    def _start_voice_manager(self) -> None:
        try:
            from sound_demo.integrate import VoiceBleIntegration

            self._voice = VoiceBleIntegration(log=log_info)
            log_info("语音传输已就绪（FFE1: sound ON/OFF + 0x0B 音频包）")
        except ImportError as e:
            log_warn(f"无法加载 sound_demo: {e}")

    def _start_ros_bridge(self) -> None:
        if not self.ros_control:
            return
        try:
            from ble_ros_bridge import BleRosBridge
        except ImportError as e:
            log_warn(f"无法加载 ble_ros_bridge: {e}")
            return
        self._ros_bridge = BleRosBridge(log=log_info)
        if self._ros_bridge.start():
            log_info("ROS 控制桥接已启动（/cmd_vel + /joy_msg）")
        else:
            log_warn("ROS 桥接启动失败")
            self._ros_bridge = None

    def _start_dispatcher(self) -> None:
        try:
            from ble_command_dispatcher import CommandDispatcher
        except ImportError as e:
            log_warn(f"无法加载 ble_command_dispatcher: {e}")
            return
        self._dispatcher = CommandDispatcher(
            handle=self._handle_dispatched,
            ack=self._send_ack if self.echo else None,
            echo_confirm=self._send_command_echo if self.echo else None,
            log_rx=log_rx,
            log_warn=log_warn,
        )
        self._dispatcher.start()
        log_info("指令分发器已启动")

    def _wire_mp_telemetry(self) -> None:
        """MP 上电后立即推 mp:ON，避免小程序自动站立误判为未上电。"""
        if self._ros_bridge is None or self._telemetry is None:
            return

        def _on_mp_state(motor_on: bool) -> None:
            wire = "ON" if motor_on else "OFF"
            self._telemetry.push_mp_state(wire, force=True)

        self._ros_bridge.set_motor_power_listener(_on_mp_state)
        wire = self._ros_bridge.get_motor_power_wire()
        if wire in ("ON", "OFF"):
            self._telemetry.push_mp_state(wire, force=True)

    def _start_telemetry(self) -> None:
        if self._notify_chrc is None:
            return
        try:
            from ble_status_telemetry import BleStatusTelemetry
        except ImportError as e:
            log_warn(f"无法加载 ble_status_telemetry: {e}")
            return
        self._telemetry = BleStatusTelemetry(
            notify=self._notify_on_main_thread,
            motor_power_fn=self._read_motor_power_wire,
            battery_fn=self._read_battery_pct,
        )
        self._telemetry.start()

    def _read_motor_power_wire(self) -> Optional[str]:
        if self._ros_bridge is None:
            return None
        return self._ros_bridge.get_motor_power_wire()

    def _read_battery_pct(self) -> Optional[int]:
        if self._ros_bridge is None:
            return None
        return self._ros_bridge.get_battery_pct()

    def run(self) -> int:
        adapter_path = self._find_adapter()
        self._adapter_path = adapter_path
        log_info(f"使用适配器: {adapter_path}")
        self._set_adapter_props(adapter_path)
        self._watch_devices()
        self._start_locate_face_manager()
        self._start_hand_identify_manager()
        self._start_volume_manager()
        self._start_voice_manager()
        self._start_ros_bridge()
        self._start_dispatcher()

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
        if self.enable_voice and self._voice is not None:
            from sound_demo.integrate import AUDIO_CHAR_UUID

            self._audio_chrc = Characteristic(
                self.bus,
                2,
                AUDIO_CHAR_UUID,
                ["write", "write-without-response"],
                service,
                on_write=self._on_audio_write,
            )
            service.add_characteristic(self._audio_chrc)
        app.add_service(service)
        self._start_telemetry()
        self._wire_mp_telemetry()

        adv = Advertisement(self.bus, 0, self.name)
        adv._server = self
        self._adv = adv
        adv_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), LE_ADV_MANAGER_IFACE
        )
        self._adv_manager = adv_manager
        gatt_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE, adapter_path), GATT_MANAGER_IFACE
        )

        def register_done() -> None:
            log_info("GATT 服务已注册，等待手机连接")

        def register_failed(error: dbus.DBusException) -> None:
            log_warn(f"GATT 注册失败: {error}")
            log_info(
                "  尝试: ./start.sh --setup  "
                "或在 /etc/bluetooth/main.conf 加 Experimental=true"
            )
            self.mainloop.quit()

        def adv_done() -> None:
            self._adv_registered = True
            log_info(f"BLE 广播已开启，小程序应能扫到名称: {self.name}")
            log_info(f"    或按服务 UUID 扫描: {SERVICE_UUID}")
            self._verify_le_advertising(adapter_path)
            # BlueZ 5.53 bug: MGMT Add Advertising 数据为空，手动写入 HCI 广播包
            hci_dev = adapter_path.split("/")[-1]  # /org/bluez/hci0 → hci0
            threading.Thread(
                target=self._patch_le_adv_data,
                kwargs={"hci_dev": hci_dev},
                daemon=True,
            ).start()

        def adv_failed(error: dbus.DBusException) -> None:
            log_warn(f"广播注册失败: {error}")
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
        log_info(">>> Bird BLE 遥控服务 <<<")
        log_info(f"    广播名: {self.name}  MAC: {addr}")
        log_info("    FFE0/FFE1/FFE2 | 协议见 BLE_PROTOCOL.md")
        if self._voice is not None:
            log_info("    FFE1 语音: sound ON/OFF + 0x0B 音频包 | 见 sound_demo/README.md")
        if self.enable_voice and self._voice is not None:
            log_info("    FFE3 语音(可选备用通道)")
        log_info("    日志: RX红(收) TX绿(发)")
        if self.ros_control:
            log_info("    摇杆→/cmd_vel 20Hz | 模式/动作→/joy_msg | 状态→FFE2")
        log_info("    Ctrl+C 退出")
        print()

        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            log_info("退出")
        finally:
            if self._telemetry is not None:
                self._telemetry.stop()
            if self._dispatcher is not None:
                self._dispatcher.stop()
            if self._ros_bridge is not None:
                self._ros_bridge.stop()
            if self._locate_face is not None:
                self._locate_face.stop()
            if self._hand_identify is not None:
                self._hand_identify.stop()
            if self._voice is not None:
                self._voice.on_disconnect()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Bird BLE GATT 遥控服务")
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
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="不将 BLE 指令转为 ROS 话题（仅打印）",
    )
    parser.add_argument(
        "--enable-voice",
        action="store_true",
        help="启用 FFE3 语音 PCM 实时播放（sound_demo）",
    )
    args = parser.parse_args()
    server = BleGattServer(
        args.adapter,
        args.name,
        echo=not args.no_echo,
        ros_control=not args.no_ros,
        enable_voice=args.enable_voice,
    )
    return server.run()


if __name__ == "__main__":
    sys.exit(main())
