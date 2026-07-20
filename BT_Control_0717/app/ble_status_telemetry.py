#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人 → 小程序 状态遥测（FFE2 notify）。

  IP:192.168.19.11   局域网 IPv4 四段完整地址
  pwr:83       电量 0~100 实际值；握手后立即发送，下降时再推
  mp:ON        电机电源状态；连接后 5s 连发 3 次，变化时再推
  fsm:5        FSM 状态变化时连发 3 次
"""

from __future__ import annotations

import glob
import os
import socket
import sys
import threading
import time
from typing import Callable, Optional

MotorPowerFn = Callable[[], Optional[str]]
BatteryFn = Callable[[], Optional[int]]
OnBatteryFn = Callable[[int], None]

from ble_log import log_info, log_tx, log_warn

IP_POLL_SEC = 15.0
PWR_POLL_SEC = 5.0
FSM_REPEAT = 3
FSM_REPEAT_GAP_SEC = 0.05
MP_BURST_DELAY_SEC = 5.0
MP_BURST_COUNT = 3
MP_BURST_GAP_SEC = 1.0
PWR_RETRY_INTERVAL_SEC = 0.2
PWR_RETRY_MAX_SEC = 8.0
# 机器人实际发布电量的话题（优先 /battery_level UInt8）
BATTERY_FLOAT_TOPICS = ("/battery_level", "/pwr", "/battery_percent", "/battery")
BATTERY_INT_TOPICS = ("/pwr", "/battery_percent", "/battery")

NotifyFn = Callable[[bytes], None]


def _bootstrap_ros_python_path() -> None:
    extra = [
        "/opt/ros/noetic/lib/python3/dist-packages",
        os.path.expanduser("~/sim2real/install/lib/python3/dist-packages"),
        os.path.expanduser("~/sim2real/devel/lib/python3/dist-packages"),
    ]
    for p in extra:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def read_lan_ip() -> Optional[str]:
    """返回局域网 IPv4 四段，如 192.168.19.11。"""
    candidates = []
    try:
        out = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        for item in out:
            ip = item[4][0]
            if not ip.startswith("127.") and ip.count(".") == 3:
                candidates.append(ip)
    except OSError:
        pass

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            candidates.insert(0, ip)
    except OSError:
        pass

    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4 and (parts[0] == "192" or parts[0] == "10"):
            return ip
    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4:
            return ip
    return None


# 兼容旧调用名
read_lan_ip_suffix = read_lan_ip


def read_battery_percent() -> Optional[int]:
    for path in sorted(glob.glob("/sys/class/power_supply/*/capacity")):
        try:
            with open(path, "r", encoding="ascii") as f:
                val = int(f.read().strip())
            if 0 <= val <= 100:
                return val
        except (OSError, ValueError):
            continue
    return None


def _normalize_pwr(pct: int) -> int:
    return max(0, min(100, int(pct)))


def _format_pwr(pct: int) -> str:
    """实际电量，如 pwr:83"""
    return f"pwr:{_normalize_pwr(pct)}"


class BleStatusTelemetry:
    def __init__(
        self,
        notify: NotifyFn,
        motor_power_fn: Optional[MotorPowerFn] = None,
        battery_fn: Optional[BatteryFn] = None,
        on_battery_pct: Optional[OnBatteryFn] = None,
    ) -> None:
        self._notify = notify
        self._motor_power_fn = motor_power_fn
        self._battery_fn = battery_fn
        self._on_battery_pct_fn = on_battery_pct
        self._stop = threading.Event()
        self._subscribed = threading.Event()
        self._lock = threading.Lock()
        self._last_ip: Optional[str] = None
        self._last_pwr_sent: Optional[int] = None
        self._last_mp_sent: Optional[str] = None
        self._last_fsm: Optional[int] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._mp_burst_gen = 0
        self._pwr_retry_gen = 0
        self._ros_battery_pct: Optional[int] = None

    def start(self) -> None:
        if self._poll_thread is not None:
            return
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        self._ros_thread = threading.Thread(target=self._ros_loop, daemon=True)
        self._ros_thread.start()
        log_info("状态遥测已启动（IP / pwr / mp / fsm → FFE2）")

    def stop(self) -> None:
        self._stop.set()

    def on_subscribed(self) -> None:
        """FFE2 订阅成功 → 立即推 IP/pwr/fsm；电量未就绪则短时重试；5 秒后连发 3 次 mp。"""
        self._subscribed.set()
        with self._lock:
            self._last_pwr_sent = None
            self._pwr_retry_gen += 1
            pwr_gen = self._pwr_retry_gen
            self._mp_burst_gen += 1
            mp_gen = self._mp_burst_gen
        self._push_snapshot()
        self._send_mp_state(force=True)
        threading.Thread(
            target=self._pwr_retry_loop, args=(pwr_gen,), daemon=True
        ).start()
        threading.Thread(target=self._mp_burst_loop, args=(mp_gen,), daemon=True).start()

    def on_unsubscribed(self) -> None:
        self._subscribed.clear()
        with self._lock:
            self._mp_burst_gen += 1
            self._pwr_retry_gen += 1

    def _mp_burst_loop(self, gen: int) -> None:
        time.sleep(MP_BURST_DELAY_SEC)
        for i in range(MP_BURST_COUNT):
            if self._stop.is_set() or not self._subscribed.is_set():
                return
            with self._lock:
                if gen != self._mp_burst_gen:
                    return
            self._send_mp_state(force=True)
            if i + 1 < MP_BURST_COUNT:
                time.sleep(MP_BURST_GAP_SEC)

    def _pwr_retry_loop(self, gen: int) -> None:
        """ROS /pwr 可能晚于 FFE2 订阅到达，短时重试直到发出 pwr。"""
        deadline = time.monotonic() + PWR_RETRY_MAX_SEC
        while time.monotonic() < deadline:
            if self._stop.is_set() or not self._subscribed.is_set():
                return
            with self._lock:
                if gen != self._pwr_retry_gen:
                    return
                if self._last_pwr_sent is not None:
                    return
            pct = self._read_pwr_source()
            if pct is None:
                pct = self._poll_ros_battery_once()
            if pct is not None:
                self._maybe_send_pwr(pct, force=True)
                return
            time.sleep(PWR_RETRY_INTERVAL_SEC)
        log_warn("握手后仍未读到电量（/battery_level 无数据）")

    def _poll_ros_battery_once(self) -> Optional[int]:
        """主动拉取一次 ROS 电量（订阅回调尚未到时）。"""
        _bootstrap_ros_python_path()
        try:
            import rospy
            from std_msgs.msg import Float32, Int32, UInt8
        except ImportError:
            return None
        if not rospy.core.is_initialized():
            try:
                rospy.init_node("ble_status_telemetry_poll", anonymous=True, disable_signals=True)
            except Exception:
                return None
        try:
            msg = rospy.wait_for_message("/battery_level", UInt8, timeout=0.5)
            return _normalize_pwr(int(msg.data))
        except Exception:
            pass
        for topic in BATTERY_FLOAT_TOPICS:
            for msg_type in (Float32, Int32):
                try:
                    msg = rospy.wait_for_message(topic, msg_type, timeout=0.08)
                    return _normalize_pwr(int(msg.data))
                except Exception:
                    continue
        return None

    def _push_snapshot(self) -> None:
        ip = read_lan_ip()
        if ip:
            self._send_ip(ip, force=True)
        pct = self._read_pwr_source()
        if pct is None:
            pct = self._poll_ros_battery_once()
        if pct is not None:
            self._maybe_send_pwr(pct, force=True)
        with self._lock:
            fsm = self._last_fsm
        if fsm is not None:
            self._send_fsm(fsm, force=True)

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            ip = read_lan_ip()
            if ip:
                self._send_ip(ip)
            pct = self._read_pwr_source()
            if pct is not None:
                self._maybe_send_pwr(pct)
            time.sleep(PWR_POLL_SEC)

    def _read_pwr_source(self) -> Optional[int]:
        if self._battery_fn is not None:
            try:
                pct = self._battery_fn()
            except Exception:
                pct = None
            if pct is not None:
                return pct
        with self._lock:
            ros_pct = self._ros_battery_pct
        if ros_pct is not None:
            return ros_pct
        return read_battery_percent()

    def _on_battery_pct(self, pct: int) -> None:
        pct = _normalize_pwr(pct)
        with self._lock:
            self._ros_battery_pct = pct
            need_send = self._subscribed.is_set() and self._last_pwr_sent is None
        if self._on_battery_pct_fn is not None:
            try:
                self._on_battery_pct_fn(pct)
            except Exception:
                pass
        if need_send:
            self._maybe_send_pwr(pct, force=True)
        elif self._subscribed.is_set():
            self._maybe_send_pwr(pct)

    def _ros_loop(self) -> None:
        _bootstrap_ros_python_path()
        try:
            import rospy
            from std_msgs.msg import Float32, Int32, UInt8
        except ImportError:
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("ble_status_telemetry", anonymous=True, disable_signals=True)
        except Exception:
            return

        def on_fsm(msg: Int32) -> None:
            state = int(msg.data)
            with self._lock:
                prev = self._last_fsm
                self._last_fsm = state
            if prev != state:
                self._send_fsm(state)

        def on_float_battery(msg: Float32) -> None:
            self._on_battery_pct(int(msg.data))

        def on_int_battery(msg: Int32) -> None:
            self._on_battery_pct(int(msg.data))

        def on_uint8_battery(msg: UInt8) -> None:
            self._on_battery_pct(int(msg.data))

        rospy.Subscriber("/fsm_state", Int32, on_fsm, queue_size=1)
        rospy.Subscriber("/battery_level", UInt8, on_uint8_battery, queue_size=1)
        for topic in BATTERY_FLOAT_TOPICS:
            if topic == "/battery_level":
                continue
            rospy.Subscriber(topic, Float32, on_float_battery, queue_size=1)
        for topic in BATTERY_INT_TOPICS:
            rospy.Subscriber(topic, Int32, on_int_battery, queue_size=1)

        try:
            from sensor_msgs.msg import BatteryState

            def on_battery_state(msg: BatteryState) -> None:
                pct = int(msg.percentage * 100) if msg.percentage <= 1.0 else int(msg.percentage)
                self._on_battery_pct(pct)

            rospy.Subscriber("/battery_state", BatteryState, on_battery_state, queue_size=1)
        except ImportError:
            pass

        try:
            from livelybot_power.msg import Power_switch

            def on_power_switch(msg: Power_switch) -> None:
                wire = "ON" if msg.power_switch else "OFF"
                self._send_mp_wire(wire)

            rospy.Subscriber(
                "/power_switch_state", Power_switch, on_power_switch, queue_size=1
            )
        except ImportError:
            pass

        rospy.spin()

    def _tx(self, text: str, repeat: int = 1) -> bool:
        if not self._subscribed.is_set():
            return False
        payload = text.encode("utf-8", errors="replace")[:180]
        for i in range(repeat):
            try:
                self._notify(payload)
                log_tx(text)
            except Exception as e:
                log_warn(f"notify 失败: {e}")
                return False
            if i + 1 < repeat:
                time.sleep(FSM_REPEAT_GAP_SEC)
        return True

    def _send_ip(self, ip: str, force: bool = False) -> None:
        with self._lock:
            if not force and ip == self._last_ip:
                return
            self._last_ip = ip
        self._tx(f"IP:{ip}")

    def _maybe_send_pwr(self, raw_pct: int, force: bool = False) -> None:
        if not self._subscribed.is_set():
            return
        pct = _normalize_pwr(raw_pct)
        with self._lock:
            prev = self._last_pwr_sent
            should_send = force or prev is None or pct < prev
        if not should_send:
            return
        if self._tx(_format_pwr(pct)):
            with self._lock:
                self._last_pwr_sent = pct

    def _send_mp_state(self, force: bool = False) -> None:
        if self._motor_power_fn is None:
            return
        wire = self._motor_power_fn()
        if wire not in ("ON", "OFF"):
            return
        self._send_mp_wire(wire, force=force)

    def _send_mp_wire(self, wire: str, force: bool = False) -> None:
        if wire not in ("ON", "OFF"):
            return
        with self._lock:
            if not force and wire == self._last_mp_sent:
                return
            self._last_mp_sent = wire
        self._tx(f"mp:{wire}")

    def push_mp_state(self, wire: str, force: bool = True) -> None:
        """MP 指令后立即推送 mp:ON/OFF，供小程序自动站立判断。"""
        self._send_mp_wire(wire, force=force)

    def _send_fsm(self, state: int, force: bool = False) -> None:
        repeat = FSM_REPEAT if not force else 1
        self._tx(f"fsm:{state}", repeat=repeat)
