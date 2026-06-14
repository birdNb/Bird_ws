#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人 → 小程序 状态遥测（FFE2 notify）。

  IP:19.11     局域网 IP 后两段
  pwr:50       电量 5% 步进，下降时推送
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

from ble_log import log_info, log_tx, log_warn

PWR_STEP = 5
IP_POLL_SEC = 15.0
PWR_POLL_SEC = 5.0
FSM_REPEAT = 3
FSM_REPEAT_GAP_SEC = 0.05

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


def read_lan_ip_suffix() -> Optional[str]:
    """192.168.19.11 → 19.11"""
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
            return f"{parts[2]}.{parts[3]}"
    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4:
            return f"{parts[2]}.{parts[3]}"
    return None


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


def quantize_pwr(pct: int) -> int:
    return max(0, min(100, (pct // PWR_STEP) * PWR_STEP))


class BleStatusTelemetry:
    def __init__(self, notify: NotifyFn) -> None:
        self._notify = notify
        self._stop = threading.Event()
        self._subscribed = threading.Event()
        self._lock = threading.Lock()
        self._last_ip: Optional[str] = None
        self._last_pwr_sent: Optional[int] = None
        self._last_fsm: Optional[int] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._ros_battery_pct: Optional[int] = None

    def start(self) -> None:
        if self._poll_thread is not None:
            return
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        self._ros_thread = threading.Thread(target=self._ros_loop, daemon=True)
        self._ros_thread.start()
        log_info("状态遥测已启动（IP / pwr / fsm → FFE2）")

    def stop(self) -> None:
        self._stop.set()

    def on_subscribed(self) -> None:
        self._subscribed.set()
        self._push_snapshot()

    def on_unsubscribed(self) -> None:
        self._subscribed.clear()

    def _push_snapshot(self) -> None:
        ip = read_lan_ip_suffix()
        if ip:
            self._send_ip(ip, force=True)
        pct = self._read_pwr_source()
        if pct is not None:
            with self._lock:
                self._last_pwr_sent = None
            self._maybe_send_pwr(pct, force=True)
        with self._lock:
            fsm = self._last_fsm
        if fsm is not None:
            self._send_fsm(fsm, force=True)

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            ip = read_lan_ip_suffix()
            if ip:
                self._send_ip(ip)
            pct = self._read_pwr_source()
            if pct is not None:
                self._maybe_send_pwr(pct)
            time.sleep(PWR_POLL_SEC)

    def _read_pwr_source(self) -> Optional[int]:
        with self._lock:
            ros_pct = self._ros_battery_pct
        if ros_pct is not None:
            return ros_pct
        return read_battery_percent()

    def _ros_loop(self) -> None:
        _bootstrap_ros_python_path()
        try:
            import rospy
            from std_msgs.msg import Float32, Int32
        except ImportError:
            return
        try:
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
            with self._lock:
                self._ros_battery_pct = max(0, min(100, int(msg.data)))

        rospy.Subscriber("/fsm_state", Int32, on_fsm, queue_size=1)
        for topic in ("/pwr", "/battery_percent", "/battery"):
            rospy.Subscriber(topic, Float32, on_float_battery, queue_size=1)

        try:
            from sensor_msgs.msg import BatteryState

            def on_battery_state(msg: BatteryState) -> None:
                pct = int(msg.percentage * 100) if msg.percentage <= 1.0 else int(msg.percentage)
                with self._lock:
                    self._ros_battery_pct = max(0, min(100, pct))

            rospy.Subscriber("/battery_state", BatteryState, on_battery_state, queue_size=1)
        except ImportError:
            pass

        rospy.spin()

    def _tx(self, text: str, repeat: int = 1) -> None:
        if not self._subscribed.is_set():
            return
        payload = text.encode("utf-8", errors="replace")[:180]
        for i in range(repeat):
            try:
                self._notify(payload)
                log_tx(text)
            except Exception as e:
                log_warn(f"notify 失败: {e}")
                return
            if i + 1 < repeat:
                time.sleep(FSM_REPEAT_GAP_SEC)

    def _send_ip(self, suffix: str, force: bool = False) -> None:
        with self._lock:
            if not force and suffix == self._last_ip:
                return
            self._last_ip = suffix
        self._tx(f"IP:{suffix}")

    def _maybe_send_pwr(self, raw_pct: int, force: bool = False) -> None:
        q = quantize_pwr(raw_pct)
        with self._lock:
            prev = self._last_pwr_sent
            if force:
                self._last_pwr_sent = q
                self._tx(f"pwr:{q}")
                return
            if prev is None:
                self._last_pwr_sent = q
                self._tx(f"pwr:{q}")
                return
            if q < prev:
                self._last_pwr_sent = q
                self._tx(f"pwr:{q}")

    def _send_fsm(self, state: int, force: bool = False) -> None:
        repeat = FSM_REPEAT if not force else 1
        self._tx(f"fsm:{state}", repeat=repeat)
