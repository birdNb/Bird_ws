#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE MP ON/OFF → /power_switch_control（livelybot 功率板电机供电）。"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

POWER_TOPIC = "/power_switch_control"
# 上电后硬件 /power_switch_state 可能短暂仍为 OFF，勿立刻覆盖
HARDWARE_OFF_GRACE_SEC = 3.0

LogFn = Callable[[str], None]
StateFn = Callable[[bool], None]


class MotorPowerController:
    """MP ON=接通电机供电，MP OFF=断开电机供电。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._pub = None
        self._pending: Optional[str] = None
        self._motor_on: Optional[bool] = None
        self._grace_until = 0.0
        self._on_state_changed: Optional[StateFn] = None

    def set_state_listener(self, fn: Optional[StateFn]) -> None:
        self._on_state_changed = fn

    def attach_publisher(self, pub) -> None:
        self._pub = pub
        self._log(f"[MP] 已发布 {POWER_TOPIC}")

    def update_state_from_hardware(self, motor_on: bool) -> None:
        """订阅 /power_switch_state 回调。"""
        now = time.monotonic()
        with self._lock:
            if (
                not motor_on
                and self._motor_on
                and now < self._grace_until
            ):
                return
            if self._motor_on == motor_on:
                return
            self._motor_on = motor_on
        self._notify_state(motor_on)

    def update_state(self, motor_on: bool) -> None:
        with self._lock:
            if self._motor_on == motor_on:
                return
            self._motor_on = motor_on
        self._notify_state(motor_on)

    def get_state_wire(self) -> Optional[str]:
        with self._lock:
            if self._motor_on is None:
                return None
            return "ON" if self._motor_on else "OFF"

    def enqueue(self, action: str) -> bool:
        cmd = action.strip().upper()
        if cmd not in ("ON", "OFF"):
            return False
        with self._lock:
            self._pending = cmd
        return True

    def apply_immediate(self, action: str) -> bool:
        """已就绪则立刻下发，否则入队等待 tick。"""
        cmd = action.strip().upper()
        if cmd not in ("ON", "OFF"):
            return False
        with self._lock:
            pub = self._pub
            if pub is None:
                self._pending = cmd
                return False
        self._apply(cmd)
        return True

    def tick(self) -> None:
        with self._lock:
            action = self._pending
            self._pending = None
        if action is not None:
            self._apply(action)

    def _apply(self, action: str) -> None:
        if self._pub is None:
            self._log("[MP] 未就绪，忽略指令")
            return
        try:
            from livelybot_power.msg import Power_switch
        except ImportError:
            self._log("[MP] 未找到 livelybot_power，无法控制电机电源")
            return

        on = action == "ON"
        msg = Power_switch()
        msg.control_switch = 1
        msg.power_switch = 1 if on else 0
        self._pub.publish(msg)
        with self._lock:
            if on:
                self._grace_until = time.monotonic() + HARDWARE_OFF_GRACE_SEC
            else:
                self._grace_until = 0.0
            self._motor_on = on
        self._log(f"[MP] 电机电源已{'开启' if on else '关闭'} ({action})")
        self._notify_state(on)

    def _notify_state(self, motor_on: bool) -> None:
        if self._on_state_changed is not None:
            try:
                self._on_state_changed(motor_on)
            except Exception as exc:
                self._log(f"[MP] 状态回调失败: {exc}")
