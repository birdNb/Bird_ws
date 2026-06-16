#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE MP ON/OFF → /power_switch_control（livelybot 功率板电机供电）。"""

from __future__ import annotations

import threading
from typing import Callable, Optional

POWER_TOPIC = "/power_switch_control"

LogFn = Callable[[str], None]


class MotorPowerController:
    """MP ON=接通电机供电，MP OFF=断开电机供电。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._pub = None
        self._pending: Optional[str] = None
        self._motor_on: Optional[bool] = None

    def attach_publisher(self, pub) -> None:
        self._pub = pub
        self._log(f"[MP] 已发布 {POWER_TOPIC}")

    def enqueue(self, action: str) -> bool:
        cmd = action.strip().upper()
        if cmd not in ("ON", "OFF"):
            return False
        with self._lock:
            self._pending = cmd
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
        self._motor_on = on
        self._log(f"[MP] 电机电源已{'开启' if on else '关闭'} ({action})")
