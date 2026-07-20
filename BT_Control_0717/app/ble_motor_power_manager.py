#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE MP ON/OFF → /power_switch_control（+ /com_power_control 兜底）。"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

POWER_TOPIC = "/power_switch_control"
COM_POWER_TOPIC = "/com_power_control"
# 上电后硬件 /power_switch_state 可能短暂仍为 OFF，勿立刻覆盖
HARDWARE_OFF_GRACE_SEC = 3.0
MP_PUBLISH_BURST = 2
MP_PUBLISH_GAP_SEC = 0.1
SUBSCRIBER_WAIT_SEC = 1.0

LogFn = Callable[[str], None]
StateFn = Callable[[bool], None]


class MotorPowerController:
    """MP ON=接通电机供电，MP OFF=断开电机供电。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._pub = None
        self._com_pub = None
        self._pending: Optional[str] = None
        self._motor_on: Optional[bool] = None
        # 最近一次 BLE/软件指令意图；None=尚未指令，以硬件为准
        self._desired: Optional[bool] = None
        self._grace_until = 0.0
        self._on_state_changed: Optional[StateFn] = None

    def set_state_listener(self, fn: Optional[StateFn]) -> None:
        self._on_state_changed = fn

    def attach_publisher(self, pub, com_pub=None) -> None:
        self._pub = pub
        self._com_pub = com_pub
        self._log(f"[MP] 已发布 {POWER_TOPIC}" + (f" + {COM_POWER_TOPIC}" if com_pub else ""))

    def update_state_from_hardware(self, motor_on: bool) -> None:
        """订阅 /power_switch_state 回调（受最近指令意图过滤，避免误报）。"""
        now = time.monotonic()
        with self._lock:
            desired = self._desired
            # MP OFF 后功率板话题常仍短暂残留 ON → 勿当成真上电（会误推 mp:ON / 语音）
            if motor_on and desired is False:
                return
            # MP ON 后短暂忽略硬件 OFF
            if (
                not motor_on
                and desired is True
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

    def apply_immediate(self, action: str, soft: bool = False) -> bool:
        """已就绪则立刻下发，否则入队等待 tick。soft=True 时 OFF 少发且不碰 com_power。"""
        cmd = action.strip().upper()
        if cmd not in ("ON", "OFF"):
            return False
        with self._lock:
            pub = self._pub
            if pub is None:
                self._pending = cmd
                return False
        self._apply(cmd, soft=soft)
        return True

    def tick(self) -> None:
        with self._lock:
            action = self._pending
            self._pending = None
        if action is not None:
            self._apply(action)

    def _wait_subscribers(self) -> int:
        """等待 power_node 等订阅者连上；返回连接数。"""
        deadline = time.monotonic() + SUBSCRIBER_WAIT_SEC
        n = 0
        while time.monotonic() < deadline:
            n = 0
            if self._pub is not None:
                try:
                    n = max(n, int(self._pub.get_num_connections()))
                except Exception:
                    pass
            if self._com_pub is not None:
                try:
                    n = max(n, int(self._com_pub.get_num_connections()))
                except Exception:
                    pass
            if n > 0:
                return n
            time.sleep(0.05)
        return n

    def _apply(self, action: str, soft: bool = False) -> None:
        if self._pub is None:
            self._log("[MP] 未就绪，忽略指令")
            return
        try:
            from livelybot_power.msg import Power_switch
            from std_msgs.msg import UInt8
        except ImportError:
            self._log("[MP] 未找到 livelybot_power，无法控制电机电源")
            return

        on = action == "ON"
        conns = self._wait_subscribers()
        if conns <= 0:
            self._log(
                f"[MP][warn] {POWER_TOPIC} 无订阅者（power_node 可能未运行），"
                f"仍尝试下发 {action}"
            )

        msg = Power_switch()
        msg.control_switch = 1
        msg.power_switch = 1 if on else 0
        com = UInt8()
        com.data = 1 if on else 0

        # soft OFF：单次主通道，避免连发/双通道冲击主节点
        bursts = 1 if (soft and not on) else MP_PUBLISH_BURST
        use_com = not (soft and not on)

        for i in range(bursts):
            try:
                self._pub.publish(msg)
            except Exception as e:
                self._log(f"[MP] publish 失败: {e}")
                break
            if use_com and self._com_pub is not None:
                try:
                    self._com_pub.publish(com)
                except Exception:
                    pass
            if i + 1 < bursts:
                time.sleep(MP_PUBLISH_GAP_SEC)

        with self._lock:
            self._desired = on
            if on:
                self._grace_until = time.monotonic() + HARDWARE_OFF_GRACE_SEC
            else:
                self._grace_until = 0.0
            prev = self._motor_on
            self._motor_on = on
        self._log(
            f"[MP] 电机电源已{'开启' if on else '关闭'} ({action})"
            f" | subscribers={conns}"
            + (" | soft" if soft else "")
        )
        # 仅状态变化时回调，避免重复播「电机上电」
        if prev != on:
            self._notify_state(on)

    def _notify_state(self, motor_on: bool) -> None:
        if self._on_state_changed is not None:
            try:
                self._on_state_changed(motor_on)
            except Exception as exc:
                self._log(f"[MP] 状态回调失败: {exc}")
