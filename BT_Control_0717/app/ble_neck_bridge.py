#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""脖子控制：FFE1 P{n}Y{m} / neck0 → /pi_plus_absolute。"""

from __future__ import annotations

import math
import re
import threading
from typing import Callable, Optional, Tuple

NECK_TOPIC = "/pi_plus_absolute"
HEAD_YAW_JOINT = "head_yaw_joint"
HEAD_PITCH_JOINT = "head_pitch_joint"
NECK_STEP_DEG = 10.0
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0
PITCH_DOWN_DEG = 60.0
HOME_RATE_DEG_PER_SEC = 45.0
TICK_HZ = 20.0

NECK_OFFSET_RE = re.compile(r"^[Pp]([+-]?\d+)Y([+-]?\d+)$")
NECK_CENTER_RE = re.compile(r"^neck0$", re.IGNORECASE)

LogFn = Callable[[str], None]


def parse_neck_command(text: str) -> Optional[Tuple[str, int, int]]:
    raw = text.strip()
    if NECK_CENTER_RE.match(raw):
        return ("neck0", 0, 0)
    m = NECK_OFFSET_RE.match(raw)
    if not m:
        return None
    return (raw, int(m.group(1)), int(m.group(2)))


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _clamp_pitch_deg(deg: float) -> float:
    return max(PITCH_UP_DEG, min(PITCH_DOWN_DEG, deg))


def _clamp_yaw_deg(deg: float) -> float:
    return max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, deg))


def _step_toward_zero(val: float, step: float) -> float:
    if abs(val) <= 1e-4:
        return 0.0
    return val - math.copysign(min(step, abs(val)), val)


class NeckController:
    """P+步=抬头/右转 10°，P-步=低头/左转 10°；neck0 / P0Y0 平滑回中。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._pub = None
        self._pending: Optional[str] = None
        self._homing = False

    def attach_publisher(self, pub) -> None:
        self._pub = pub
        self._log(f"[neck] 已发布 {NECK_TOPIC}（步进 {NECK_STEP_DEG}°）")

    def enqueue(self, text: str) -> bool:
        """仅缓存指令；实际 publish 在 ROS 线程 tick() 中执行。"""
        if parse_neck_command(text) is None:
            return False
        with self._lock:
            self._pending = text.strip()
        return True

    def tick(self) -> None:
        with self._lock:
            text = self._pending
            self._pending = None
        if text is not None:
            self.handle(text)
        self._tick_homing()

    def handle(self, text: str) -> bool:
        if self._pub is None:
            self._log("[neck] 未就绪，忽略指令")
            return False
        parsed = parse_neck_command(text)
        if parsed is None:
            return False
        wire, p_steps, y_steps = parsed
        if wire == "neck0" or (p_steps == 0 and y_steps == 0):
            self._start_smooth_home()
            self._log(f"[neck] 回中 {wire} → yaw=0 pitch=0（平滑）")
            return True
        with self._lock:
            self._homing = False
            # P+ = 抬头（pitch 减小，因 PITCH_UP_DEG 为负值）
            # Y+ = 右转（yaw 减小，因坐标系符号约定）
            self._pitch_deg = _clamp_pitch_deg(
                self._pitch_deg - p_steps * NECK_STEP_DEG
            )
            self._yaw_deg = _clamp_yaw_deg(self._yaw_deg - y_steps * NECK_STEP_DEG)
            yaw, pitch = self._yaw_deg, self._pitch_deg
        self._publish(yaw, pitch)
        self._log(
            f"[neck] {wire} → pitch={pitch:+.1f}° yaw={yaw:+.1f}° "
            f"(P{p_steps:+d} Y{y_steps:+d} ×{NECK_STEP_DEG:.0f}°)"
        )
        return True

    def _start_smooth_home(self) -> None:
        with self._lock:
            self._homing = True

    def _tick_homing(self) -> None:
        if self._pub is None:
            return
        with self._lock:
            if not self._homing:
                return
            step = HOME_RATE_DEG_PER_SEC / TICK_HZ
            self._yaw_deg = _step_toward_zero(self._yaw_deg, step)
            self._pitch_deg = _step_toward_zero(self._pitch_deg, step)
            yaw, pitch = self._yaw_deg, self._pitch_deg
            if abs(yaw) < 1e-4 and abs(pitch) < 1e-4:
                self._yaw_deg = 0.0
                self._pitch_deg = 0.0
                self._homing = False
                yaw, pitch = 0.0, 0.0
        self._publish(yaw, pitch)

    def _set_center(self) -> None:
        with self._lock:
            self._yaw_deg = 0.0
            self._pitch_deg = 0.0
        self._publish(0.0, 0.0)

    def _publish(self, yaw_deg: float, pitch_deg: float) -> None:
        if self._pub is None:
            return
        from sensor_msgs.msg import JointState
        import rospy

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [_deg2rad(yaw_deg), _deg2rad(pitch_deg)]
        self._pub.publish(msg)
