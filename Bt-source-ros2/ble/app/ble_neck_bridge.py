#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脖子控制（对齐 BLE_PROTOCOL.md §1.4）：

  P{n}Y{m}  pitch/yaw 步进（整数，可带 +/-），每步 10°
  neck0     平滑回中
  P0Y0      同 neck0，平滑回中

方向（协议）：
  P+ 往上 / P- 往下
  Y+ 往右 / Y- 往左

发布 /pi_plus_absolute：head_yaw_joint、head_pitch_joint（弧度）。
内部用「上/右为负角度」的关节约定，使协议方向与真机一致。
"""

from __future__ import annotations

import math
import os
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
NECK_STATE_FILE = "/tmp/locate_face_neck.state"

# 协议：P1Y0 / P-1Y0 / P0Y1 / neck0；Y 大小写均可
NECK_OFFSET_RE = re.compile(r"^[Pp]([+-]?\d+)[Yy]([+-]?\d+)$")
NECK_CENTER_RE = re.compile(r"^neck0$", re.IGNORECASE)

LogFn = Callable[[str], None]


def parse_neck_command(text: str) -> Optional[Tuple[str, int, int]]:
    raw = text.strip()
    if NECK_CENTER_RE.match(raw):
        return ("neck0", 0, 0)
    m = NECK_OFFSET_RE.match(raw)
    if not m:
        return None
    p_steps = int(m.group(1))
    y_steps = int(m.group(2))
    wire = f"P{p_steps}Y{y_steps}"
    return (wire, p_steps, y_steps)


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
    """协议：P+抬头 / P-低头 / Y+右转 / Y-左转，每步 10°；neck0/P0Y0 平滑回中。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._pub = None
        self._clock = None
        self._pending: Optional[str] = None
        self._homing = False
        self._load_state()

    def _load_state(self) -> None:
        try:
            with open(NECK_STATE_FILE, "r", encoding="ascii") as f:
                parts = f.read().strip().split()
            if len(parts) >= 2:
                self._yaw_deg = _clamp_yaw_deg(float(parts[0]))
                self._pitch_deg = _clamp_pitch_deg(float(parts[1]))
        except (OSError, ValueError):
            pass

    def _save_state(self, yaw_deg: float, pitch_deg: float) -> None:
        try:
            tmp = f"{NECK_STATE_FILE}.tmp"
            with open(tmp, "w", encoding="ascii") as f:
                f.write(f"{yaw_deg:.4f} {pitch_deg:.4f}\n")
            os.replace(tmp, NECK_STATE_FILE)
        except OSError:
            pass

    def attach_publisher(self, pub, clock=None) -> None:
        self._pub = pub
        self._clock = clock
        self._log(
            f"[neck] 已发布 {NECK_TOPIC}（步进 {NECK_STEP_DEG:.0f}°｜"
            f"P+上/P-下｜Y+右/Y-左｜neck0/P0Y0 回中）"
        )

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
        # neck0 / P0Y0 → 平滑回中（协议回中）
        if wire == "neck0" or (p_steps == 0 and y_steps == 0):
            self._start_smooth_home()
            self._log(f"[neck] 回中 {wire} → yaw=0 pitch=0（平滑）")
            return True
        with self._lock:
            self._homing = False
            # 协议 P+往上、Y+往右；关节约定上/右为负角 → 用减法累加步进
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
            self._homing = False
        self._publish(0.0, 0.0)

    def _publish(self, yaw_deg: float, pitch_deg: float) -> None:
        if self._pub is None:
            return
        from sensor_msgs.msg import JointState
        from builtin_interfaces.msg import Time

        msg = JointState()
        if self._clock is not None:
            msg.header.stamp = self._clock.now().to_msg()
        else:
            msg.header.stamp = Time()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [_deg2rad(yaw_deg), _deg2rad(pitch_deg)]
        self._pub.publish(msg)
        self._save_state(yaw_deg, pitch_deg)
