#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手势 1 撒娇扭腰：waist_yaw ±45° 来回 2 次，并触发 cheer(挥双手)。

仅发布 waist_yaw_joint，不控 head，避免与脸部跟踪冲突。
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Optional, Set

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "common"))
from paths import setup_paths  # noqa: E402

setup_paths(motion=True)

from ros_setup import require_sim2real_msg

require_sim2real_msg()

import rospy
from sensor_msgs.msg import JointState
from sim2real_msg.msg import Joy

from ros_control import ABSOLUTE_TOPIC, FsmStateMonitor

# ----- 时序（与 waist_coquette_player 中 COQUETTE_BUSY_SEC 对齐） -----
SWAY_AMPLITUDE_DEG = 45.0
SWAY_CYCLES = 2
RAMP_SEC = 1.0          # 每段转腰时长(秒)，越大越慢
SWAY_HOLD_SEC = 0.1    # 到达左右端点后停留(秒)
ACTION_DURATION_SEC = 5.0
ARM_RESET_WAIT_SEC = 0.5
TRIGGER_PULSE_SEC = 0.5
JOY_MSG_TOPIC = "/joy_msg"
JOY_PUBLISH_HZ = 20
PUBLISH_HZ = 50

WAIST_YAW_JOINT = "waist_yaw_joint"
CHEER_KEYS = {"rt", "a"}

SWAY_SEGMENT_COUNT = SWAY_CYCLES * 2 + 1  # 每半周一段，最后回中
SWAY_HOLD_COUNT = SWAY_CYCLES * 2         # 左右端点停留次数
ACTION_TOTAL_SEC = (
    SWAY_SEGMENT_COUNT * RAMP_SEC
    + SWAY_HOLD_COUNT * SWAY_HOLD_SEC
    + ACTION_DURATION_SEC
    + ARM_RESET_WAIT_SEC
)


def _parse_keys(combo: str) -> Set[str]:
    return {p.strip().lower() for p in combo.split("+") if p.strip()}


def _joy_from_keys(keys: Set[str], pressed: bool) -> Joy:
    msg = Joy()
    field_map = {
        "a": "a", "b": "b", "x": "x", "y": "y",
        "lb": "lb", "rb": "rb", "back": "back", "start": "start",
        "lt": "lt", "rt": "rt",
        "l": "L", "r": "R", "center": "center",
    }
    press_val = 1.0 if pressed else 0.0
    trig_press, trig_release = -1.0, 1.0
    for key in keys:
        attr = field_map.get(key)
        if attr is None:
            continue
        val = (
            (trig_press if pressed else trig_release)
            if key in ("lt", "rt")
            else press_val
        )
        setattr(msg, attr, val)
    return msg


def _pulse_joy(
    pub: rospy.Publisher,
    keys: Set[str],
    *,
    duration_sec: float,
    dry_run: bool,
    abort_evt: Optional[threading.Event],
) -> None:
    if dry_run or not keys:
        time.sleep(min(duration_sec, 0.1))
        return
    press = _joy_from_keys(keys, True)
    release = _joy_from_keys(keys, False)
    interval = 1.0 / max(JOY_PUBLISH_HZ, 1)
    end_t = time.time() + max(0.05, duration_sec)
    while time.time() < end_t and not rospy.is_shutdown():
        if abort_evt is not None and abort_evt.is_set():
            return
        pub.publish(press)
        time.sleep(interval)
    for _ in range(3):
        if rospy.is_shutdown() or (
            abort_evt is not None and abort_evt.is_set()
        ):
            return
        pub.publish(release)
        time.sleep(interval)


def _quintic_alpha(t: float, duration: float) -> float:
    if duration <= 0:
        return 1.0
    x = min(1.0, max(0.0, t / duration))
    return 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5


def _ramp_position(
    pub: rospy.Publisher,
    start_rad: float,
    goal_rad: float,
    *,
    duration_sec: float,
    dry_run: bool,
    abort_evt,
) -> None:
    msg = JointState()
    msg.name = [WAIST_YAW_JOINT]
    msg.velocity = []
    msg.effort = []
    interval = 1.0 / max(PUBLISH_HZ, 1)
    t0 = time.time()
    while not rospy.is_shutdown():
        if abort_evt is not None and abort_evt.is_set():
            return
        elapsed = time.time() - t0
        alpha = _quintic_alpha(elapsed, duration_sec)
        pos = start_rad + (goal_rad - start_rad) * alpha
        if not dry_run:
            msg.header.stamp = rospy.Time.now()
            msg.position = [pos]
            pub.publish(msg)
        if elapsed >= duration_sec:
            break
        time.sleep(interval)


def _hold_position(
    pub: rospy.Publisher,
    pos_rad: float,
    *,
    duration_sec: float,
    dry_run: bool,
    abort_evt,
) -> None:
    if duration_sec <= 0:
        return
    msg = JointState()
    msg.name = [WAIST_YAW_JOINT]
    msg.velocity = []
    msg.effort = []
    interval = 1.0 / max(PUBLISH_HZ, 1)
    end_t = time.time() + duration_sec
    while time.time() < end_t and not rospy.is_shutdown():
        if abort_evt is not None and abort_evt.is_set():
            return
        if not dry_run:
            msg.header.stamp = rospy.Time.now()
            msg.position = [pos_rad]
            pub.publish(msg)
        time.sleep(interval)


def _wait_fsm(skip: bool) -> None:
    if skip:
        return
    fsm = FsmStateMonitor()
    fsm.wait_for_exec_default(timeout=30.0)


def run_coquette_action(
    *,
    dry_run: bool = False,
    abort_evt=None,
    skip_fsm_wait: bool = False,
) -> None:
    """执行撒娇：挥双手(cheer) + 腰部 ±45° 来回 2 次 + 回中。"""
    _wait_fsm(skip_fsm_wait)
    if abort_evt is not None and abort_evt.is_set():
        return

    amp = math.radians(SWAY_AMPLITUDE_DEG)
    targets = []
    sign = 1.0
    for _ in range(SWAY_CYCLES):
        targets.append(sign * amp)
        targets.append(-sign * amp)
        sign *= -1.0
    targets.append(0.0)

    waist_pub = rospy.Publisher(ABSOLUTE_TOPIC, JointState, queue_size=10)
    joy_pub = rospy.Publisher(JOY_MSG_TOPIC, Joy, queue_size=1)
    if not dry_run:
        t0 = time.time()
        while (
            waist_pub.get_num_connections() == 0
            and not rospy.is_shutdown()
            and time.time() - t0 < 3.0
        ):
            time.sleep(0.05)

    cheer_keys = _parse_keys("rt+a")
    _pulse_joy(
        joy_pub, cheer_keys,
        duration_sec=TRIGGER_PULSE_SEC,
        dry_run=dry_run,
        abort_evt=abort_evt,
    )

    cur = 0.0
    sway_t0 = time.time()
    last_idx = len(targets) - 1
    for idx, goal in enumerate(targets):
        if abort_evt is not None and abort_evt.is_set():
            break
        _ramp_position(
            waist_pub, cur, goal,
            duration_sec=RAMP_SEC,
            dry_run=dry_run,
            abort_evt=abort_evt,
        )
        cur = goal
        if (
            idx < last_idx
            and abs(goal) > 1e-6
            and SWAY_HOLD_SEC > 0
        ):
            _hold_position(
                waist_pub, cur,
                duration_sec=SWAY_HOLD_SEC,
                dry_run=dry_run,
                abort_evt=abort_evt,
            )

    cheer_remain = max(
        0.0,
        ACTION_DURATION_SEC - (time.time() - sway_t0),
    )
    end_cheer = time.time() + cheer_remain
    while time.time() < end_cheer and not rospy.is_shutdown():
        if abort_evt is not None and abort_evt.is_set():
            break
        time.sleep(0.05)

    if abort_evt is None or not abort_evt.is_set():
        _pulse_joy(
            joy_pub, cheer_keys,
            duration_sec=TRIGGER_PULSE_SEC,
            dry_run=dry_run,
            abort_evt=abort_evt,
        )

    if cur != 0.0 and (abort_evt is None or not abort_evt.is_set()):
        _ramp_position(
            waist_pub, cur, 0.0,
            duration_sec=RAMP_SEC,
            dry_run=dry_run,
            abort_evt=abort_evt,
        )

    if ARM_RESET_WAIT_SEC > 0:
        time.sleep(ARM_RESET_WAIT_SEC)
