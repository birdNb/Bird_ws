#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 /joy_msg 触发 waypoint 动作，通过 /action_config 触发 policy_change（踢球等）。"""

import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "common"))
from paths import setup_paths  # noqa: E402

setup_paths(motion=True)

from ros_setup import require_sim2real_msg

require_sim2real_msg()

import rospy
from sim2real_msg.msg import Joy
from std_msgs.msg import String

from gesture_actions import (
    ACTION_CONFIG_TOPIC,
    GESTURE_ACTION_LABELS,
    GESTURE_JOY_ACTION_SPECS,
    GESTURE_POLICY_ACTION_SPECS,
    format_action_trigger_line,
)

JOY_MSG_TOPIC = "/joy_msg"
JOY_PUBLISH_HZ = 20

ACTION_DURATION_SEC = 5.0
TRIGGER_PULSE_SEC = 0.5
ACTION_COOLDOWN_SEC = ACTION_DURATION_SEC + 1.0
BUTTON_PRESS = 1.0
BUTTON_RELEASE = 0.0
TRIGGER_PRESS = -1.0
TRIGGER_RELEASE = 1.0


def _parse_key_combo(combo: str) -> Set[str]:
    return {p.strip().lower() for p in combo.split("+") if p.strip()}


def _joy_key_value(key: str, pressed: bool) -> float:
    if key in ("lt", "rt"):
        return TRIGGER_PRESS if pressed else TRIGGER_RELEASE
    return BUTTON_PRESS if pressed else BUTTON_RELEASE


def _joy_from_keys(keys: Set[str], pressed: bool) -> Joy:
    msg = Joy()
    field_map = {
        "a": "a", "b": "b", "x": "x", "y": "y",
        "lb": "lb", "rb": "rb", "back": "back", "start": "start",
        "lt": "lt", "rt": "rt",
        "l": "L", "r": "R", "center": "center",
    }
    for key in keys:
        attr = field_map.get(key)
        if attr is None:
            rospy.logwarn("[gesture_action] 未知按键: %s", key)
            continue
        setattr(msg, attr, _joy_key_value(key, pressed))
    return msg


def _pulse_keys(
    pub: rospy.Publisher,
    keys: Set[str],
    *,
    duration_sec: float,
    dry_run: bool,
    abort_evt: threading.Event,
) -> None:
    if dry_run or not keys:
        return
    press = _joy_from_keys(keys, pressed=True)
    release = _joy_from_keys(keys, pressed=False)
    interval = 1.0 / max(JOY_PUBLISH_HZ, 1)
    end_t = time.time() + max(0.05, duration_sec)
    while time.time() < end_t and not rospy.is_shutdown():
        if abort_evt.is_set():
            break
        pub.publish(press)
        time.sleep(interval)
    for _ in range(3):
        if rospy.is_shutdown() or abort_evt.is_set():
            break
        pub.publish(release)
        time.sleep(interval)


def _publish_policy_name(
    pub: rospy.Publisher,
    policy_name: str,
    *,
    dry_run: bool,
    abort_evt: threading.Event,
) -> None:
    if dry_run or not policy_name:
        return
    msg = String(data=policy_name)
    interval = 1.0 / max(JOY_PUBLISH_HZ, 1)
    for _ in range(3):
        if rospy.is_shutdown() or abort_evt.is_set():
            break
        pub.publish(msg)
        time.sleep(interval)


@dataclass
class JoyActionSpec:
    gesture: int
    action_name: str
    label: str
    key_combo: str
    keepalive_sec: float

    @classmethod
    def from_gesture(cls, gesture: int) -> Optional["JoyActionSpec"]:
        row = GESTURE_JOY_ACTION_SPECS.get(gesture)
        if row is None:
            return None
        name, combo, keepalive = row
        return cls(
            gesture=gesture,
            action_name=name,
            label=GESTURE_ACTION_LABELS.get(gesture, name),
            key_combo=combo,
            keepalive_sec=keepalive,
        )


@dataclass
class PolicyActionSpec:
    gesture: int
    policy_name: str
    label: str
    duration_sec: float

    @classmethod
    def from_gesture(cls, gesture: int) -> Optional["PolicyActionSpec"]:
        row = GESTURE_POLICY_ACTION_SPECS.get(gesture)
        if row is None:
            return None
        name, duration = row
        return cls(
            gesture=gesture,
            policy_name=name,
            label=GESTURE_ACTION_LABELS.get(gesture, name),
            duration_sec=duration,
        )


class GestureActionPlayer:
    """手势边沿触发：G2/G3 joy_msg 脉冲；G4 policy_change 话题指令。"""

    def __init__(
        self,
        dry_run: bool = True,
        cooldown_sec: float = ACTION_COOLDOWN_SEC,
    ):
        self._dry_run = dry_run
        self._cooldown_sec = cooldown_sec
        self._joy_pub = rospy.Publisher(JOY_MSG_TOPIC, Joy, queue_size=1)
        self._policy_pub = rospy.Publisher(ACTION_CONFIG_TOPIC, String, queue_size=1)
        if not self._dry_run:
            t0 = time.time()
            while (
                self._joy_pub.get_num_connections() == 0
                and self._policy_pub.get_num_connections() == 0
                and not rospy.is_shutdown()
                and time.time() - t0 < 5.0
            ):
                time.sleep(0.05)
            if self._joy_pub.get_num_connections() == 0:
                rospy.logwarn("[gesture_action] 尚无 /joy_msg 订阅者")
            if self._policy_pub.get_num_connections() == 0:
                rospy.logwarn(
                    "[gesture_action] 尚无 %s 订阅者, policy 动作可能无效",
                    ACTION_CONFIG_TOPIC,
                )
        self._lock = threading.Lock()
        self._last_gesture = -1
        self._last_fire_t = 0.0
        self._busy_until = 0.0
        self._last_label = ""
        self._worker: Optional[threading.Thread] = None
        self._abort_evt = threading.Event()
        self._active_keys: Set[str] = set()

    @property
    def is_busy(self) -> bool:
        return time.time() < self._busy_until

    @property
    def last_label(self) -> str:
        return self._last_label

    def abort(self, *, fast: bool = False):
        keys = set(self._active_keys)
        label = self._last_label
        self._abort_evt.set()
        self._busy_until = 0.0
        self._last_label = ""
        if not fast and not self._dry_run and keys:
            _pulse_keys(
                self._joy_pub, keys,
                duration_sec=TRIGGER_PULSE_SEC,
                dry_run=False,
                abort_evt=threading.Event(),
            )
        self._active_keys.clear()
        if not fast and not self._dry_run:
            release = Joy()
            for _ in range(3):
                if self._abort_evt.is_set():
                    break
                self._joy_pub.publish(release)
                time.sleep(1.0 / max(JOY_PUBLISH_HZ, 1))
        if label and not fast:
            rospy.logwarn("[gesture_action] >>> 动作中止: %s", label)
        with self._lock:
            worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=0.15 if fast else 0.8)
        with self._lock:
            self._worker = None
        self._abort_evt.clear()

    def update(
        self,
        gesture: int,
        *,
        has_hand: bool,
        in_range: bool,
        joy_blocking: bool = False,
        fsm_ok: bool = True,
        allow_retry: bool = False,
    ) -> bool:
        is_joy = gesture in GESTURE_JOY_ACTION_SPECS
        is_policy = gesture in GESTURE_POLICY_ACTION_SPECS
        if not is_joy and not is_policy:
            self._last_gesture = -1
            return False

        prev = self._last_gesture
        self._last_gesture = gesture

        if joy_blocking or not fsm_ok or self.is_busy:
            return False
        if not has_hand or not in_range:
            return False
        if gesture == 0:
            return False
        if gesture == prev and not allow_retry:
            return False
        if time.time() - self._last_fire_t < self._cooldown_sec:
            return False

        if is_policy:
            spec = PolicyActionSpec.from_gesture(gesture)
            if spec is None:
                return False
            self._last_fire_t = time.time()
            self._busy_until = time.time() + spec.duration_sec + 0.5
            self._last_label = spec.label
            self._start_policy_play(spec)
            return True

        spec = JoyActionSpec.from_gesture(gesture)
        if spec is None:
            return False
        self._last_fire_t = time.time()
        self._busy_until = (
            time.time() + ACTION_DURATION_SEC + TRIGGER_PULSE_SEC * 2 + 0.5
        )
        self._last_label = spec.label
        self._start_joy_play(spec)
        return True

    def _start_joy_play(self, spec: JoyActionSpec):
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                rospy.logwarn("[gesture_action] 上一动作未完成，跳过 %s", spec.label)
                return
            self._worker = threading.Thread(
                target=self._play_joy_blocking,
                args=(spec,),
                daemon=True,
            )
            self._worker.start()

    def _start_policy_play(self, spec: PolicyActionSpec):
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                rospy.logwarn("[gesture_action] 上一动作未完成，跳过 %s", spec.label)
                return
            self._worker = threading.Thread(
                target=self._play_policy_blocking,
                args=(spec,),
                daemon=True,
            )
            self._worker.start()

    def _play_joy_blocking(self, spec: JoyActionSpec):
        keys = _parse_key_combo(spec.key_combo)
        self._active_keys = set(keys)
        line = format_action_trigger_line(spec.gesture, dry_run=self._dry_run)
        rospy.loginfo("[gesture_action] %s", line)
        print(line, flush=True)
        if self._dry_run:
            time.sleep(ACTION_DURATION_SEC)
            self._active_keys.clear()
            return

        try:
            _pulse_keys(
                self._joy_pub, keys,
                duration_sec=TRIGGER_PULSE_SEC,
                dry_run=False,
                abort_evt=self._abort_evt,
            )
            start_t = time.time()
            while (
                time.time() - start_t < ACTION_DURATION_SEC
                and not rospy.is_shutdown()
                and not self._abort_evt.is_set()
            ):
                time.sleep(0.05)

            if not self._abort_evt.is_set():
                stop_line = (
                    f">>> 动作停止: {spec.label} "
                    f"({ACTION_DURATION_SEC:.0f}s 后再次发送 {spec.key_combo})"
                )
                rospy.loginfo("[gesture_action] %s", stop_line)
                print(stop_line, flush=True)
                _pulse_keys(
                    self._joy_pub, keys,
                    duration_sec=TRIGGER_PULSE_SEC,
                    dry_run=False,
                    abort_evt=self._abort_evt,
                )
        finally:
            self._active_keys.clear()
            if not self._abort_evt.is_set():
                release = _joy_from_keys(keys, pressed=False) if keys else Joy()
                interval = 1.0 / max(JOY_PUBLISH_HZ, 1)
                for _ in range(3):
                    if rospy.is_shutdown():
                        break
                    self._joy_pub.publish(release)
                    time.sleep(interval)

    def _play_policy_blocking(self, spec: PolicyActionSpec):
        self._active_keys.clear()
        line = format_action_trigger_line(spec.gesture, dry_run=self._dry_run)
        rospy.loginfo("[gesture_action] %s", line)
        print(line, flush=True)
        if self._dry_run:
            time.sleep(spec.duration_sec)
            return

        try:
            _publish_policy_name(
                self._policy_pub,
                spec.policy_name,
                dry_run=False,
                abort_evt=self._abort_evt,
            )
            start_t = time.time()
            while (
                time.time() - start_t < spec.duration_sec
                and not rospy.is_shutdown()
                and not self._abort_evt.is_set()
            ):
                time.sleep(0.05)
            if not self._abort_evt.is_set():
                done_line = (
                    f">>> 动作完成: {spec.label} "
                    f"(policy {spec.policy_name} 由控制器自动回 walk)"
                )
                rospy.loginfo("[gesture_action] %s", done_line)
                print(done_line, flush=True)
        finally:
            self._active_keys.clear()
