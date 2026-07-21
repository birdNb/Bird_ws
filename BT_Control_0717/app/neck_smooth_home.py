#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平滑脖子回中（locate_face OFF 时由 BLE 调用，不依赖头追进程存活）。"""

from __future__ import annotations

import math
import os
import sys
import time

_RATE_HZ = 50
_HOME_RATE_DEG = 90.0
_MAX_SEC = 8.0
_TOPIC = "/pi_plus_absolute"
_YAW_JOINT = "head_yaw_joint"
_PITCH_JOINT = "head_pitch_joint"


def _bootstrap_ros() -> None:
    for p in (
        "/opt/ros/noetic/lib/python3/dist-packages",
        os.path.expanduser("~/sim2real/devel/lib/python3/dist-packages"),
        os.path.expanduser("~/sim2real/install/lib/python3/dist-packages"),
    ):
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def _step_toward_zero(val_rad: float, step_rad: float) -> float:
    if abs(val_rad) <= 1e-4:
        return 0.0
    return val_rad - math.copysign(min(step_rad, abs(val_rad)), val_rad)


def smooth_home(yaw_deg: float, pitch_deg: float) -> None:
    _bootstrap_ros()
    import rospy
    from sensor_msgs.msg import JointState

    if not rospy.core.is_initialized():
        rospy.init_node("neck_smooth_home", anonymous=True, disable_signals=True)

    pub = rospy.Publisher(_TOPIC, JointState, queue_size=10)
    t0 = time.monotonic()
    while pub.get_num_connections() == 0 and time.monotonic() - t0 < 1.5:
        time.sleep(0.05)

    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    if abs(yaw) < 1e-4 and abs(pitch) < 1e-4:
        return

    dt = 1.0 / _RATE_HZ
    step = math.radians(_HOME_RATE_DEG * dt)
    deadline = time.monotonic() + _MAX_SEC
    msg = JointState()
    msg.name = [_YAW_JOINT, _PITCH_JOINT]

    while time.monotonic() < deadline:
        yaw = _step_toward_zero(yaw, step)
        pitch = _step_toward_zero(pitch, step)
        msg.header.stamp = rospy.Time.now()
        msg.position = [yaw, pitch]
        pub.publish(msg)
        if abs(yaw) < 1e-4 and abs(pitch) < 1e-4:
            break
        time.sleep(dt)

    msg.position = [0.0, 0.0]
    for _ in range(_RATE_HZ // 2):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        time.sleep(dt)


def main() -> int:
    yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    pitch = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    smooth_home(yaw, pitch)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
