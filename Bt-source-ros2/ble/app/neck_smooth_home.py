#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平滑脖子回中（locate_face OFF 时由 BLE 调用，不依赖头追进程存活）。"""

from __future__ import annotations

import math
import os
import sys
import time

_RATE_HZ = 50
_HOME_RATE_DEG = 45.0
_MAX_SEC = 8.0
_TOPIC = "/pi_plus_absolute"
_YAW_JOINT = "head_yaw_joint"
_PITCH_JOINT = "head_pitch_joint"


def _bootstrap_ros() -> None:
    home = os.environ.get("BIRD_HOME") or os.path.expanduser("~")
    ws = os.environ.get("COLCON_WS") or os.path.join(home, "colcon_ws")
    extra = [
        "/opt/ros/foxy/lib/python3.8/site-packages",
        "/opt/ros/foxy/local/lib/python3.8/dist-packages",
    ]
    install = os.path.join(ws, "install")
    if os.path.isdir(install):
        for name in os.listdir(install):
            site = os.path.join(install, name, "lib", "python3.8", "site-packages")
            if os.path.isdir(site):
                extra.append(site)
    for p in extra:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def _step_toward_zero(val_rad: float, step_rad: float) -> float:
    if abs(val_rad) <= 1e-4:
        return 0.0
    return val_rad - math.copysign(min(step_rad, abs(val_rad)), val_rad)


def smooth_home(yaw_deg: float, pitch_deg: float) -> None:
    _bootstrap_ros()
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("neck_smooth_home")
    pub = node.create_publisher(JointState, _TOPIC, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 1.5:
        time.sleep(0.05)

    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    if abs(yaw) < 1e-4 and abs(pitch) < 1e-4:
        node.destroy_node()
        return

    dt = 1.0 / _RATE_HZ
    step = math.radians(_HOME_RATE_DEG * dt)
    deadline = time.monotonic() + _MAX_SEC
    msg = JointState()
    msg.name = [_YAW_JOINT, _PITCH_JOINT]

    while time.monotonic() < deadline:
        yaw = _step_toward_zero(yaw, step)
        pitch = _step_toward_zero(pitch, step)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.position = [yaw, pitch]
        pub.publish(msg)
        if abs(yaw) < 1e-4 and abs(pitch) < 1e-4:
            break
        time.sleep(dt)

    msg.position = [0.0, 0.0]
    for _ in range(_RATE_HZ // 2):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(dt)
    node.destroy_node()


def main() -> int:
    yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    pitch = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    smooth_home(yaw, pitch)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
