#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
右手肩 pitch/roll 力矩 → /cmd_vel 映射桥。

订阅: /error_joint_states (sensor_msgs/JointState.effort)
发布: /cmd_vel (geometry_msgs/Twist)

映射:
  pitch ∈ [-1, -2.5]  →  linear.x ∈ [0, +1.5]   # 负力矩越大越快前进
  pitch ∈ [+1, +2]    →  linear.x ∈ [0, -1.0]   # 正力矩越大越快后退
  roll  < -1          →  angular.z = -1.0        # 向右转
  roll  > +1          →  angular.z = +1.0        # 向左转
  死区内              →  对应分量=0；vx/wz 都为 0 时不发布

用法:
  ./run_pitch_bridge.sh
  ./run_pitch_bridge.sh --dry-run
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
from typing import Optional


def _ensure_sim2real_python() -> None:
    for rel in (
        "~/sim2real/devel/lib/python3/dist-packages",
        "~/sim2real/install/lib/python3/dist-packages",
    ):
        p = os.path.expanduser(rel)
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


_ensure_sim2real_python()

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


JOINT_STATE_TOPIC = "/error_joint_states"
CMD_VEL_TOPIC = "/cmd_vel"
PITCH_JOINT = "r_shoulder_pitch_joint"
ROLL_JOINT = "r_shoulder_roll_joint"

# 前进: pitch -1 → 0, pitch -2.5 → +1.5
FWD_TAU_NEAR = -1.0
FWD_TAU_FAR = -2.5
FWD_VX_MAX = 1.5

# 后退: pitch +1 → 0, pitch +2 → -1
BWD_TAU_NEAR = 1.0
BWD_TAU_FAR = 2.0
BWD_VX_MAX = -1.0

# roll 转向阈值（已按实机校正符号）
ROLL_TURN_THRESH = 1.0
WZ_RIGHT = -1.0  # roll < -1 → 右转
WZ_LEFT = 1.0    # roll > +1 → 左转


def map_pitch_to_vx(tau: float) -> float:
    """pitch 力矩 → linear.x。区间外钳位到对应端点；死区返回 0。"""
    if tau <= FWD_TAU_NEAR:
        t = max(FWD_TAU_FAR, min(FWD_TAU_NEAR, tau))
        alpha = (FWD_TAU_NEAR - t) / (FWD_TAU_NEAR - FWD_TAU_FAR)
        return alpha * FWD_VX_MAX

    if tau >= BWD_TAU_NEAR:
        t = max(BWD_TAU_NEAR, min(BWD_TAU_FAR, tau))
        alpha = (t - BWD_TAU_NEAR) / (BWD_TAU_FAR - BWD_TAU_NEAR)
        return alpha * BWD_VX_MAX

    return 0.0


def map_roll_to_wz(tau: float) -> float:
    """roll 力矩 → angular.z（阶跃）。"""
    if tau < -ROLL_TURN_THRESH:
        return WZ_RIGHT
    if tau > ROLL_TURN_THRESH:
        return WZ_LEFT
    return 0.0


class PitchTorqueCmdVelBridge:
    def __init__(
        self,
        *,
        pitch_joint: str,
        roll_joint: str,
        joint_state_topic: str,
        cmd_vel_topic: str,
        rate_hz: float,
        print_hz: float,
        ema_alpha: float,
        dry_run: bool,
        vx_deadband: float,
    ):
        self.pitch_joint = pitch_joint
        self.roll_joint = roll_joint
        self.rate_hz = max(float(rate_hz), 1.0)
        self.print_hz = max(float(print_hz), 0.0)
        self.ema_alpha = min(max(float(ema_alpha), 0.0), 1.0)
        self.dry_run = dry_run
        self.vx_deadband = max(float(vx_deadband), 0.0)

        self._lock = threading.Lock()
        self._pitch: Optional[float] = None
        self._roll: Optional[float] = None
        self._vx_filt: Optional[float] = None
        self._msg_count = 0
        self._warned_missing = False
        self._was_active = False
        # 松手后连发几次零速，确保底盘停转
        self._stop_pulses_left = 0
        self._stop_pulses = 5

        self._pub = None
        if not dry_run:
            self._pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_state,
            queue_size=20,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "[pitch_bridge] sub %s → %s%s",
            joint_state_topic,
            cmd_vel_topic,
            " [DRY-RUN]" if dry_run else "",
        )
        rospy.loginfo(
            "[pitch_bridge] pitch=%s  roll=%s",
            pitch_joint,
            roll_joint,
        )
        rospy.loginfo(
            "[pitch_bridge] map: pitch[%.1f,%.1f]→vx[0,%+.1f]; "
            "pitch[%.1f,%.1f]→vx[0,%+.1f]; "
            "roll<%.1f→wz=%+.1f; roll>%.1f→wz=%+.1f",
            FWD_TAU_NEAR,
            FWD_TAU_FAR,
            FWD_VX_MAX,
            BWD_TAU_NEAR,
            BWD_TAU_FAR,
            BWD_VX_MAX,
            -ROLL_TURN_THRESH,
            WZ_RIGHT,
            ROLL_TURN_THRESH,
            WZ_LEFT,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        ip = name_to_i.get(self.pitch_joint)
        ir = name_to_i.get(self.roll_joint)
        if ip is None or ir is None:
            if not self._warned_missing:
                self._warned_missing = True
                missing = []
                if ip is None:
                    missing.append(self.pitch_joint)
                if ir is None:
                    missing.append(self.roll_joint)
                rospy.logwarn(
                    "[pitch_bridge] JointState 无 %s，有: %s",
                    missing,
                    ", ".join(msg.name[:20]),
                )
            return
        if not msg.effort or max(ip, ir) >= len(msg.effort):
            return
        with self._lock:
            self._pitch = float(msg.effort[ip])
            self._roll = float(msg.effort[ir])
            self._msg_count += 1

    def _publish_twist(self, vx: float, wz: float) -> None:
        if self._pub is None:
            return
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self._pub.publish(tw)

    def _tick(self) -> None:
        with self._lock:
            pitch = self._pitch
            roll = self._roll
        if pitch is None or roll is None:
            return

        vx_raw = map_pitch_to_vx(pitch)
        wz = map_roll_to_wz(roll)

        with self._lock:
            if self._vx_filt is None or self.ema_alpha >= 1.0:
                vx = vx_raw
            else:
                a = self.ema_alpha
                vx = a * vx_raw + (1.0 - a) * self._vx_filt
            self._vx_filt = vx

        if abs(vx) < self.vx_deadband:
            vx = 0.0

        active = abs(vx) >= 1e-9 or abs(wz) >= 1e-9

        if active:
            self._was_active = True
            self._stop_pulses_left = 0
            self._publish_twist(vx, wz)
            return

        # 刚从有指令变为空闲：连发零速刹停，避免自转不停
        if self._was_active:
            self._was_active = False
            self._stop_pulses_left = self._stop_pulses
            rospy.loginfo("[pitch_bridge] 松手 → 发零速刹停")

        if self._stop_pulses_left > 0:
            self._stop_pulses_left -= 1
            self._publish_twist(0.0, 0.0)
            return
        # 之后保持静默，不抢手柄

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        print_every = (
            max(1, int(round(self.rate_hz / self.print_hz)))
            if self.print_hz > 0
            else 0
        )
        n = 0
        while not rospy.is_shutdown():
            self._tick()
            n += 1
            if print_every and n % print_every == 0:
                with self._lock:
                    pitch = self._pitch
                    roll = self._roll
                    vx = self._vx_filt
                    cnt = self._msg_count
                if pitch is not None and roll is not None:
                    vx_show = 0.0 if vx is None else vx
                    if abs(vx_show) < self.vx_deadband:
                        vx_show = 0.0
                    wz_show = map_roll_to_wz(roll)
                    idle = abs(vx_show) < 1e-9 and abs(wz_show) < 1e-9
                    print(
                        "pitch=%+.3f  roll=%+.3f  →  vx=%+.3f  wz=%+.1f%s  (n=%d)"
                        % (
                            pitch,
                            roll,
                            vx_show,
                            wz_show,
                            " [idle]" if idle else "",
                            cnt,
                        ),
                        flush=True,
                    )
            rate.sleep()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="pitch/roll 力矩 → /cmd_vel 桥")
    p.add_argument("--pitch-joint", default=PITCH_JOINT)
    p.add_argument("--roll-joint", default=ROLL_JOINT)
    p.add_argument("--joint-state-topic", default=JOINT_STATE_TOPIC)
    p.add_argument("--cmd-vel-topic", default=CMD_VEL_TOPIC)
    p.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz")
    p.add_argument("--print-hz", type=float, default=10.0, help="终端打印频率，0=不打印")
    p.add_argument(
        "--ema",
        type=float,
        default=0.35,
        help="vx EMA 系数 0~1，越大越跟手（1=无滤波）",
    )
    p.add_argument(
        "--vx-deadband",
        type=float,
        default=0.02,
        help="|vx| 小于此值视为 0",
    )
    p.add_argument("--dry-run", action="store_true", help="只映射打印，不发 /cmd_vel")
    return p


def main() -> None:
    args = build_parser().parse_args()
    rospy.init_node("pitch_torque_cmd_vel_bridge", anonymous=True)
    bridge = PitchTorqueCmdVelBridge(
        pitch_joint=args.pitch_joint,
        roll_joint=args.roll_joint,
        joint_state_topic=args.joint_state_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        rate_hz=args.rate,
        print_hz=args.print_hz,
        ema_alpha=args.ema,
        dry_run=args.dry_run,
        vx_deadband=args.vx_deadband,
    )
    try:
        bridge.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
