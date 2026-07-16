#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
肩/脖子力矩 → /cmd_vel 映射桥。

订阅: /error_joint_states (sensor_msgs/JointState.effort)
发布: /cmd_vel (geometry_msgs/Twist)

肩部（左右取 |分量| 更大侧）:
  pitch ∈ [-1, -2.5]  →  linear.x ∈ [0, +1.5]   # 负力矩越大越快前进
  pitch ∈ [+1, +2]    →  linear.x ∈ [0, -1.0]   # 正力矩越大越快后退
  roll  < -1          →  angular.z = -1.0        # 向右转
  roll  > +1          →  angular.z = +1.0        # 向左转

脖子:
  pitch ∈ [-0.5, -1]  →  linear.x ∈ [0, +1]     # 负力矩 → 前进
  pitch ∈ [+0.3, +1]  →  linear.x ∈ [0, -1]     # 正力矩 → 后退
  yaw   < -0.5        →  angular.z = +1.0        # 向左自转
  yaw   > +0.5        →  angular.z = -1.0        # 向右自转

肩与脖子最终再按 |分量| 取强者。死区内对应分量=0；vx/wz 都为 0 时不发布。

用法:
  ./run_torque_bridge.sh
  ./run_torque_bridge.sh --dry-run
  ./run_torque_bridge.sh --side right
  ./run_torque_bridge.sh --no-neck
  ./run_torque_bridge.sh --no-arms

注意:
  仅在 FSM=EXEC_DEFAULT(5) 时发布 /cmd_vel。
  做自定义动作(EXEC_CUSTOM 等)时自动静默，避免力矩误触发速度、打断动作配乐。
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
from typing import Dict, List, Optional, Tuple


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
from std_msgs.msg import Int32


JOINT_STATE_TOPIC = "/error_joint_states"
CMD_VEL_TOPIC = "/cmd_vel"
FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5

# 左右肩关节
R_PITCH = "r_shoulder_pitch_joint"
R_ROLL = "r_shoulder_roll_joint"
L_PITCH = "l_shoulder_pitch_joint"
L_ROLL = "l_shoulder_roll_joint"

# 脖子关节（机型配置左右轴名为 head_yaw_joint）
NECK_PITCH = "head_pitch_joint"
NECK_YAW = "head_yaw_joint"

FSM_NAME = {
    0: "INIT",
    1: "ERROR",
    2: "CANDIDATE_DEFAULT",
    3: "CANDIDATE_CUSTOM",
    4: "CANDIDATE_REMOTE",
    5: "EXEC_DEFAULT",
    6: "EXEC_CUSTOM",
    7: "EXEC_REMOTE",
    8: "PROTECTION_SHUTDOWN",
    14: "EXEC_TEACHING",
    16: "EXEC_DEVELOP",
}

# ---- 肩 pitch → vx ----
# 前进: pitch -1 → 0, pitch -2.5 → +1.5
ARM_FWD_TAU_NEAR = -1.0
ARM_FWD_TAU_FAR = -2.5
ARM_FWD_VX_MAX = 1.5

# 后退: pitch +1 → 0, pitch +2 → -1
ARM_BWD_TAU_NEAR = 1.0
ARM_BWD_TAU_FAR = 2.0
ARM_BWD_VX_MAX = -1.0

# 肩 roll 转向阈值（已按实机校正符号）
ARM_ROLL_TURN_THRESH = 1.0
ARM_WZ_RIGHT = -1.0  # roll < -1 → 右转
ARM_WZ_LEFT = 1.0    # roll > +1 → 左转

# ---- 脖子 pitch → vx ----
# 前进: pitch -0.5 → 0, pitch -1 → +1
NECK_FWD_TAU_NEAR = -0.5
NECK_FWD_TAU_FAR = -1.0
NECK_FWD_VX_MAX = 1.0

# 后退: pitch +0.3 → 0, pitch +1 → -1
NECK_BWD_TAU_NEAR = 0.3
NECK_BWD_TAU_FAR = 1.0
NECK_BWD_VX_MAX = -1.0

# 脖子 yaw 自转（阶跃）
NECK_YAW_THRESH = 0.5
NECK_WZ_LEFT = 1.0    # yaw < -0.5 → 向左自转
NECK_WZ_RIGHT = -1.0  # yaw > +0.5 → 向右自转


def _map_pitch_range(
    tau: float,
    fwd_near: float,
    fwd_far: float,
    fwd_vx: float,
    bwd_near: float,
    bwd_far: float,
    bwd_vx: float,
) -> float:
    """通用 pitch 力矩线性映射。区间外钳位；死区返回 0。"""
    if tau <= fwd_near:
        t = max(fwd_far, min(fwd_near, tau))
        alpha = (fwd_near - t) / (fwd_near - fwd_far)
        return alpha * fwd_vx

    if tau >= bwd_near:
        t = max(bwd_near, min(bwd_far, tau))
        alpha = (t - bwd_near) / (bwd_far - bwd_near)
        return alpha * bwd_vx

    return 0.0


def map_arm_pitch_to_vx(tau: float) -> float:
    return _map_pitch_range(
        tau,
        ARM_FWD_TAU_NEAR,
        ARM_FWD_TAU_FAR,
        ARM_FWD_VX_MAX,
        ARM_BWD_TAU_NEAR,
        ARM_BWD_TAU_FAR,
        ARM_BWD_VX_MAX,
    )


def map_arm_roll_to_wz(tau: float) -> float:
    if tau < -ARM_ROLL_TURN_THRESH:
        return ARM_WZ_RIGHT
    if tau > ARM_ROLL_TURN_THRESH:
        return ARM_WZ_LEFT
    return 0.0


def map_neck_pitch_to_vx(tau: float) -> float:
    return _map_pitch_range(
        tau,
        NECK_FWD_TAU_NEAR,
        NECK_FWD_TAU_FAR,
        NECK_FWD_VX_MAX,
        NECK_BWD_TAU_NEAR,
        NECK_BWD_TAU_FAR,
        NECK_BWD_VX_MAX,
    )


def map_neck_yaw_to_wz(tau: float) -> float:
    if tau < -NECK_YAW_THRESH:
        return NECK_WZ_LEFT
    if tau > NECK_YAW_THRESH:
        return NECK_WZ_RIGHT
    return 0.0


def _pick_stronger(a: float, b: float) -> float:
    """取绝对值更大的一侧。"""
    return a if abs(a) >= abs(b) else b


class FsmGate:
    """仅 EXEC_DEFAULT(5) 允许发 /cmd_vel。"""

    def __init__(self, topic: str = FSM_STATE_TOPIC):
        self._lock = threading.Lock()
        self._state: Optional[int] = None
        rospy.Subscriber(topic, Int32, self._cb, queue_size=10)

    def _cb(self, msg: Int32) -> None:
        with self._lock:
            self._state = int(msg.data)

    @property
    def state(self) -> Optional[int]:
        with self._lock:
            return self._state

    def allow_cmd_vel(self) -> bool:
        return self.state == FSM_EXEC_DEFAULT

    def state_name(self) -> str:
        s = self.state
        if s is None:
            return "UNKNOWN"
        return FSM_NAME.get(s, "UNKNOWN(%d)" % s)


class TorqueCmdVelBridge:
    def __init__(
        self,
        *,
        arms: List[Tuple[str, str, str]],
        enable_neck: bool,
        neck_pitch_joint: str,
        neck_yaw_joint: str,
        joint_state_topic: str,
        cmd_vel_topic: str,
        rate_hz: float,
        print_hz: float,
        ema_alpha: float,
        dry_run: bool,
        vx_deadband: float,
        require_fsm: bool,
    ):
        # arms: [(side, pitch_joint, roll_joint), ...]
        self.arms = list(arms)
        self.enable_neck = bool(enable_neck)
        self.neck_pitch_joint = neck_pitch_joint
        self.neck_yaw_joint = neck_yaw_joint
        self.rate_hz = max(float(rate_hz), 1.0)
        self.print_hz = max(float(print_hz), 0.0)
        self.ema_alpha = min(max(float(ema_alpha), 0.0), 1.0)
        self.dry_run = dry_run
        self.vx_deadband = max(float(vx_deadband), 0.0)
        self.require_fsm = require_fsm

        self._lock = threading.Lock()
        # side -> {pitch, roll}
        self._tau: Dict[str, Dict[str, Optional[float]]] = {
            side: {"pitch": None, "roll": None} for side, _, _ in self.arms
        }
        self._neck: Dict[str, Optional[float]] = {"pitch": None, "yaw": None}
        self._vx_filt: Optional[float] = None
        self._msg_count = 0
        self._warned_missing = False
        self._was_active = False
        self._stop_pulses_left = 0
        self._stop_pulses = 5
        self._fsm_paused = False

        if not self.arms and not self.enable_neck:
            raise SystemExit("至少启用肩(--side)或脖子(默认)，不能同时 --no-arms 且 --no-neck")

        self._fsm = FsmGate() if require_fsm else None

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
            "[torque_bridge] sub %s → %s%s",
            joint_state_topic,
            cmd_vel_topic,
            " [DRY-RUN]" if dry_run else "",
        )
        for side, pj, rj in self.arms:
            rospy.loginfo("[torque_bridge] arm %s: pitch=%s  roll=%s", side, pj, rj)
        if self.arms:
            rospy.loginfo(
                "[torque_bridge] arm map: pitch[%.1f,%.1f]→vx[0,%+.1f]; "
                "pitch[%.1f,%.1f]→vx[0,%+.1f]; "
                "roll<%.1f→wz=%+.1f; roll>%.1f→wz=%+.1f",
                ARM_FWD_TAU_NEAR,
                ARM_FWD_TAU_FAR,
                ARM_FWD_VX_MAX,
                ARM_BWD_TAU_NEAR,
                ARM_BWD_TAU_FAR,
                ARM_BWD_VX_MAX,
                -ARM_ROLL_TURN_THRESH,
                ARM_WZ_RIGHT,
                ARM_ROLL_TURN_THRESH,
                ARM_WZ_LEFT,
            )
        if self.enable_neck:
            rospy.loginfo(
                "[torque_bridge] neck: pitch=%s  yaw=%s",
                self.neck_pitch_joint,
                self.neck_yaw_joint,
            )
            rospy.loginfo(
                "[torque_bridge] neck map: pitch[%.1f,%.1f]→vx[0,%+.1f]; "
                "pitch[%.1f,%.1f]→vx[0,%+.1f]; "
                "yaw<%.1f→wz=%+.1f; yaw>%.1f→wz=%+.1f",
                NECK_FWD_TAU_NEAR,
                NECK_FWD_TAU_FAR,
                NECK_FWD_VX_MAX,
                NECK_BWD_TAU_NEAR,
                NECK_BWD_TAU_FAR,
                NECK_BWD_VX_MAX,
                -NECK_YAW_THRESH,
                NECK_WZ_LEFT,
                NECK_YAW_THRESH,
                NECK_WZ_RIGHT,
            )
        if require_fsm:
            rospy.loginfo(
                "[torque_bridge] FSM 守门: 仅 EXEC_DEFAULT(5) 发 /cmd_vel "
                "(做动作时自动静默，不打断配乐)"
            )

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        missing = []
        updates: Dict[str, Dict[str, float]] = {}
        for side, pj, rj in self.arms:
            ip = name_to_i.get(pj)
            ir = name_to_i.get(rj)
            if ip is None or ir is None:
                if ip is None:
                    missing.append(pj)
                if ir is None:
                    missing.append(rj)
                continue
            if not msg.effort or max(ip, ir) >= len(msg.effort):
                continue
            updates[side] = {
                "pitch": float(msg.effort[ip]),
                "roll": float(msg.effort[ir]),
            }

        neck_update: Optional[Dict[str, float]] = None
        if self.enable_neck:
            ip = name_to_i.get(self.neck_pitch_joint)
            iy = name_to_i.get(self.neck_yaw_joint)
            if ip is None:
                missing.append(self.neck_pitch_joint)
            if iy is None:
                missing.append(self.neck_yaw_joint)
            if (
                ip is not None
                and iy is not None
                and msg.effort
                and max(ip, iy) < len(msg.effort)
            ):
                neck_update = {
                    "pitch": float(msg.effort[ip]),
                    "yaw": float(msg.effort[iy]),
                }

        if missing and not self._warned_missing:
            self._warned_missing = True
            rospy.logwarn(
                "[torque_bridge] JointState 缺少: %s；有: %s",
                missing,
                ", ".join(msg.name[:24]),
            )
        if not updates and neck_update is None:
            return
        with self._lock:
            for side, vals in updates.items():
                self._tau[side]["pitch"] = vals["pitch"]
                self._tau[side]["roll"] = vals["roll"]
            if neck_update is not None:
                self._neck["pitch"] = neck_update["pitch"]
                self._neck["yaw"] = neck_update["yaw"]
            self._msg_count += 1

    def _snapshot_tau(self) -> Optional[Dict[str, Dict[str, float]]]:
        with self._lock:
            out: Dict[str, Dict[str, float]] = {}
            for side, _, _ in self.arms:
                p = self._tau[side]["pitch"]
                r = self._tau[side]["roll"]
                if p is None or r is None:
                    continue
                out[side] = {"pitch": p, "roll": r}
            return out if out else None

    def _snapshot_neck(self) -> Optional[Dict[str, float]]:
        if not self.enable_neck:
            return None
        with self._lock:
            p = self._neck["pitch"]
            y = self._neck["yaw"]
            if p is None or y is None:
                return None
            return {"pitch": p, "yaw": y}

    def _combine(
        self,
        tau: Optional[Dict[str, Dict[str, float]]],
        neck: Optional[Dict[str, float]],
    ) -> Tuple[float, float]:
        """肩+脖子映射后按 |分量| 取强者。左手 pitch 符号与右手相反，取反后再映射。"""
        vx = 0.0
        wz = 0.0
        if tau:
            for side, vals in tau.items():
                pitch = vals["pitch"]
                if side == "left":
                    pitch = -pitch
                vx = _pick_stronger(vx, map_arm_pitch_to_vx(pitch))
                wz = _pick_stronger(wz, map_arm_roll_to_wz(vals["roll"]))
        if neck:
            vx = _pick_stronger(vx, map_neck_pitch_to_vx(neck["pitch"]))
            wz = _pick_stronger(wz, map_neck_yaw_to_wz(neck["yaw"]))
        return vx, wz

    def _has_any_source(self) -> bool:
        if self._snapshot_tau() is not None:
            return True
        if self._snapshot_neck() is not None:
            return True
        return False

    def _publish_twist(self, vx: float, wz: float) -> None:
        if self._pub is None:
            return
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self._pub.publish(tw)

    def _tick(self) -> None:
        if not self._has_any_source():
            return

        if self._fsm is not None and not self._fsm.allow_cmd_vel():
            if not self._fsm_paused:
                self._fsm_paused = True
                rospy.loginfo(
                    "[torque_bridge] FSM=%s，暂停 /cmd_vel（保护动作/配乐）",
                    self._fsm.state_name(),
                )
            self._was_active = False
            self._stop_pulses_left = 0
            self._vx_filt = None
            return
        if self._fsm_paused:
            self._fsm_paused = False
            rospy.loginfo(
                "[torque_bridge] FSM=%s，恢复力矩→速度",
                self._fsm.state_name() if self._fsm else "n/a",
            )

        tau = self._snapshot_tau()
        neck = self._snapshot_neck()
        vx_raw, wz = self._combine(tau, neck)

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

        if self._was_active:
            self._was_active = False
            self._stop_pulses_left = self._stop_pulses
            rospy.loginfo("[torque_bridge] 松手 → 发零速刹停")

        if self._stop_pulses_left > 0:
            self._stop_pulses_left -= 1
            self._publish_twist(0.0, 0.0)
            return

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
                tau = self._snapshot_tau()
                neck = self._snapshot_neck()
                with self._lock:
                    vx = self._vx_filt
                    cnt = self._msg_count
                if tau is not None or neck is not None:
                    vx_raw, wz_show = self._combine(tau, neck)
                    vx_show = 0.0 if vx is None else vx
                    if abs(vx_show) < self.vx_deadband:
                        vx_show = 0.0
                    if self._fsm is not None and not self._fsm.allow_cmd_vel():
                        fsm_tag = " [FSM:%s pause]" % self._fsm.state_name()
                        vx_show = 0.0
                        wz_show = 0.0
                    else:
                        fsm_tag = ""
                    idle = abs(vx_show) < 1e-9 and abs(wz_show) < 1e-9
                    parts = []
                    if tau:
                        for side, _, _ in self.arms:
                            if side in tau:
                                parts.append(
                                    "%s:p=%+.2f,r=%+.2f"
                                    % (side[0], tau[side]["pitch"], tau[side]["roll"])
                                )
                    if neck:
                        parts.append(
                            "n:p=%+.2f,y=%+.2f" % (neck["pitch"], neck["yaw"])
                        )
                    print(
                        "%s  →  vx=%+.3f  wz=%+.1f%s%s  (n=%d)"
                        % (
                            "  ".join(parts),
                            vx_show,
                            wz_show,
                            " [idle]" if idle and not fsm_tag else "",
                            fsm_tag,
                            cnt,
                        ),
                        flush=True,
                    )
            rate.sleep()


def _arms_from_side(side: str, enable_arms: bool) -> List[Tuple[str, str, str]]:
    if not enable_arms:
        return []
    side = side.lower()
    arms: List[Tuple[str, str, str]] = []
    if side in ("both", "left", "l"):
        arms.append(("left", L_PITCH, L_ROLL))
    if side in ("both", "right", "r"):
        arms.append(("right", R_PITCH, R_ROLL))
    if not arms:
        raise SystemExit("--side 需为 both / left / right")
    return arms


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="肩/脖子力矩 → /cmd_vel 桥")
    p.add_argument(
        "--side",
        default="both",
        choices=("both", "left", "right"),
        help="肩控制侧：both=左右同步（默认）",
    )
    p.add_argument("--no-arms", action="store_true", help="关闭肩力矩映射")
    p.add_argument("--no-neck", action="store_true", help="关闭脖子力矩映射")
    p.add_argument("--neck-pitch-joint", default=NECK_PITCH)
    p.add_argument("--neck-yaw-joint", default=NECK_YAW)
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
    p.add_argument(
        "--no-fsm",
        action="store_true",
        help="关闭 FSM 守门（不推荐；做动作时可能抢 /cmd_vel）",
    )
    p.add_argument("--dry-run", action="store_true", help="只映射打印，不发 /cmd_vel")
    return p


def main() -> None:
    args = build_parser().parse_args()
    rospy.init_node("torque_cmd_vel_bridge", anonymous=True)
    bridge = TorqueCmdVelBridge(
        arms=_arms_from_side(args.side, enable_arms=not args.no_arms),
        enable_neck=not args.no_neck,
        neck_pitch_joint=args.neck_pitch_joint,
        neck_yaw_joint=args.neck_yaw_joint,
        joint_state_topic=args.joint_state_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        rate_hz=args.rate,
        print_hz=args.print_hz,
        ema_alpha=args.ema,
        dry_run=args.dry_run,
        vx_deadband=args.vx_deadband,
        require_fsm=not args.no_fsm,
    )
    try:
        bridge.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
