#!/usr/bin/env python3
"""限力矩顺时针顶硬限位：一次一轴，撞硬限位后复位，只把限位 URDF 角写入 yaml。

寻限位轴 MIXED_MODE + 正 torques → midware pos_vel_MAXtqe（CDC 0x90）。
同一总线帧不能混 0x90 / 0xB0，否则后写的模式会冲掉前面的指令。
因此本脚本对所有已接管轴都走 0x90：运动轴 vel>0，其余轴发当前位置。
pos_vel_MAXtqe 在 vel=0 时只锁当前位置，预备/斜坡必须带速度。

只有 |τ| 顶满保护值且速度≈0 才停；位置冻住但力矩不够则继续推。
顺时针 = 电机编码器增加。URDF 方向 = joints.yaml direction。
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import GetAvailableMotors, ReleaseControl, RequestControl
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEG = math.pi / 180.0
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_YAML = os.path.join(HERE, "standing_pose.yaml")

# 远端先、近端后。dir: 电机顺时针对应的 URDF 方向（joints.yaml direction）
GROUPS: Dict[str, Dict[str, Any]] = {
    "right_arm": {
        "title": "右手 4 轴",
        "joints": (
            "r_elbow_joint",
            "r_upper_arm_joint",
            "r_shoulder_roll_joint",
            "r_shoulder_pitch_joint",
        ),
        "dir": {
            "r_shoulder_pitch_joint": -1.0,
            "r_shoulder_roll_joint": -1.0,
            "r_upper_arm_joint": -1.0,
            "r_elbow_joint": 1.0,
        },
        "tau": {
            "r_elbow_joint": 1.0,
            "r_upper_arm_joint": 2.0,
            "r_shoulder_roll_joint": 2.0,
            "r_shoulder_pitch_joint": 2.0,
        },
        "seek_vel": 2.0,
        "yaml_key": "right_arm_limit_q",
        "scan_begin": "# === right_arm_limit_scan (generated) ===",
        "scan_end": "# === end right_arm_limit_scan ===",
    },
    "waist_neck": {
        "title": "腰 + 脖子",
        "joints": (
            "head_pitch_joint",
            "head_yaw_joint",
            "waist_yaw_joint",
        ),
        "dir": {
            "head_pitch_joint": -1.0,
            "head_yaw_joint": 1.0,
            "waist_yaw_joint": 1.0,
        },
        "tau": {
            "head_pitch_joint": 1.0,
            "head_yaw_joint": 1.0,
            "waist_yaw_joint": 1.0,
        },
        "yaml_key": "waist_neck_limit_q",
        "scan_begin": "# === waist_neck_limit_scan (generated) ===",
        "scan_end": "# === end waist_neck_limit_scan ===",
    },
    "right_leg": {
        "title": "右腿",
        "joints": (
            "r_ankle_roll_joint",
            "r_ankle_pitch_joint",
            "r_calf_joint",
            "r_thigh_joint",
            "r_hip_roll_joint",
            "r_hip_pitch_joint",
        ),
        "dir": {
            "r_hip_pitch_joint": -1.0,
            "r_hip_roll_joint": -1.0,
            "r_thigh_joint": -1.0,
            "r_calf_joint": 1.0,
            "r_ankle_pitch_joint": -1.0,
            "r_ankle_roll_joint": 1.0,
        },
        # hip roll/pitch 抬腿抗重力：需 ≥5 N·m；勿用 --tau-protect 全局覆盖
        "tau": {
            "r_hip_pitch_joint": 5.0,
            "r_hip_roll_joint": 5.0,
            "r_thigh_joint": 4.0,
            "r_calf_joint": 2.0,
            "r_ankle_pitch_joint": 2.0,
            "r_ankle_roll_joint": 2.0,
            "waist_yaw_joint": 3.0,
        },
        # 预备：腰 → (踝 pitch -30° 与膝 +30° 同时) → 髋抬；踝 roll 测前卸力
        "prep": (
            {
                "name": "waist_yaw_joint",
                "delta_deg": -90.0,
                "note": "腰右转 90°",
                "hold_after": False,
            },
            (
                {
                    "name": "r_ankle_pitch_joint",
                    "delta_deg": -30.0,
                    "note": "右踝 pitch -30°",
                    "hold_after": True,
                },
                {
                    "name": "r_calf_joint",
                    "delta_deg": 30.0,
                    "note": "右膝(calf)弯曲 +30°",
                    "hold_after": True,
                },
            ),
            {
                "name": "r_hip_roll_joint",
                "delta_deg": -45.0,
                "note": "右髋 roll 向上 45°",
                "hold_after": True,
            },
        ),
        # 踝 roll 抬腿前卸力；测完复位后固定。踝 pitch 抬腿前已弯到 -30° 并保持
        "unload_until_sought": ("r_ankle_roll_joint",),
        # 寻限位时：组内腿轴位控固定；组外（臂/头/左腿/腰）卸力
        "idle_mode": "leg_hold",
        # 这些轴顶满保护力矩后容易卡住判不出：持续满力矩 ≥ 该秒数则强制记限位并返回
        "force_tau_limit_sec": {
            "r_hip_roll_joint": 1.5,
            "r_thigh_joint": 1.5,
        },
        "hold_after_restore": True,
        "restore_prep": True,
        "yaml_key": "right_leg_limit_q",
        "scan_begin": "# === right_leg_limit_scan (generated) ===",
        "scan_end": "# === end right_leg_limit_scan ===",
    },
}


@dataclass
class JointRecord:
    name: str
    motor_id: int
    seek_dir_urdf: float
    start_q: float = 0.0
    tau_protect: float = 2.0
    tau_abort: float = 2.0
    tau_bias: float = 0.0
    q_enc_limit: Optional[float] = None
    q_travel_rad: Optional[float] = None
    stall_tau: Optional[float] = None
    restored_q: Optional[float] = None
    status: str = "pending"
    note: str = ""
    samples: List[float] = field(default_factory=list)


class CwLimit(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cw_limit")
        self.args = args
        self.group_cfg = GROUPS[args.group]
        self.group_joints: Tuple[str, ...] = self.group_cfg["joints"]
        self.tau_abort = float(args.tau_abort)
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.rate_hz = float(args.rate)
        self.seek_vel = float(
            args.seek_vel
            if args.seek_vel is not None
            else self.group_cfg.get("seek_vel", 1.0)
        )
        self.min_travel = float(args.min_travel_deg) * DEG
        self.max_travel = float(args.max_travel_deg) * DEG
        self.stall_vel = float(args.stall_vel)
        self.stall_hold = float(args.stall_hold)
        self.bias_sec = float(args.bias_sec)
        self.backoff = float(args.backoff_deg) * DEG
        self.restore_sec = float(args.restore_sec)
        self.seek_timeout = float(args.seek_timeout)
        self.yaml_path = os.path.abspath(args.yaml)
        self.clockwise = not args.ccw
        self.scan_begin = self.group_cfg["scan_begin"]
        self.scan_end = self.group_cfg["scan_end"]
        self.yaml_key = self.group_cfg["yaml_key"]

        self.js_names: List[str] = []
        self.q: Dict[str, float] = {}
        self.dq: Dict[str, float] = {}
        self.tau: Dict[str, float] = {}
        self.js_stamp = 0.0

        self.uuid = ""
        self.motor_ids: List[int] = []
        self.ctrl_names: List[str] = []
        self.seek_names: List[str] = []
        self.hold_names: set = set()
        self.hold_q: Dict[str, float] = {}
        self.prep_origin: Dict[str, float] = {}
        self.run_origin: Dict[str, float] = {}
        # 未测完前卸力；测完复位后从集合移除并位控固定
        self.unload_until_sought: set = set(self.group_cfg.get("unload_until_sought", ()))
        self.records: Dict[str, JointRecord] = {}
        self._stopping = False

        self.cmd_pub = self.create_publisher(MotorControlCommand, "control_command", 20)
        self.create_subscription(JointState, "joint_states", self._on_js, 50)
        self.cli_get = self.create_client(GetAvailableMotors, "get_available_motors")
        self.cli_req = self.create_client(RequestControl, "request_control")
        self.cli_rel = self.create_client(ReleaseControl, "release_control")

    def _on_js(self, msg: JointState) -> None:
        if not self.js_names and msg.name:
            self.js_names = list(msg.name)
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.q[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.dq[name] = float(msg.velocity[i])
            if i < len(msg.effort):
                self.tau[name] = float(msg.effort[i])
        self.js_stamp = time.monotonic()

    def spin_for(self, dt: float) -> None:
        deadline = time.monotonic() + dt
        while rclpy.ok() and not self._stopping and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))

    def wait_js(self, timeout: float = 5.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.js_names and (time.monotonic() - self.js_stamp) < 0.5:
                return True
            self.spin_for(0.05)
        return False

    def call_srv(self, client, request, timeout: float = 8.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"服务不可用: {client.srv_name}")
        future = client.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f"调用超时: {client.srv_name}")
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)
        print(msg, flush=True)

    def warn(self, msg: str) -> None:
        self.get_logger().warn(msg)
        print("[warn] " + msg, flush=True)

    def motor_id(self, name: str) -> int:
        if name not in self.js_names:
            raise RuntimeError(f"joint_states 中没有 {name}")
        return self.js_names.index(name)

    def seek_dir(self, name: str) -> float:
        base = float(self.group_cfg["dir"][name])
        return base if self.clockwise else -base

    def tau_protect_of(self, name: str) -> float:
        if self.args.tau_protect is not None:
            return float(self.args.tau_protect)
        return float(self.group_cfg["tau"].get(name, 2.0))

    def tau_abort_of(self, name: str) -> float:
        return max(self.tau_abort, self.tau_protect_of(name) + 2.0)

    def max_lead(self, name: Optional[str] = None) -> float:
        if name:
            tau = self.tau_protect_of(name)
        else:
            tau = min(self.group_cfg["tau"].values())
        return max(tau / max(self.kp, 1e-3), 0.02)

    def motors_idle(self, names: Sequence[str]) -> Tuple[bool, str]:
        req = GetAvailableMotors.Request()
        req.node_name = ""
        resp = self.call_srv(self.cli_get, req)
        idle = set(resp.motor_names)
        missing = [n for n in names if n not in idle]
        if missing:
            return False, "仍被占用: " + ", ".join(missing)
        return True, "idle"

    def maybe_stop_controller(self) -> None:
        if not self.args.takeover:
            return
        self.warn(" --takeover: 停止 hightorque_controller_node，释放电机")
        subprocess.call(
            ["pkill", "-INT", "-f", "hightorque_controller_node"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:
            self.spin_for(0.3)
            ok, _ = self.motors_idle(self.seek_names)
            if ok:
                self.log("控制器已释放目标电机")
                return
        self.warn("等待释放超时，仍尝试申请控制权")

    def request_group(self) -> None:
        self.seek_names = [n for n in self.group_joints if n in self.args.joints]
        if not self.seek_names:
            raise RuntimeError("没有要测试的关节")
        idle, msg = self.motors_idle(self.seek_names)
        if not idle:
            self.warn(msg)
            self.maybe_stop_controller()
            idle, msg = self.motors_idle(self.seek_names)
            if not idle and not self.args.takeover:
                raise RuntimeError(msg + "。先卸力后停控制器，或加 --takeover")

        get_req = GetAvailableMotors.Request()
        get_req.node_name = ""
        idle_resp = self.call_srv(self.cli_get, get_req)
        self.ctrl_names = list(idle_resp.motor_names)
        self.motor_ids = list(idle_resp.motor_ids)
        missing = [n for n in self.seek_names if n not in self.ctrl_names]
        if missing:
            raise RuntimeError("目标关节仍被占用: " + ", ".join(missing))
        if not self.ctrl_names:
            raise RuntimeError("没有空闲电机")

        idle_mode = self.group_cfg.get("idle_mode", "unload")
        # damping：超时进阻尼；leg_hold/unload：超时卸力（腿轴由指令帧维持位控）
        use_damp_default = idle_mode == "damping"
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = list(self.motor_ids)
        req.control_mode = RequestControl.Request.MIXED_MODE
        req.default_behavior = (
            RequestControl.Request.DAMPING_MODE if use_damp_default
            else RequestControl.Request.ZERO_TORQUE_MODE
        )
        req.timeout_ms = 2000
        req.default_kp = [0.0] * len(self.motor_ids)
        req.default_kd = [(1.0 if use_damp_default else 0.0)] * len(self.motor_ids)
        resp = self.call_srv(self.cli_req, req)
        if not resp.success:
            raise RuntimeError("申请控制权失败: " + resp.message)
        self.uuid = resp.uuid
        if idle_mode == "leg_hold":
            rest = "腿轴固定、其余卸力"
        elif idle_mode == "damping":
            rest = "其余阻尼"
        else:
            rest = "其余卸力"
        self.log(
            f"已申请 {len(self.ctrl_names)} 轴（寻限位 {self.group_cfg['title']}，{rest}） "
            f"uuid={self.uuid[:8]}"
        )

    def release_group(self, mode: int = ReleaseControl.Request.ZERO_TORQUE_MODE) -> None:
        if not self.uuid:
            return
        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = mode
        try:
            resp = self.call_srv(self.cli_rel, req, timeout=3.0)
            self.log("测完已卸力 (kp=0 kd=0 τ=0): " + (resp.message if resp else "no resp"))
        except Exception as exc:
            self.warn(f"释放控制权失败: {exc}")
        self.uuid = ""

    def publish_targets(
        self,
        targets: Dict[str, float],
        seeking: Optional[str] = None,
        moving: Optional[Any] = None,
    ) -> None:
        cmd = MotorControlCommand()
        cmd.uuid = self.uuid
        cmd.motor_ids = list(self.motor_ids)
        positions: List[float] = []
        velocities: List[float] = []
        kps: List[float] = []
        kds: List[float] = []
        torques: List[float] = []
        # pos_vel_MAXtqe 的 vel 是去目标位置的速度上限，必须 >0 才会动。
        # 全轴走同一 0x90 帧：kp>0 且 τ>0，避免和 0xB0 混帧把寻限位指令冲掉。
        cmd_vel = abs(self.seek_vel)
        idle_mode = self.group_cfg.get("idle_mode", "unload")
        leg_set = set(self.group_joints)
        if moving is None:
            moving_set: set = set()
        elif isinstance(moving, str):
            moving_set = {moving}
        else:
            moving_set = set(moving)
        for name in self.ctrl_names:
            q_now = self.q.get(name, 0.0)
            raw = targets.get(name, q_now)
            if name == seeking or name in moving_set:
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(abs(self.tau_protect_of(name)))
                velocities.append(cmd_vel)
                positions.append(float(raw))
            elif name in self.unload_until_sought:
                # 尚未测完的踝等：卸力，方便离支架
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(0.05)
                velocities.append(0.0)
                positions.append(float(q_now))
            elif name in self.hold_names or (
                idle_mode == "leg_hold" and name in leg_set
            ):
                # 腿轴 / 显式保持：位控固定
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(abs(self.tau_protect_of(name)))
                velocities.append(0.0)
                positions.append(float(raw))
            elif idle_mode == "damping":
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(2.0)
                velocities.append(0.0)
                positions.append(float(q_now))
            else:
                # 卸力：极小 τ 仍走 0x90，避免混帧
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(0.05)
                velocities.append(0.0)
                positions.append(float(q_now))
        cmd.positions = positions
        cmd.velocities = velocities
        cmd.kp = kps
        cmd.kd = kds
        cmd.torques = torques
        self.cmd_pub.publish(cmd)

    def hold_loop(self, duration: float, seeking: Optional[str] = None) -> None:
        t0 = time.monotonic()
        dt = 1.0 / self.rate_hz
        while time.monotonic() - t0 < duration and rclpy.ok() and not self._stopping:
            self.publish_targets(self.hold_q, seeking=seeking)
            self.spin_for(dt)

    def sample_bias(self, name: str) -> float:
        acc: List[float] = []
        t0 = time.monotonic()
        dt = 1.0 / self.rate_hz
        while time.monotonic() - t0 < self.bias_sec:
            self.publish_targets(self.hold_q, seeking=None)
            if name in self.tau:
                acc.append(self.tau[name])
            self.spin_for(dt)
        return sum(acc) / max(len(acc), 1)

    def seek_one(self, rec: JointRecord) -> None:
        name = rec.name
        d = rec.seek_dir_urdf
        rec.start_q = self.q[name]
        rec.tau_protect = self.tau_protect_of(name)
        rec.tau_abort = self.tau_abort_of(name)
        rec.tau_bias = self.sample_bias(name)
        self.log(
            f"[{name}] 起点 {rec.start_q:.4f} rad, τ_protect={rec.tau_protect:.2f} N·m, "
            f"τ_bias={rec.tau_bias:.3f} N·m, URDF方向 {d:+.0f} (电机顺时针)"
        )

        stall_t0: Optional[float] = None
        freeze_t0: Optional[float] = None
        freeze_q: Optional[float] = None
        window: deque = deque(maxlen=40)
        t0 = time.monotonic()
        dt = 1.0 / self.rate_hz
        cmd_q = rec.start_q
        freeze_eps = 0.5 * DEG
        warned_soft = False
        force_tau_sec = float(
            self.group_cfg.get("force_tau_limit_sec", {}).get(name, 0.0) or 0.0
        )
        tau_full_t0: Optional[float] = None

        while rclpy.ok() and not self._stopping:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed > self.seek_timeout:
                rec.status = "fail"
                rec.note = f"超时 {self.seek_timeout:.0f}s 未顶到硬件限位"
                break

            q = self.q.get(name, cmd_q)
            dq = self.dq.get(name, 0.0)
            tau = self.tau.get(name, 0.0)
            travel = (q - rec.start_q) * d
            tau_rel = tau - rec.tau_bias

            cmd_q = rec.start_q + d * min(self.seek_vel * elapsed, self.max_travel + 0.05)
            self.hold_q[name] = cmd_q
            self.publish_targets(self.hold_q, seeking=name)

            lead = (cmd_q - q) * d
            at_protect = (
                abs(tau) >= rec.tau_protect * 0.9
                or abs(tau_rel) >= rec.tau_protect * 0.9
            )
            # hip_roll / thigh：满保护力矩持续 force_tau_sec 则强制记限位并返回
            if force_tau_sec > 0.0 and at_protect and travel >= self.min_travel:
                if tau_full_t0 is None:
                    tau_full_t0 = now
                    self.log(
                        f"[{name}] 已达保护力矩 |τ|={abs(tau):.2f}，"
                        f"{force_tau_sec:.1f}s 内强制记限位并返回"
                    )
                elif now - tau_full_t0 >= force_tau_sec:
                    rec.q_enc_limit = median(list(window)) if window else q
                    rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                    rec.stall_tau = tau
                    rec.status = "limit"
                    rec.note = (
                        f"满力矩强制限位 {force_tau_sec:.1f}s |τ|={abs(tau):.2f} "
                        f"(保护 {rec.tau_protect:.2f}), 行程 {rec.q_travel_rad / DEG:.2f} deg"
                    )
                    self.log(
                        f"[{name}] 强制超时恢复: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
                    )
                    break
            else:
                tau_full_t0 = None

            moving = abs(dq) >= self.stall_vel
            if moving:
                stall_t0 = None
                freeze_t0 = None
                freeze_q = None
                window.clear()
            else:
                window.append(q)
                if freeze_t0 is None or freeze_q is None or abs(q - freeze_q) > freeze_eps:
                    freeze_t0 = now
                    freeze_q = q
                frozen = (
                    travel >= self.min_travel
                    and freeze_t0 is not None
                    and now - freeze_t0 >= self.stall_hold
                )
                # 指令在寻限位方向上超前实际角：电机跟不上才是真堵转。
                # 抬腿抗重力时 |τ| 可能一直顶满 protect，不能单靠绝对力矩判限位。
                cmd_ahead = lead >= max(self.max_lead(name), 0.12)
                torque_hit = travel >= self.min_travel and (
                    abs(tau_rel) >= rec.tau_protect
                    or (abs(tau) >= rec.tau_protect * 0.9 and cmd_ahead)
                )
                if stall_t0 is None and torque_hit:
                    stall_t0 = now
                hardware_stop = (
                    frozen
                    and torque_hit
                    and stall_t0 is not None
                    and now - stall_t0 >= self.stall_hold
                )
                if frozen and not torque_hit and not warned_soft:
                    self.warn(
                        f"[{name}] 位置停滞但 |τ|={abs(tau):.2f} |τ-τb|={abs(tau_rel):.2f} "
                        f"lead={lead:.3f} < 保护 {rec.tau_protect:.2f} N·m，"
                        "当作软件限位/重力负载，继续往硬件限位推"
                    )
                    warned_soft = True
                if hardware_stop:
                    rec.q_enc_limit = median(list(window)) if window else q
                    rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                    rec.stall_tau = tau
                    rec.status = "limit"
                    rec.note = (
                        f"硬件限位 |τ|={abs(tau):.2f} |τ-τb|={abs(tau_rel):.2f} "
                        f"lead={lead:.3f} (保护 {rec.tau_protect:.2f}), "
                        f"行程 {rec.q_travel_rad / DEG:.2f} deg"
                    )
                    self.log(f"[{name}] 撞限位，开始返回: {rec.note}, q_limit={rec.q_enc_limit:.4f}")
                    break

            if rec.status == "pending" and (
                abs(tau) >= rec.tau_abort or abs(tau_rel) >= rec.tau_abort
            ):
                rec.status = "fail"
                rec.note = f"力矩过大 τ={tau:.2f} (rel {tau_rel:.2f}) ≥ {rec.tau_abort}"
                break

            if travel > self.max_travel:
                rec.status = "fail"
                rec.note = f"超过行程帽 {self.max_travel / DEG:.1f} deg 仍未顶满保护力矩"
                break

            if int(elapsed * 4) != int((elapsed - dt) * 4):
                self.log(
                    f"[{name}] t={elapsed:5.1f}s  q={q:+.4f}  "
                    f"Δ={(q - rec.start_q) / DEG:+6.2f}°  "
                    f"τ={tau:+.3f} (rel {tau_rel:+.3f})  dq={dq:+.3f}"
                )
            self.spin_for(dt)
        else:
            if rec.status == "pending":
                rec.status = "fail"
                rec.note = "循环结束未采样"

        if rec.q_enc_limit is None and rec.status != "pending":
            rec.q_enc_limit = self.q.get(name, rec.start_q)
            rec.q_travel_rad = rec.q_enc_limit - rec.start_q
            rec.stall_tau = self.tau.get(name, rec.tau_bias)
        q_now = self.q.get(name, rec.start_q)
        back = (rec.q_enc_limit if rec.q_enc_limit is not None else q_now) - d * self.backoff
        restore_dist = abs(q_now - rec.start_q)
        restore_dur = max(
            self.restore_sec,
            restore_dist / max(self.seek_vel, 0.2) + 0.8,
        )
        self.log(
            f"[{name}] 离开限位：先退到 {back:.4f}，再回到起点 {rec.start_q:.4f} "
            f"(当前 {q_now:.4f}，约 {restore_dist / DEG:.1f}°)"
        )
        # 即使 Ctrl+C，也要先离开硬限位再回起点，不能因 _stopping 跳过复位
        self._move_name(
            name,
            back,
            duration=max(0.5, abs(self.backoff) / max(self.seek_vel, 0.2) + 0.3),
            ignore_stop=True,
        )
        self._move_name(name, rec.start_q, duration=restore_dur, ignore_stop=True)
        rec.restored_q = self.q.get(name, rec.start_q)
        if rec.status == "limit":
            rec.status = "ok"
        restore_target = rec.start_q
        # 测完后：退出卸力列表，位控固定在恢复位置
        if self.group_cfg.get("hold_after_restore") or name in self.unload_until_sought:
            self.unload_until_sought.discard(name)
            self.hold_q[name] = restore_target
            self.hold_names.add(name)
            self.hold_loop(0.3)
            if name in ("r_ankle_roll_joint", "r_ankle_pitch_joint"):
                self.log(f"[{name}] 已固定在恢复位置 {restore_target:.4f} rad")
        err_deg = abs(rec.restored_q - rec.start_q) / DEG
        self.log(
            f"[{name}] 复位 → {rec.restored_q:.4f} rad "
            f"(目标 {rec.start_q:.4f}, 误差 {err_deg:.1f}°, status={rec.status})"
        )

    def _move_name(
        self,
        name: str,
        target: float,
        duration: float,
        *,
        ignore_stop: bool = False,
    ) -> None:
        self._move_names({name: target}, duration, ignore_stop=ignore_stop)

    def _move_names(
        self,
        targets: Dict[str, float],
        duration: float,
        *,
        ignore_stop: bool = False,
    ) -> None:
        if not targets:
            return
        starts = {n: self.q.get(n, t) for n, t in targets.items()}
        max_dist = max(abs(targets[n] - starts[n]) for n in targets)
        duration = max(duration, max_dist / max(self.seek_vel, 0.2) + 0.5)
        steps = max(1, int(duration * self.rate_hz))
        dt = 1.0 / self.rate_hz
        moving = list(targets.keys())
        for i in range(steps + 1):
            if not rclpy.ok():
                break
            if self._stopping and not ignore_stop:
                break
            t = i / steps
            s = 3 * t * t - 2 * t * t * t
            for name, target in targets.items():
                self.hold_q[name] = starts[name] + (target - starts[name]) * s
            self.publish_targets(self.hold_q, moving=moving)
            self.spin_for(dt)
        for name, target in targets.items():
            self.hold_q[name] = target
        settle_t0 = time.monotonic()
        settle_budget = max(4.0, max_dist / max(self.seek_vel, 0.2) + 1.0)
        while rclpy.ok():
            if self._stopping and not ignore_stop:
                break
            pending = [
                n for n, tgt in targets.items()
                if abs(self.q.get(n, tgt) - tgt) >= 5.0 * DEG
            ]
            if not pending:
                break
            if time.monotonic() - settle_t0 > settle_budget:
                for n in pending:
                    self.warn(
                        f"[{n}] 目标 {targets[n]:.4f} 实际 {self.q.get(n, targets[n]):.4f} rad，未到位"
                    )
                break
            self.publish_targets(self.hold_q, moving=moving)
            self.spin_for(dt)

    def _prep_delta(self, name: str, delta_deg: float) -> float:
        if name == "r_hip_roll_joint" and self.args.flip_hip_roll:
            delta_deg = -delta_deg
        if name == "waist_yaw_joint" and self.args.flip_waist:
            delta_deg = -delta_deg
        return delta_deg

    def _iter_prep_batches(self) -> List[List[Dict[str, Any]]]:
        batches: List[List[Dict[str, Any]]] = []
        for item in self.group_cfg.get("prep", ()):
            if isinstance(item, dict):
                batches.append([item])
            else:
                batches.append(list(item))
        return batches

    def _flatten_prep_steps(self) -> List[Dict[str, Any]]:
        steps: List[Dict[str, Any]] = []
        for batch in self._iter_prep_batches():
            steps.extend(batch)
        return steps

    def run_prep(self) -> None:
        for batch in self._iter_prep_batches():
            planned: Dict[str, float] = {}
            starts: Dict[str, float] = {}
            deltas: Dict[str, float] = {}
            hold_flags: Dict[str, bool] = {}
            for step in batch:
                name = step["name"]
                delta_deg = self._prep_delta(name, float(step["delta_deg"]))
                if name not in self.ctrl_names:
                    raise RuntimeError(f"预备关节未被接管: {name}")
                delta = delta_deg * DEG
                start = self.q.get(name, 0.0)
                target = start + delta
                if name in self.run_origin:
                    self.prep_origin[name] = self.run_origin[name]
                else:
                    self.prep_origin[name] = start
                hold_after = bool(step.get("hold_after", True))
                hold_flags[name] = hold_after
                starts[name] = start
                deltas[name] = delta_deg
                planned[name] = target
                after = "到位后保持" if hold_after else "到位后卸力"
                self.log(
                    f"预备 {step.get('note', name)}: {start:.4f} → {target:.4f} rad "
                    f"({delta_deg:+.0f}°), {after}，τ={self.tau_protect_of(name):.1f} N·m "
                    f"(启动角 {self.prep_origin[name]:.4f})"
                )
                self.hold_names.add(name)
            if len(batch) > 1:
                self.log(
                    "  ↑ 以上 "
                    + " / ".join(step.get("note", step["name"]) for step in batch)
                    + " 同时运动"
                )
            max_delta = max(abs(d) for d in deltas.values()) if deltas else 0.0
            self._move_names(
                planned,
                duration=max(2.5, max_delta * DEG / max(self.seek_vel, 0.2) + 0.5),
            )
            for name, target in planned.items():
                self.hold_q[name] = target
                actual = self.q.get(name, target)
                self.log(
                    f"  {name} 实际 {actual:.4f} rad，相对起点 "
                    f"{(actual - starts[name]) / DEG:+.1f}° (目标 {deltas[name]:+.0f}°)"
                )
                if hold_flags[name]:
                    pass
                else:
                    self.hold_names.discard(name)
                    self.log(f"  {name} 已卸力")
            self.hold_loop(0.4)

    def restore_prep(self) -> None:
        if not self.group_cfg.get("restore_prep"):
            return
        hold_map = {
            step["name"]: bool(step.get("hold_after", True))
            for step in self._flatten_prep_steps()
        }
        seen = set()
        # 预备批次逆序：同批内同时回到启动角（踝 pitch / 膝 calf 一起）
        for batch in reversed(self._iter_prep_batches()):
            if not rclpy.ok():
                break
            planned: Dict[str, float] = {}
            for step in batch:
                name = step["name"]
                origin = self.run_origin.get(name)
                if origin is None:
                    continue
                planned[name] = origin
                seen.add(name)
                if name in self.unload_until_sought:
                    self.hold_names.discard(name)
                else:
                    self.hold_names.add(name)
                self.log(f"复位 {name} → 启动角 {origin:.4f} rad")
            if not planned:
                continue
            if len(planned) > 1:
                self.log(
                    "  ↑ 以上 "
                    + " / ".join(planned.keys())
                    + " 同时复位"
                )
            self._move_names(planned, duration=2.0, ignore_stop=True)
            for name, origin in planned.items():
                self.hold_q[name] = origin
                still_unload = name in self.unload_until_sought
                if still_unload or not hold_map.get(name, True):
                    self.hold_names.discard(name)
                    self.log(f"  {name} 复位后卸力")
            self.hold_loop(0.3)

        # 其余腿轴（未在预备里的）逐个回启动角
        for name in reversed(list(self.group_joints)):
            if not rclpy.ok():
                break
            if name in seen:
                continue
            origin = self.run_origin.get(name)
            if origin is None:
                continue
            self.log(f"复位 {name} → 启动角 {origin:.4f} rad")
            still_unload = name in self.unload_until_sought
            if still_unload:
                self.hold_names.discard(name)
            else:
                self.hold_names.add(name)
            self._move_name(
                name,
                origin,
                duration=max(
                    2.0,
                    abs(self.q.get(name, origin) - origin) / max(self.seek_vel, 0.2) + 0.5,
                ),
                ignore_stop=True,
            )
            self.hold_q[name] = origin
            if still_unload or not hold_map.get(name, True):
                self.hold_names.discard(name)
                self.hold_loop(0.3)
                self.log(f"  {name} 复位后卸力")
            else:
                self.hold_loop(0.3)

    def write_yaml(self) -> None:
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lines: List[str] = [
            self.scan_begin,
            f"# 生成时间: {stamp}",
            "# 仅记录撞到硬件限位时的 URDF 位置 [rad]",
            f"{self.yaml_key}:",
        ]
        n_header = len(lines)
        for name in self.seek_names:
            rec = self.records.get(name)
            if rec is None or rec.q_enc_limit is None:
                continue
            lines.append(f"  {name}: {fmt(rec.q_enc_limit)}")
        if len(lines) == n_header:
            lines.append("  {}")
        lines.append(self.scan_end)
        block = "\n".join(lines) + "\n"

        existing = ""
        if os.path.isfile(self.yaml_path):
            with open(self.yaml_path, "r", encoding="utf-8") as f:
                existing = f.read()
        if self.scan_begin in existing and self.scan_end in existing:
            pre = existing[: existing.index(self.scan_begin)]
            post = existing[existing.index(self.scan_end) + len(self.scan_end) :]
            text = pre.rstrip() + "\n\n" + block + post.lstrip("\n")
        else:
            text = existing.rstrip() + "\n\n" + block
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            f.write(text)
        self.log(f"已写入 {self.yaml_path}")

    def run(self) -> int:
        if not self.wait_js():
            self.warn("未收到 /joint_states，请先启动 pi_plus_orin / midware")
            return 1
        missing = [n for n in self.args.joints if n not in self.js_names]
        if missing:
            raise RuntimeError("关节不存在: " + ", ".join(missing))

        if self.args.dry_run:
            self.log("dry-run: 不出力、不申请控制权")
            for step in self._flatten_prep_steps():
                self.log(f"  预备 {step.get('note', step['name'])}  Δ={step['delta_deg']:+.0f}°")
            for name in self.args.joints:
                mid = self.motor_id(name)
                d = self.seek_dir(name)
                q0 = self.q.get(name, float("nan"))
                self.log(
                    f"  {name} id={mid} q={q0:.4f} URDF_dir={d:+.0f} "
                    f"τ_protect={self.tau_protect_of(name):.2f}"
                )
            return 0

        self.request_group()
        try:
            for name in self.ctrl_names:
                self.hold_q[name] = self.q.get(name, 0.0)
            self.hold_loop(0.4)
            # 最终复位基准：程序启动时读到的位置
            self.run_origin = {
                n: float(self.q.get(n, 0.0)) for n in self.ctrl_names
            }
            self.log(
                "已记录启动姿态（最终复位基准）: "
                + ", ".join(
                    f"{n}={self.run_origin[n]:.3f}"
                    for n in list(self.group_joints)[:3]
                )
                + ", ..."
            )
            for n in self.unload_until_sought:
                self.hold_names.discard(n)
            if self.unload_until_sought:
                self.log(
                    "抬腿前卸力、测完后固定: "
                    + ", ".join(sorted(self.unload_until_sought))
                )
            self.run_prep()

            for name in self.seek_names:
                rec = JointRecord(
                    name=name,
                    motor_id=self.motor_id(name),
                    seek_dir_urdf=self.seek_dir(name),
                    tau_protect=self.tau_protect_of(name),
                    tau_abort=self.tau_abort_of(name),
                )
                self.records[name] = rec
                self.log(f"==== {name} 顺时针寻限位 ====")
                try:
                    self.seek_one(rec)
                except Exception as exc:
                    rec.status = "fail"
                    rec.note = str(exc)
                    self.warn(f"[{name}] 异常: {exc}")
                    try:
                        target = rec.start_q or self.q.get(name, 0.0)
                        self._move_name(name, target, 1.5, ignore_stop=True)
                        if self.group_cfg.get("hold_after_restore"):
                            self.unload_until_sought.discard(name)
                            self.hold_q[name] = target
                            self.hold_names.add(name)
                    except Exception:
                        pass
                if self._stopping:
                    self.warn("已中断：完成本轴返回后停止后续关节")
                    break
                self.hold_loop(0.3)
            self.restore_prep()
            return 0
        finally:
            try:
                self.write_yaml()
            except Exception as exc:
                self.warn(f"写 yaml 失败: {exc}")
            self.release_group()


def median(xs: Sequence[float]) -> float:
    ys = sorted(xs)
    n = len(ys)
    if n == 0:
        return float("nan")
    if n % 2:
        return ys[n // 2]
    return 0.5 * (ys[n // 2 - 1] + ys[n // 2])


def fmt(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "null"
    return f"{v:.6f}"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="限力矩 1 rad/s 顺时针寻硬限位后复位")
    p.add_argument("--group", choices=sorted(GROUPS), default="right_arm")
    args_pre, _ = p.parse_known_args(argv)
    group_joints = list(GROUPS[args_pre.group]["joints"])

    p.add_argument("--joints", nargs="+", default=group_joints, help="要测的关节，默认按组远端先")
    p.add_argument("--tau-protect", type=float, default=None, help="覆盖组内所有轴保护力矩 N·m")
    p.add_argument("--tau-abort", type=float, default=8.0, help="中止力矩下限 N·m；各轴会自动抬到 protect+2")
    p.add_argument("--kp", type=float, default=8.0)
    p.add_argument("--kd", type=float, default=0.6)
    p.add_argument("--seek-vel", type=float, default=None, help="斜坡速度 rad/s；默认用手部 2 / 其它 1")
    p.add_argument("--min-travel-deg", type=float, default=5.0)
    p.add_argument("--max-travel-deg", type=float, default=350.0)
    p.add_argument("--stall-vel", type=float, default=0.02)
    p.add_argument("--stall-hold", type=float, default=0.2)
    p.add_argument("--bias-sec", type=float, default=0.3)
    p.add_argument("--backoff-deg", type=float, default=4.0)
    p.add_argument("--restore-sec", type=float, default=3.0)
    p.add_argument("--seek-timeout", type=float, default=45.0)
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--yaml", default=DEFAULT_YAML)
    p.add_argument("--ccw", action="store_true", help="改为电机逆时针")
    p.add_argument("--flip-waist", action="store_true", help="预备腰方向再反过来")
    p.add_argument("--flip-hip-roll", action="store_true", help="预备髋 roll 方向再反过来")
    p.add_argument("--takeover", action="store_true", help="停掉 hightorque_controller 以拿到电机")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    allowed = GROUPS[args.group]["dir"]
    unknown = [n for n in args.joints if n not in allowed]
    if unknown:
        print(f"不是组 {args.group} 的关节: " + ", ".join(unknown), file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = CwLimit(args)

    def _sig(_signo, _frame):
        node._stopping = True
        node.warn("收到中断，将复位并卸力")

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        return node.run()
    except Exception as exc:
        node.warn(str(exc))
        try:
            node.write_yaml()
        except Exception:
            pass
        node.release_group()
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
