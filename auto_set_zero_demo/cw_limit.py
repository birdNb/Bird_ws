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
import re
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

# 固定参考限位（standing_pose.yaml 中不覆盖）
LIMIT_REF_KEYS = (
    "right_arm_limit_q",
    "waist_neck_limit_q",
    "right_leg_limit_q",
)


def load_limit_refs(yaml_path: str) -> Dict[str, float]:
    """读取 standing_pose.yaml 中固定参考限位（不依赖 PyYAML）。"""
    out: Dict[str, float] = {}
    if not os.path.isfile(yaml_path):
        return out
    with open(yaml_path, "r", encoding="utf-8") as f:
        text = f.read()
    for key in LIMIT_REF_KEYS:
        m = re.search(
            rf"(?m)^{re.escape(key)}:\s*\n((?:^[ \t]+.*\n)*)",
            text,
        )
        if not m:
            continue
        for line in m.group(1).splitlines():
            mm = re.match(
                r"^\s+([A-Za-z0-9_]+):\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?\d+)?)",
                line,
            )
            if mm:
                out[mm.group(1)] = float(mm.group(2))
    return out


# 并行标定：各车道关节顺序（组内仍远端→近端串行；车道之间同时）
PARALLEL_LANES: Dict[str, Tuple[str, ...]] = {
    "arm": (
        "r_elbow_joint",
        "r_upper_arm_joint",
        "r_shoulder_roll_joint",
        "r_shoulder_pitch_joint",
    ),
    "head": (
        "head_pitch_joint",
        "head_yaw_joint",
    ),
    "leg": (
        "r_ankle_roll_joint",
        "r_ankle_pitch_joint",
        "r_calf_joint",
        "r_thigh_joint",
        "r_hip_roll_joint",
        "r_hip_pitch_joint",
    ),
}
PARALLEL_WAIST = "waist_yaw_joint"
# 与预备腰右转同向（URDF 减小）；手脚标定完后继续向右撞限位
PARALLEL_WAIST_SEEK_DIR = -1.0
PARALLEL_YAML_GROUPS = ("right_arm", "waist_neck", "right_leg")
# 计算零位阶段：先只动腰+上半身（腿保持标定初始姿态，左半身标准零位不动）
UPPER_BODY_ZERO_JOINTS: Tuple[str, ...] = (
    "r_elbow_joint",
    "r_upper_arm_joint",
    "r_shoulder_roll_joint",
    "r_shoulder_pitch_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
)
# 左半身：右半身标定限位时保持卸力（kp/kd 极小 τ，不位控）
LEFT_BODY_JOINTS: Tuple[str, ...] = (
    "l_ankle_roll_joint",
    "l_ankle_pitch_joint",
    "l_calf_joint",
    "l_thigh_joint",
    "l_hip_roll_joint",
    "l_hip_pitch_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
)

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
            "r_shoulder_roll_joint": 3.0,
            "r_shoulder_pitch_joint": 2.0,
        },
        "force_tau_limit_sec": {
            "r_shoulder_roll_joint": 2.0,
            "r_shoulder_pitch_joint": 2.0,
        },
        "seek_vel": 2.0,
        "restore_vel": 2.5,
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
            # 与预备「腰右转 90°」同向：继续向右撞硬限位
            "waist_yaw_joint": -1.0,
        },
        "tau": {
            "head_pitch_joint": 1.0,
            "head_yaw_joint": 1.0,
            "waist_yaw_joint": 1.0,
        },
        "yaml_key": "waist_neck_limit_q",
        "seek_vel": 1.5,
        "restore_vel": 2.5,
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
            "r_ankle_pitch_joint": 1.0,
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
        # 预备：腰 → (踝 pitch -15° 与膝 +15° 同时) → 髋抬；踝 roll 测前卸力
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
                    "delta_deg": -15.0,
                    "note": "右踝 pitch -15°",
                    "hold_after": True,
                },
                {
                    "name": "r_calf_joint",
                    "delta_deg": 15.0,
                    "note": "右膝(calf)弯曲 +15°",
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
        # 踝 roll 抬腿前卸力；测完复位后固定。踝 pitch 抬腿前已弯到 -15° 并保持
        "unload_until_sought": ("r_ankle_roll_joint",),
        # 寻限位时：组内腿轴位控固定；组外（臂/头/左腿/腰）卸力
        "idle_mode": "leg_hold",
        # 覆盖默认满力矩强制返回时间（秒）；未列出的轴用 --force-tau-sec
        "force_tau_limit_sec": {
            "r_hip_roll_joint": 1.0,
            "r_thigh_joint": 1.0,
        },
        "hold_after_restore": True,
        "restore_prep": True,
        "seek_vel": 1.5,
        "restore_vel": 2.5,
        "yaml_key": "right_leg_limit_q",
        "scan_begin": "# === right_leg_limit_scan (generated) ===",
        "scan_end": "# === end right_leg_limit_scan ===",
    },
}

# 跨组关节元数据（并行模式查方向/力矩）
JOINT_META: Dict[str, Dict[str, Any]] = {}
for _gname, _g in GROUPS.items():
    for _j, _d in _g["dir"].items():
        tau = float(_g["tau"].get(_j, 2.0))
        force_map = _g.get("force_tau_limit_sec", {}) or {}
        force = float(force_map[_j]) if isinstance(force_map, dict) and _j in force_map else None
        if _j not in JOINT_META:
            JOINT_META[_j] = {
                "group": _gname,
                "dir": float(_d),
                "tau": tau,
                "force_tau": force,
                "seek_vel": float(_g.get("seek_vel", 1.0)),
            }
        else:
            JOINT_META[_j]["tau"] = max(float(JOINT_META[_j]["tau"]), tau)
            if force is not None:
                JOINT_META[_j]["force_tau"] = force


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
        # 撞限位后/预备复位用更快速度，避免跟寻限位一样慢
        self.restore_vel = float(
            args.restore_vel
            if getattr(args, "restore_vel", None) is not None
            else self.group_cfg.get("restore_vel", max(self.seek_vel, 2.5))
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
        self.group_name = str(args.group)
        self.scan_begin = self.group_cfg["scan_begin"]
        self.scan_end = self.group_cfg["scan_end"]
        self.yaml_key = self.group_cfg["yaml_key"]
        self.force_tau_sec_default = float(
            getattr(args, "force_tau_sec", 1.0) or 1.0
        )
        self.chain_hold = bool(getattr(args, "groups", None)) or bool(
            getattr(args, "parallel", False)
        )
        self.parallel_mode = bool(getattr(args, "parallel", False))
        # 并行时累计各轴结果，按组写 yaml
        self.all_records: Dict[str, JointRecord] = {}
        self.limit_ref: Dict[str, float] = load_limit_refs(self.yaml_path)
        self.write_zero = bool(getattr(args, "write_zero", True))
        self.update_ref = bool(getattr(args, "update_ref", False))
        self.zero_motor_ids: List[int] = []
        self.home_q: Dict[str, float] = {}

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
        self.left_body_unload: set = set()
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
        try:
            from hightorque_msgs.srv import ResetZero

            self.cli_reset = self.create_client(ResetZero, "reset_zero")
            self._ResetZero = ResetZero
        except Exception:
            self.cli_reset = None
            self._ResetZero = None

    def setup_left_body_unload(self) -> None:
        """左半身全程卸力，右臂/右腿标定时不位控。"""
        self.left_body_unload = set()
        names: List[str] = []
        for name in LEFT_BODY_JOINTS:
            if name not in self.ctrl_names:
                self.warn(f"[左半身] {name} 不在控制列表，跳过")
                continue
            self.left_body_unload.add(name)
            self.hold_names.discard(name)
            names.append(name)
        if not names:
            self.warn("左半身无可用轴，无法卸力")
            return
        self.log(
            f"左半身卸力 {len(names)} 轴（标定右半身时保持零力矩）"
        )
        self.hold_loop(0.35)

    def refresh_left_body_unload(self) -> None:
        """防止其它步骤把左半身重新加入 hold。"""
        for name in self.left_body_unload:
            self.hold_names.discard(name)

    def remember_zero_joint(self, name: str, q_home: float) -> None:
        if getattr(self.args, "no_write_zero", False):
            return
        mid = self.motor_id(name)
        self.home_q[name] = float(q_home)
        if mid not in self.zero_motor_ids:
            self.zero_motor_ids.append(mid)

    def home_from_limit(self, name: str, q_limit_meas: float) -> Optional[float]:
        """q_home = q_meas_limit - q_ref_limit；结果拨到靠近 0 的连续支路。"""
        if name not in self.limit_ref:
            return None
        q_ref = float(self.limit_ref[name])
        q_meas = float(q_limit_meas)
        d = self.seek_dir(name)
        # 参考限位若明显超出结构行程，多半是 ±π 连续角折叠；收回到 seek 侧结构极限
        q_ref = self._canonical_limit_ref(name, q_ref, d)
        # 参考限位与实测限位放到同一连续支路，再相减
        q_ref_c = unwrap_near(q_ref, q_meas)
        q_home = q_meas - q_ref_c
        # 标定正确时 home≈0；拨到 [-π,π] 便于对照左半身标准零位
        q_home = unwrap_near(q_home, 0.0)
        if abs(q_home) > 25.0 * DEG:
            self.warn(
                f"[{name}] 计算零位偏离 0 达 {q_home / DEG:+.1f}° "
                f"(实测限位 {q_meas:.4f}, 参考 {q_ref:.2f})；"
                "请对照左半身标准零位，必要时用 --update-ref 重采参考限位"
            )
        return q_home

    def _canonical_limit_ref(self, name: str, q_ref: float, seek_dir: float) -> float:
        """把明显越界的 yaml 参考限位收到结构行程内（同 seek 方向）。"""
        # 与 standing_pose.yaml structural_limits_sim_ref_deg 对齐
        abs_lim = {
            "r_elbow_joint": 110.0 * DEG,
            "r_upper_arm_joint": 150.0 * DEG,
            "r_shoulder_pitch_joint": 150.0 * DEG,
            "r_shoulder_roll_joint": 180.0 * DEG,
            "head_pitch_joint": 90.0 * DEG,
            "head_yaw_joint": 90.0 * DEG,
            "waist_yaw_joint": 180.0 * DEG,
            "r_hip_pitch_joint": 150.0 * DEG,
            "r_hip_roll_joint": 180.0 * DEG,
            "r_thigh_joint": 165.0 * DEG,
            "r_calf_joint": 143.0 * DEG,
            "r_ankle_pitch_joint": 56.0 * DEG,
            "r_ankle_roll_joint": 45.0 * DEG,
        }.get(name)
        if abs_lim is None:
            return float(q_ref)
        if abs(q_ref) <= abs_lim + 8.0 * DEG:
            return float(q_ref)
        side = 1.0 if float(seek_dir) >= 0.0 else -1.0
        q_adj = side * abs_lim
        self.warn(
            f"[{name}] 参考限位 {q_ref:.2f} 超出结构约 ±{abs_lim:.2f}，"
            f"按 seek_dir 收为 {q_adj:.2f}"
        )
        return float(q_adj)

    def upper_body_homes(self) -> Dict[str, float]:
        return {
            n: q
            for n, q in self.home_q.items()
            if n in UPPER_BODY_ZERO_JOINTS
        }

    def upper_body_zero_ids(self) -> List[int]:
        ids: List[int] = []
        for name in UPPER_BODY_ZERO_JOINTS:
            if name not in self.home_q:
                continue
            mid = self.motor_id(name)
            if mid not in ids:
                ids.append(mid)
        return ids

    def move_joints_to_homes(
        self,
        homes: Dict[str, float],
        *,
        note: str = "",
    ) -> None:
        """逐轴移到计算零位并核对到位（避免多轴同时动互相顶住）。"""
        if note:
            self.log(note)
        rvel = max(self.restore_vel, 0.2)
        for name in UPPER_BODY_ZERO_JOINTS:
            if name not in homes:
                continue
            if name not in self.ctrl_names:
                continue
            target = homes[name]
            d = -float(JOINT_META.get(name, {}).get("dir", self.seek_dir(name)))
            q0 = self.q.get(name, target)
            tgt = align_goal_along_dir(q0, target, d)
            self.log(
                f"[{name}] → 计算零位 {tgt:.4f} rad "
                f"(约 {tgt / DEG:+.1f}°, 从 {q0:.4f})"
            )
            self.hold_names.add(name)
            self.unload_until_sought.discard(name)
            dist = abs(tgt - q0)
            self._move_name(
                name,
                tgt,
                duration=max(1.2, dist / rvel + 0.5),
                ignore_stop=True,
                vel=rvel,
                path_dir=d,
            )
            self.hold_q[name] = tgt
            self.hold_loop(0.35)
            q_fb = self.q.get(name, tgt)
            err = abs(unwrap_near(q_fb, tgt) - tgt)
            if err > 8.0 * DEG:
                self.warn(
                    f"[{name}] 未到位: 目标 {tgt:.4f} 反馈 {q_fb:.4f} "
                    f"误差 {err / DEG:.1f}°"
                )
            else:
                self.log(
                    f"[{name}] 已到位: 反馈 {q_fb:.4f} rad "
                    f"(误差 {err / DEG:.1f}°)"
                )
            # 更新记录为实际指令目标，供写零提示
            self.home_q[name] = tgt

    def prompt_write_zero(self, homes: Optional[Dict[str, float]] = None) -> bool:
        """打印计算零位状态，再询问是否一键写零。"""
        show = homes if homes is not None else self.home_q
        print("", flush=True)
        print("=" * 60, flush=True)
        print("  腰+上半身已到【计算零位】（非电机真零；腿/左半身未改）", flush=True)
        print("=" * 60, flush=True)
        for name in UPPER_BODY_ZERO_JOINTS:
            if name not in show:
                continue
            q_cmd = show[name]
            q_fb = self.q.get(name, float("nan"))
            mid = self.motor_id(name)
            err = abs(unwrap_near(q_fb, q_cmd) - q_cmd) / DEG
            flag = "OK" if err < 8.0 else "偏差大"
            print(
                f"  {name:28s}  id={mid:2d}  "
                f"计算零位={q_cmd:+.4f}  反馈={q_fb:+.4f} rad "
                f"({q_fb / DEG:+.1f}°)  [{flag}]",
                flush=True,
            )
        print("=" * 60, flush=True)
        print("  若确认姿态接近左半身镜像/标准站姿，可对上述电机一键写零", flush=True)
        print("  （当前角 → 电机 0 并写 Flash；腿与左半身不写）", flush=True)
        print("=" * 60, flush=True)
        if getattr(self.args, "no_write_zero", False):
            print("已指定 --no-write-zero，跳过写零", flush=True)
            return False
        if getattr(self.args, "yes", False):
            print("已指定 --yes，自动确认写零", flush=True)
            return True
        while True:
            try:
                ans = input("是否对【腰+上半身】执行电机一键写零？[y/N]: ").strip().lower()
            except EOFError:
                print("无交互输入，跳过写零", flush=True)
                return False
            if ans in ("y", "yes"):
                return True
            if ans in ("n", "no", ""):
                return False
            print("请输入 y 或 n", flush=True)

    def move_hold_pose(
        self,
        targets: Dict[str, float],
        *,
        path_dir: Optional[Dict[str, float]] = None,
        note: str = "",
    ) -> None:
        if not targets:
            return
        if note:
            self.log(note)
        rvel = max(self.restore_vel, 0.2)
        planned = dict(targets)
        dirs = dict(path_dir or {})
        for n, t in list(planned.items()):
            qn = self.q.get(n, t)
            if n in dirs:
                planned[n] = align_goal_along_dir(qn, t, dirs[n])
            else:
                planned[n] = unwrap_near(t, qn)
        max_dist = max(abs(self.q.get(n, t) - t) for n, t in planned.items())
        for n in planned:
            self.hold_names.add(n)
            self.unload_until_sought.discard(n)
        self._move_names(
            planned,
            duration=max(1.5, max_dist / rvel + 0.5),
            ignore_stop=True,
            vel=rvel,
            path_dir=dirs or None,
        )
        for n, t in planned.items():
            self.hold_q[n] = t
        self.hold_loop(0.4)

    def call_reset_zero(self, motor_ids: Sequence[int]) -> bool:
        if not motor_ids:
            self.warn("没有需要写零的电机")
            return False
        if self.cli_reset is None or self._ResetZero is None:
            self.warn("ResetZero 服务接口不可用，请先编译 hightorque_msgs / midware")
            return False
        if not self.cli_reset.wait_for_service(timeout_sec=3.0):
            self.warn("/reset_zero 服务不可用")
            return False
        req = self._ResetZero.Request()
        req.timeout_ms = 60000
        if not hasattr(req, "motor_ids"):
            self.warn(
                "当前 /reset_zero 不支持 motor_ids（需重编译 hightorque_msgs + midware）。"
                "已走到标定0位，但未写零。"
            )
            return False
        req.motor_ids = [int(x) for x in motor_ids]
        self.log(
            "调用 /reset_zero，电机索引: "
            + ", ".join(str(i) for i in req.motor_ids)
        )
        try:
            resp = self.call_srv(self.cli_reset, req, timeout=70.0)
        except Exception as exc:
            self.warn(f"/reset_zero 调用失败: {exc}")
            return False
        ok = bool(getattr(resp, "success", False))
        msg = getattr(resp, "message", "")
        if ok:
            self.log(f"写零成功: {msg}")
        else:
            self.warn(f"写零失败: {msg}")
        return ok

    def apply_group(self, group: str) -> None:
        """切换当前扫描组（同一次控制权内，避免组间卸力卡顿）。"""
        if group not in GROUPS:
            raise RuntimeError(f"未知组: {group}")
        self.group_name = group
        self.group_cfg = GROUPS[group]
        self.group_joints = self.group_cfg["joints"]
        self.scan_begin = self.group_cfg["scan_begin"]
        self.scan_end = self.group_cfg["scan_end"]
        self.yaml_key = self.group_cfg["yaml_key"]
        if self.args.seek_vel is None:
            self.seek_vel = float(self.group_cfg.get("seek_vel", 1.0))
        if getattr(self.args, "restore_vel", None) is None:
            self.restore_vel = float(
                self.group_cfg.get("restore_vel", max(self.seek_vel, 2.5))
            )
        # 多组串行时测整组；单组仍尊重 --joints
        if self.chain_hold:
            self.seek_names = list(self.group_joints)
        else:
            self.seek_names = [n for n in self.group_joints if n in self.args.joints]
        self.unload_until_sought = set(self.group_cfg.get("unload_until_sought", ()))
        self.records = {}
        self.prep_origin = {}
        # 组间保持全身位姿，仅预备卸力的轴临时放开
        for name in self.ctrl_names:
            self.hold_q[name] = float(self.q.get(name, self.hold_q.get(name, 0.0)))
            self.hold_names.add(name)
        for n in self.unload_until_sought:
            self.hold_names.discard(n)

    def force_tau_sec_of(self, name: str) -> float:
        cfg = self.group_cfg.get("force_tau_limit_sec", {})
        if isinstance(cfg, dict) and name in cfg:
            return float(cfg[name])
        meta = JOINT_META.get(name, {})
        if meta.get("force_tau") is not None:
            return float(meta["force_tau"])
        return float(self.force_tau_sec_default)

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
        if name in self.group_cfg.get("dir", {}):
            base = float(self.group_cfg["dir"][name])
        elif name in JOINT_META:
            base = float(JOINT_META[name]["dir"])
        else:
            raise RuntimeError(f"未知关节方向: {name}")
        return base if self.clockwise else -base

    def tau_protect_of(self, name: str) -> float:
        if self.args.tau_protect is not None:
            return float(self.args.tau_protect)
        if name in self.group_cfg.get("tau", {}):
            return float(self.group_cfg["tau"][name])
        if name in JOINT_META:
            return float(JOINT_META[name]["tau"])
        return 2.0

    def seek_vel_of(self, name: str) -> float:
        if self.args.seek_vel is not None:
            return float(self.args.seek_vel)
        if name in JOINT_META:
            return float(JOINT_META[name].get("seek_vel", self.seek_vel))
        return float(self.seek_vel)

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

    def maybe_stop_controller(self, need_names: Optional[Sequence[str]] = None) -> None:
        if not self.args.takeover:
            return
        check = list(need_names) if need_names is not None else list(self.seek_names)
        self.warn(" --takeover: 停止 hightorque_controller_node，释放电机")
        subprocess.call(
            ["pkill", "-INT", "-f", "hightorque_controller_node"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:
            self.spin_for(0.3)
            ok, _ = self.motors_idle(check)
            if ok:
                self.log("控制器已释放目标电机")
                return
        self.warn("等待释放超时，仍尝试申请控制权")

    def request_group(self, need_names: Optional[Sequence[str]] = None) -> None:
        if need_names is not None:
            check_names = list(need_names)
            # 并行/多组传入的全量关节；勿再用 group∩--joints（两组交集常为空）
            if not self.seek_names:
                self.seek_names = list(need_names)
        else:
            check_names = list(self.seek_names)
            if not self.seek_names:
                self.seek_names = [n for n in self.group_joints if n in self.args.joints]
                check_names = list(self.seek_names)
        if not check_names:
            raise RuntimeError("没有要测试的关节")
        idle, msg = self.motors_idle(check_names)
        if not idle:
            self.warn(msg)
            self.maybe_stop_controller(check_names)
            idle, msg = self.motors_idle(check_names)
            if not idle and not self.args.takeover:
                raise RuntimeError(msg + "。先卸力后停控制器，或加 --takeover")

        get_req = GetAvailableMotors.Request()
        get_req.node_name = ""
        idle_resp = self.call_srv(self.cli_get, get_req)
        self.ctrl_names = list(idle_resp.motor_names)
        self.motor_ids = list(idle_resp.motor_ids)
        missing = [n for n in check_names if n not in self.ctrl_names]
        if missing:
            raise RuntimeError("目标关节仍被占用: " + ", ".join(missing))
        if not self.ctrl_names:
            raise RuntimeError("没有空闲电机")

        # 多组串行全程位控保持，超时默认阻尼以免短暂掉指令时塌软
        idle_mode = self.group_cfg.get("idle_mode", "unload")
        use_damp_default = idle_mode == "damping" or self.chain_hold
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
        if self.chain_hold:
            rest = "多组串行保持全身"
        elif idle_mode == "leg_hold":
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
        seeking: Optional[Any] = None,
        moving: Optional[Any] = None,
        move_vel: Optional[float] = None,
        seek_vels: Optional[Dict[str, float]] = None,
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
        default_seek_vel = abs(self.seek_vel)
        move_cmd_vel = abs(move_vel if move_vel is not None else self.restore_vel)
        idle_mode = self.group_cfg.get("idle_mode", "unload")
        leg_set = set(self.group_joints)
        if seeking is None:
            seeking_set: set = set()
        elif isinstance(seeking, str):
            seeking_set = {seeking}
        else:
            seeking_set = set(seeking)
        if moving is None:
            moving_set: set = set()
        elif isinstance(moving, str):
            moving_set = {moving}
        else:
            moving_set = set(moving)
        for name in self.ctrl_names:
            q_now = self.q.get(name, 0.0)
            raw = targets.get(name, q_now)
            if name in seeking_set:
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(abs(self.tau_protect_of(name)))
                sv = default_seek_vel
                if seek_vels and name in seek_vels:
                    sv = abs(seek_vels[name])
                velocities.append(sv)
                positions.append(float(raw))
            elif name in moving_set:
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(abs(self.tau_protect_of(name)))
                velocities.append(move_cmd_vel)
                positions.append(float(raw))
            elif name in self.unload_until_sought:
                # 尚未测完的踝等：卸力，方便离支架
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(0.05)
                velocities.append(0.0)
                positions.append(float(q_now))
            elif name in self.left_body_unload:
                # 左半身：全程零力矩，不位控
                kps.append(0.0)
                kds.append(0.0)
                torques.append(0.0)
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
            elif idle_mode == "damping" or self.parallel_mode or self.chain_hold:
                kps.append(self.kp)
                kds.append(self.kd)
                torques.append(abs(self.tau_protect_of(name)) if name in targets else 2.0)
                velocities.append(0.0)
                positions.append(float(raw if name in targets else q_now))
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
        force_tau_sec = self.force_tau_sec_of(name)
        # 起点已在硬限位时行程≈0，不能等 min_travel；稍推一下仍不动就认限位
        start_at_limit_sec = 0.45
        start_move_eps = 1.0 * DEG
        tau_full_t0: Optional[float] = None
        tau_below_t0: Optional[float] = None
        force_armed_logged = False

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
            moved = abs(q - rec.start_q)
            # 起点就顶着限位：推了 start_at_limit_sec 仍几乎不动 + 力矩/超前
            start_at_limit = (
                elapsed >= start_at_limit_sec
                and moved < start_move_eps
                and lead >= 0.06
                and (
                    at_protect
                    or abs(tau) >= rec.tau_protect * 0.85
                    or abs(tau_rel) >= rec.tau_protect * 0.85
                )
            )
            travel_ok = travel >= self.min_travel or start_at_limit

            # 满保护力矩持续 force_tau_sec → 强制返回（带 0.25s 回差，防力矩抖动重置计时）
            if force_tau_sec > 0.0 and travel_ok:
                if at_protect:
                    tau_below_t0 = None
                    if tau_full_t0 is None:
                        tau_full_t0 = now
                        if not force_armed_logged:
                            why = "起点已在限位" if start_at_limit else "已达保护力矩"
                            self.log(
                                f"[{name}] {why} |τ|={abs(tau):.2f}，"
                                f"{force_tau_sec:.1f}s 内强制记限位并返回"
                            )
                            force_armed_logged = True
                    elif now - tau_full_t0 >= force_tau_sec:
                        rec.q_enc_limit = median(list(window)) if window else q
                        rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                        rec.stall_tau = tau
                        rec.status = "limit"
                        rec.note = (
                            f"满力矩强制限位 {force_tau_sec:.1f}s |τ|={abs(tau):.2f} "
                            f"(保护 {rec.tau_protect:.2f}), 行程 {rec.q_travel_rad / DEG:.2f} deg"
                            + ("，起点已在限位" if start_at_limit else "")
                        )
                        self.log(
                            f"[{name}] 强制超时恢复: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
                        )
                        break
                elif tau_full_t0 is not None:
                    if tau_below_t0 is None:
                        tau_below_t0 = now
                    elif now - tau_below_t0 >= 0.25:
                        tau_full_t0 = None
                        tau_below_t0 = None
                        force_armed_logged = False

            # 起点已在限位：快速确认后立即返回，不必再等满 force_tau_sec
            if start_at_limit and (
                now - (freeze_t0 or now) >= self.stall_hold or elapsed >= start_at_limit_sec + 0.15
            ):
                rec.q_enc_limit = median(list(window)) if window else q
                rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                rec.stall_tau = tau
                rec.status = "limit"
                rec.note = (
                    f"起点已在硬件限位 |τ|={abs(tau):.2f} lead={lead:.3f} "
                    f"(保护 {rec.tau_protect:.2f}), 移动 {moved / DEG:.2f} deg"
                )
                self.log(
                    f"[{name}] 起点限位，开始返回: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
                )
                break

            # 以位置冻结判定为主；dq 噪声不再清掉冻结计时（否则会卡在限位不回）
            if freeze_t0 is None or freeze_q is None or abs(q - freeze_q) > freeze_eps:
                freeze_t0 = now
                freeze_q = q
                stall_t0 = None
                window.clear()
            else:
                window.append(q)
                frozen = (
                    travel_ok
                    and freeze_t0 is not None
                    and now - freeze_t0 >= self.stall_hold
                )
                # 指令在寻限位方向上超前实际角：电机跟不上才是真堵转。
                cmd_ahead = lead >= max(self.max_lead(name), 0.12)
                torque_hit = travel_ok and (
                    abs(tau_rel) >= rec.tau_protect * 0.92
                    or abs(tau) >= rec.tau_protect * 0.92
                    or (abs(tau) >= rec.tau_protect * 0.85 and cmd_ahead)
                    or (frozen and abs(tau) >= rec.tau_protect * 0.85 and lead >= 0.08)
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
        q_lim = float(rec.q_enc_limit if rec.q_enc_limit is not None else q_now)
        back = q_lim - d * self.backoff
        q_home = self.home_from_limit(name, q_lim)
        if q_home is not None:
            q_home_aligned = align_goal_along_dir(q_lim, q_home, -float(d))
            self.remember_zero_joint(name, q_home_aligned)
            self.log(
                f"[{name}] 已记录计算零位 {q_home_aligned:.4f} "
                f"(参考 {self.limit_ref[name]:.2f})；本轴先回寻限位起点"
            )
            restore_goal = rec.start_q
        else:
            start_was_limit = abs(q_lim - rec.start_q) < 1.5 * DEG
            restore_goal = back if start_was_limit else rec.start_q
            self.warn(f"[{name}] 无参考限位，回退目标 {restore_goal:.4f}")
        restore_dist = abs(q_now - restore_goal)
        rvel = max(self.restore_vel, 0.2)
        restore_dur = max(self.restore_sec, restore_dist / rvel + 0.3)
        self.log(
            f"[{name}] 离开限位：先退到 {back:.4f}，再回起点 {restore_goal:.4f} "
            f"(当前 {q_now:.4f}，原路 -seek_dir)"
        )
        rev = -float(d)
        self._move_name(
            name,
            back,
            duration=max(0.3, abs(self.backoff) / rvel + 0.15),
            ignore_stop=True,
            vel=rvel,
            path_dir=rev,
        )
        self._move_name(
            name,
            restore_goal,
            duration=restore_dur,
            ignore_stop=True,
            vel=rvel,
            path_dir=rev,
        )
        rec.restored_q = self.q.get(name, restore_goal)
        if rec.status == "limit":
            rec.status = "ok"
        restore_target = restore_goal
        if (
            self.chain_hold
            or self.group_cfg.get("hold_after_restore")
            or name in self.unload_until_sought
            or True
        ):
            self.unload_until_sought.discard(name)
            self.hold_q[name] = restore_target
            self.hold_names.add(name)
            self.hold_loop(0.3)
            self.log(f"[{name}] 已固定在寻限位起点 {restore_target:.4f} rad")
        err_deg = abs(rec.restored_q - restore_target) / DEG
        self.log(
            f"[{name}] 复位 → {rec.restored_q:.4f} rad "
            f"(目标 {restore_target:.4f}, 误差 {err_deg:.1f}°, status={rec.status})"
        )

    def _move_names(
        self,
        targets: Dict[str, float],
        duration: float,
        *,
        ignore_stop: bool = False,
        vel: Optional[float] = None,
        path_dir: Optional[Dict[str, float]] = None,
    ) -> None:
        if not targets:
            return
        cmd_vel = abs(vel if vel is not None else self.restore_vel)
        starts = {n: self.q.get(n, t) for n, t in targets.items()}
        aligned: Dict[str, float] = {}
        for name, target in targets.items():
            q0 = starts[name]
            if path_dir and name in path_dir:
                # 指定运动方向（如限位返回：-seek_dir），禁止圆上最短弧
                tgt = align_goal_along_dir(q0, target, path_dir[name])
                self.log(
                    f"[{name}] 原路返回: {q0:.4f} → {tgt:.4f} "
                    f"(dir={path_dir[name]:+.0f}, 原始目标 {target:.4f})"
                )
            else:
                # 默认也按连续支路靠近，避免 ±π 折叠
                tgt = unwrap_near(target, q0)
            aligned[name] = tgt
        targets = aligned
        max_dist = max(abs(targets[n] - starts[n]) for n in targets)
        duration = max(duration, max_dist / max(cmd_vel, 0.2) + 0.25)
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
            self.publish_targets(self.hold_q, moving=moving, move_vel=cmd_vel)
            self.spin_for(dt)
        for name, target in targets.items():
            self.hold_q[name] = target
        settle_t0 = time.monotonic()
        settle_budget = max(0.6, max_dist / max(cmd_vel, 0.2) + 0.3)
        while rclpy.ok():
            if self._stopping and not ignore_stop:
                break
            pending = []
            for n, tgt in targets.items():
                qn = unwrap_near(self.q.get(n, tgt), tgt)
                if abs(qn - tgt) >= 5.0 * DEG:
                    pending.append(n)
            if not pending:
                break
            if time.monotonic() - settle_t0 > settle_budget:
                for n in pending:
                    self.warn(
                        f"[{n}] 目标 {targets[n]:.4f} 实际 {self.q.get(n, targets[n]):.4f} rad，未到位"
                    )
                break
            self.publish_targets(self.hold_q, moving=moving, move_vel=cmd_vel)
            self.spin_for(dt)

    def _move_name(
        self,
        name: str,
        target: float,
        duration: float,
        *,
        ignore_stop: bool = False,
        vel: Optional[float] = None,
        path_dir: Optional[float] = None,
    ) -> None:
        pd = {name: path_dir} if path_dir is not None else None
        self._move_names(
            {name: target},
            duration,
            ignore_stop=ignore_stop,
            vel=vel,
            path_dir=pd,
        )

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
            rvel = max(self.restore_vel, 0.2)
            self._move_names(
                planned,
                duration=max(1.2, max_delta * DEG / rvel + 0.3),
                vel=rvel,
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
            rvel = max(self.restore_vel, 0.2)
            max_dist = max(
                abs(self.q.get(n, o) - o) for n, o in planned.items()
            )
            self._move_names(
                planned,
                duration=max(1.0, max_dist / rvel + 0.3),
                ignore_stop=True,
                vel=rvel,
            )
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
                    1.0,
                    abs(self.q.get(name, origin) - origin) / max(self.restore_vel, 0.2) + 0.3,
                ),
                ignore_stop=True,
                vel=self.restore_vel,
            )
            self.hold_q[name] = origin
            if still_unload or not hold_map.get(name, True):
                self.hold_names.discard(name)
                self.hold_loop(0.3)
                self.log(f"  {name} 复位后卸力")
            else:
                self.hold_loop(0.3)

    def _replace_limit_block(self, existing: str, cfg: Dict[str, Any], block: str) -> str:
        """替换 yaml 中限位段；兼容 (generated) / (FIXED REF) 等旧标题。"""
        begin = cfg["scan_begin"]
        end = cfg["scan_end"]
        yaml_key = cfg["yaml_key"]
        if begin in existing and end in existing:
            pre = existing[: existing.index(begin)]
            post = existing[existing.index(end) + len(end) :]
            return pre.rstrip() + "\n\n" + block + post.lstrip("\n")
        if end in existing:
            pre = existing[: existing.index(end)]
            lines = pre.split("\n")
            cut = len(lines)
            for i in range(len(lines) - 1, -1, -1):
                line = lines[i]
                if line.startswith("# ===") and "limit_scan" in line:
                    cut = i
                    break
                if line.strip() == f"{yaml_key}:":
                    # 往上找注释头
                    j = i
                    while j > 0 and (lines[j - 1].startswith("#") or not lines[j - 1].strip()):
                        j -= 1
                    cut = j if lines[j].startswith("# ===") else j
                    break
            pre = "\n".join(lines[:cut])
            post = existing[existing.index(end) + len(end) :]
            return pre.rstrip() + "\n\n" + block + post.lstrip("\n")
        return existing.rstrip() + "\n\n" + block

    def write_yaml(self) -> None:
        if not self.update_ref:
            self.log("参考限位固定，跳过写入（需要时加 --update-ref）")
            return
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lines: List[str] = [
            self.scan_begin,
            f"# 生成时间: {stamp}",
            "# 仅记录撞到硬件限位时的 URDF 位置 [rad]，保留两位小数（角度精度 0.5°）",
            "# seek_dir: 电机顺时针对应的 URDF 方向；"
            "+ 表示往 URDF 增大到限位，- 表示往减小到限位",
            f"{self.yaml_key}:",
        ]
        n_header = len(lines)
        for name in self.seek_names:
            rec = self.records.get(name)
            if rec is None or rec.q_enc_limit is None:
                continue
            d = rec.seek_dir_urdf
            side = "+" if d > 0 else "-"
            lines.append(
                f"  {name}: {fmt(rec.q_enc_limit)}  "
                f"# seek_dir {d:+.0f} → 往 {side} 到限位"
            )
        if len(lines) == n_header:
            lines.append("  {}")
        lines.append(self.scan_end)
        block = "\n".join(lines) + "\n"

        existing = ""
        if os.path.isfile(self.yaml_path):
            with open(self.yaml_path, "r", encoding="utf-8") as f:
                existing = f.read()
        text = self._replace_limit_block(
            existing,
            {
                "scan_begin": self.scan_begin,
                "scan_end": self.scan_end,
                "yaml_key": self.yaml_key,
            },
            block,
        )
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            f.write(text)
        self.log(f"已写入 {self.yaml_path}")

    def write_yaml_group(self, group: str, names: Sequence[str]) -> None:
        """默认不覆盖固定参考限位；仅 --update-ref 时写入。"""
        if not self.update_ref:
            self.log(f"[{group}] 参考限位固定，跳过写入（需要时加 --update-ref）")
            return
        cfg = GROUPS[group]
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lines: List[str] = [
            cfg["scan_begin"],
            f"# 生成时间: {stamp}",
            "# 仅记录撞到硬件限位时的 URDF 位置 [rad]，保留两位小数（角度精度 0.5°）",
            "# seek_dir: 电机顺时针对应的 URDF 方向；"
            "+ 表示往 URDF 增大到限位，- 表示往减小到限位",
            f"{cfg['yaml_key']}:",
        ]
        n_header = len(lines)
        src = self.all_records if self.all_records else self.records
        for name in names:
            rec = src.get(name)
            if rec is None or rec.q_enc_limit is None:
                continue
            d = rec.seek_dir_urdf
            side = "+" if d > 0 else "-"
            lines.append(
                f"  {name}: {fmt(rec.q_enc_limit)}  "
                f"# seek_dir {d:+.0f} → 往 {side} 到限位"
            )
        if len(lines) == n_header:
            lines.append("  {}")
        lines.append(cfg["scan_end"])
        block = "\n".join(lines) + "\n"
        existing = ""
        if os.path.isfile(self.yaml_path):
            with open(self.yaml_path, "r", encoding="utf-8") as f:
                existing = f.read()
        text = self._replace_limit_block(existing, cfg, block)
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            f.write(text)
        self.log(f"已写入 [{group}] → {self.yaml_path}")

    def _prep_steps_skip_waist(self) -> List[Any]:
        """右腿预备去掉腰（腰已在并行流水线第一步单独转）。"""
        out: List[Any] = []
        for item in GROUPS["right_leg"].get("prep", ()):
            if isinstance(item, dict):
                if item.get("name") == PARALLEL_WAIST:
                    continue
                out.append(item)
            else:
                batch = [s for s in item if s.get("name") != PARALLEL_WAIST]
                if batch:
                    out.append(tuple(batch) if len(batch) > 1 else batch[0])
        return out

    def run_prep_steps(self, prep_items: Sequence[Any]) -> None:
        """执行任意预备步骤列表（与 run_prep 相同逻辑）。"""
        old = self.group_cfg
        # 临时挂上 prep
        tmp = dict(old)
        tmp["prep"] = tuple(prep_items)
        self.group_cfg = tmp
        try:
            self.run_prep()
        finally:
            self.group_cfg = old

    def _detect_limit_tick(
        self,
        rec: JointRecord,
        *,
        t0: float,
        cmd_q: float,
        stall_t0: Optional[float],
        freeze_t0: Optional[float],
        freeze_q: Optional[float],
        window: deque,
        force_tau_sec: float,
        tau_full_t0: Optional[float],
        tau_below_t0: Optional[float],
        force_armed_logged: bool,
        warned_soft: bool,
        seek_vel: float,
    ) -> Tuple[bool, Dict[str, Any]]:
        """单轴一拍寻限位检测。返回 (hit, state_updates)。"""
        name = rec.name
        d = rec.seek_dir_urdf
        now = time.monotonic()
        elapsed = now - t0
        st: Dict[str, Any] = {
            "cmd_q": cmd_q,
            "stall_t0": stall_t0,
            "freeze_t0": freeze_t0,
            "freeze_q": freeze_q,
            "tau_full_t0": tau_full_t0,
            "tau_below_t0": tau_below_t0,
            "force_armed_logged": force_armed_logged,
            "warned_soft": warned_soft,
            "log": None,
        }
        if elapsed > self.seek_timeout:
            rec.status = "fail"
            rec.note = f"超时 {self.seek_timeout:.0f}s 未顶到硬件限位"
            return True, st

        q = self.q.get(name, cmd_q)
        tau = self.tau.get(name, 0.0)
        travel = (q - rec.start_q) * d
        tau_rel = tau - rec.tau_bias
        cmd_q = rec.start_q + d * min(seek_vel * elapsed, self.max_travel + 0.05)
        st["cmd_q"] = cmd_q
        self.hold_q[name] = cmd_q

        lead = (cmd_q - q) * d
        tau_abs = abs(tau)
        tau_rel_abs = abs(tau_rel)
        # 并行多轴时 τ_rel 易受耦合干扰；强制返回必须以 |τ| 为准
        at_protect = tau_abs >= rec.tau_protect * 0.85
        at_protect_rel = (
            tau_rel_abs >= rec.tau_protect * 0.92
            and tau_abs >= rec.tau_protect * 0.45
        )
        moved = abs(q - rec.start_q)
        start_at_limit = (
            elapsed >= 0.45
            and moved < 1.0 * DEG
            and lead >= 0.06
            and (at_protect or at_protect_rel)
        )
        travel_ok = travel >= self.min_travel or start_at_limit

        freeze_eps = 0.5 * DEG
        if freeze_t0 is None or freeze_q is None or abs(q - freeze_q) > freeze_eps:
            st["freeze_t0"] = now
            st["freeze_q"] = q
            st["stall_t0"] = None
            window.clear()
            frozen = False
        else:
            window.append(q)
            frozen = travel_ok and (now - float(freeze_t0)) >= self.stall_hold

        if force_tau_sec > 0.0 and travel_ok:
            if at_protect and frozen:
                st["tau_below_t0"] = None
                if tau_full_t0 is None:
                    st["tau_full_t0"] = now
                    if not force_armed_logged:
                        st["force_armed_logged"] = True
                        st["log"] = (
                            f"[{name}] 已达保护力矩 |τ|={tau_abs:.2f} "
                            f"(rel {tau_rel:+.2f}, bias {rec.tau_bias:+.2f})，"
                            f"位置停滞，{force_tau_sec:.1f}s 内强制记限位并返回"
                        )
                elif now - tau_full_t0 >= force_tau_sec:
                    rec.q_enc_limit = median(list(window)) if window else q
                    rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                    rec.stall_tau = tau
                    rec.status = "limit"
                    rec.note = (
                        f"满力矩强制限位 {force_tau_sec:.1f}s |τ|={tau_abs:.2f} "
                        f"(保护 {rec.tau_protect:.2f}), 行程 {rec.q_travel_rad / DEG:.2f} deg"
                    )
                    st["log"] = f"[{name}] 强制超时恢复: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
                    return True, st
            elif tau_full_t0 is not None:
                if tau_below_t0 is None:
                    st["tau_below_t0"] = now
                elif now - tau_below_t0 >= 0.25:
                    st["tau_full_t0"] = None
                    st["tau_below_t0"] = None
                    st["force_armed_logged"] = False

        if start_at_limit and elapsed >= 0.6:
            rec.q_enc_limit = median(list(window)) if window else q
            rec.q_travel_rad = rec.q_enc_limit - rec.start_q
            rec.stall_tau = tau
            rec.status = "limit"
            rec.note = (
                f"起点已在硬件限位 |τ|={tau_abs:.2f} lead={lead:.3f} "
                f"(保护 {rec.tau_protect:.2f}), 移动 {moved / DEG:.2f} deg"
            )
            st["log"] = f"[{name}] 起点限位，开始返回: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
            return True, st

        if frozen:
            cmd_ahead = lead >= max(self.max_lead(name), 0.12)
            torque_hit = travel_ok and (
                tau_abs >= rec.tau_protect * 0.92
                or at_protect_rel
                or (tau_abs >= rec.tau_protect * 0.85 and cmd_ahead)
                or (tau_abs >= rec.tau_protect * 0.75 and lead >= 0.08)
            )
            if stall_t0 is None and torque_hit:
                st["stall_t0"] = now
                stall_t0 = now
            hardware_stop = (
                torque_hit
                and stall_t0 is not None
                and now - stall_t0 >= self.stall_hold
            )
            if not torque_hit and not warned_soft:
                st["warned_soft"] = True
                st["log"] = (
                    f"[{name}] 位置停滞但 |τ|={tau_abs:.2f} rel={tau_rel:+.2f} "
                    f"lead={lead:.3f} < 保护 {rec.tau_protect:.2f}，继续推"
                )
            if hardware_stop:
                rec.q_enc_limit = median(list(window)) if window else q
                rec.q_travel_rad = rec.q_enc_limit - rec.start_q
                rec.stall_tau = tau
                rec.status = "limit"
                rec.note = (
                    f"硬件限位 |τ|={tau_abs:.2f} |τ-τb|={tau_rel_abs:.2f} "
                    f"lead={lead:.3f} (保护 {rec.tau_protect:.2f}), "
                    f"行程 {rec.q_travel_rad / DEG:.2f} deg"
                )
                st["log"] = f"[{name}] 撞限位，开始返回: {rec.note}, q_limit={rec.q_enc_limit:.4f}"
                return True, st

        if abs(tau) >= rec.tau_abort or abs(tau_rel) >= rec.tau_abort:
            rec.status = "fail"
            rec.note = f"力矩过大 τ={tau:.2f} (rel {tau_rel:.2f}) ≥ {rec.tau_abort}"
            return True, st
        if travel > self.max_travel:
            rec.status = "fail"
            rec.note = f"超过行程帽 {self.max_travel / DEG:.1f} deg"
            return True, st
        return False, st

    def seek_lanes_parallel(self, lanes: Dict[str, Sequence[str]]) -> None:
        """多车道并行寻限位：车道间同时，车道内按队列串行；撞限位后该轴立即复位。"""
        dt = 1.0 / self.rate_hz
        rvel = max(self.restore_vel, 0.2)

        @dataclass
        class Lane:
            name: str
            queue: List[str]
            idx: int = 0
            phase: str = "idle"  # idle|seek|restore|done
            rec: Optional[JointRecord] = None
            t0: float = 0.0
            cmd_q: float = 0.0
            seek_vel: float = 1.5
            stall_t0: Optional[float] = None
            freeze_t0: Optional[float] = None
            freeze_q: Optional[float] = None
            window: deque = field(default_factory=lambda: deque(maxlen=40))
            force_tau_sec: float = 1.0
            tau_full_t0: Optional[float] = None
            tau_below_t0: Optional[float] = None
            force_armed_logged: bool = False
            warned_soft: bool = False
            restore_goal: float = 0.0
            restore_back: float = 0.0
            restore_t0: float = 0.0
            restore_dur: float = 1.0
            restore_start: float = 0.0
            start_was_limit: bool = False
            path_dir: float = 0.0
            restore_q0: float = 0.0  # 进入复位时的连续起点

        workers = [Lane(name=k, queue=list(v)) for k, v in lanes.items() if v]

        def start_next(w: Lane) -> None:
            while w.idx < len(w.queue):
                name = w.queue[w.idx]
                w.idx += 1
                if name not in self.ctrl_names:
                    self.warn(f"[{w.name}] 跳过未接管轴 {name}")
                    continue
                if name in self.unload_until_sought:
                    self.unload_until_sought.discard(name)
                    self.hold_names.add(name)
                rec = JointRecord(
                    name=name,
                    motor_id=self.motor_id(name),
                    seek_dir_urdf=self.seek_dir(name),
                    tau_protect=self.tau_protect_of(name),
                    tau_abort=self.tau_abort_of(name),
                )
                rec.start_q = float(self.q.get(name, 0.0))
                self.hold_names.add(name)
                rec.tau_bias = self.sample_bias(name)
                self.log(
                    f"[{name}] 起点 {rec.start_q:.4f} rad, "
                    f"τ_bias={rec.tau_bias:.3f} N·m"
                )
                self.records[name] = rec
                self.all_records[name] = rec
                w.rec = rec
                w.phase = "seek"
                w.t0 = time.monotonic()
                w.cmd_q = rec.start_q
                w.seek_vel = self.seek_vel_of(name)
                w.stall_t0 = None
                w.freeze_t0 = None
                w.freeze_q = None
                w.window = deque(maxlen=40)
                w.force_tau_sec = self.force_tau_sec_of(name)
                w.tau_full_t0 = None
                w.tau_below_t0 = None
                w.force_armed_logged = False
                w.warned_soft = False
                self.log(
                    f"==== [{w.name}] {name} 并行寻限位 "
                    f"vel={w.seek_vel:.1f} τ={rec.tau_protect:.1f} ===="
                )
                return
            w.phase = "done"
            w.rec = None

        for w in workers:
            start_next(w)

        while rclpy.ok() and not self._stopping:
            if all(w.phase == "done" for w in workers):
                break
            seeking: List[str] = []
            moving: List[str] = []
            seek_vels: Dict[str, float] = {}

            for w in workers:
                if w.phase == "seek" and w.rec is not None:
                    hit, st = self._detect_limit_tick(
                        w.rec,
                        t0=w.t0,
                        cmd_q=w.cmd_q,
                        stall_t0=w.stall_t0,
                        freeze_t0=w.freeze_t0,
                        freeze_q=w.freeze_q,
                        window=w.window,
                        force_tau_sec=w.force_tau_sec,
                        tau_full_t0=w.tau_full_t0,
                        tau_below_t0=w.tau_below_t0,
                        force_armed_logged=w.force_armed_logged,
                        warned_soft=w.warned_soft,
                        seek_vel=w.seek_vel,
                    )
                    w.cmd_q = st["cmd_q"]
                    w.stall_t0 = st["stall_t0"]
                    w.freeze_t0 = st["freeze_t0"]
                    w.freeze_q = st["freeze_q"]
                    w.tau_full_t0 = st["tau_full_t0"]
                    w.tau_below_t0 = st["tau_below_t0"]
                    w.force_armed_logged = st["force_armed_logged"]
                    w.warned_soft = st["warned_soft"]
                    if st["log"]:
                        (self.warn if "继续推" in st["log"] else self.log)(st["log"])
                    if hit:
                        if w.rec.q_enc_limit is None:
                            w.rec.q_enc_limit = self.q.get(w.rec.name, w.rec.start_q)
                            w.rec.q_travel_rad = w.rec.q_enc_limit - w.rec.start_q
                        q_lim = float(w.rec.q_enc_limit)
                        d = w.rec.seek_dir_urdf
                        q_now_fb = float(self.q.get(w.rec.name, q_lim))
                        # 限位拨到与当前反馈同连续支路，避免 ±π 折叠后走最短弧
                        q_lim_c = unwrap_near(q_lim, q_now_fb)
                        w.path_dir = -float(d)
                        w.restore_q0 = q_now_fb
                        w.restore_back = align_goal_along_dir(
                            q_now_fb, q_lim_c - d * self.backoff, w.path_dir
                        )
                        # 用与反馈同连续支路的限位算零位；本轴先回寻限位起点
                        q_home = self.home_from_limit(w.rec.name, q_lim_c)
                        if q_home is not None and not getattr(
                            self.args, "no_write_zero", False
                        ):
                            q_home_aligned = align_goal_along_dir(
                                q_lim_c, q_home, w.path_dir
                            )
                            self.remember_zero_joint(w.rec.name, q_home_aligned)
                            self.log(
                                f"[{w.rec.name}] 已记录计算零位 {q_home_aligned:.4f} "
                                f"(参考限位 {self.limit_ref[w.rec.name]:.2f}, "
                                f"连续限位 {q_lim_c:.4f})；本轴先回寻限位起点"
                            )
                        else:
                            self.warn(f"[{w.rec.name}] 无参考限位，无法计算零位")
                        # 撞限位后先沿原路回到本轴寻限位起点
                        w.start_was_limit = abs(q_lim_c - w.rec.start_q) < 1.5 * DEG
                        raw_goal = (
                            w.restore_back if w.start_was_limit else w.rec.start_q
                        )
                        w.restore_goal = align_goal_along_dir(
                            w.restore_back, raw_goal, w.path_dir
                        )
                        w.restore_start = q_now_fb
                        dist = abs(w.restore_goal - w.restore_start) + abs(self.backoff)
                        w.restore_dur = max(self.restore_sec, dist / rvel + 0.4)
                        w.restore_t0 = time.monotonic()
                        w.phase = "restore"
                        self.log(
                            f"[{w.rec.name}] 离开限位 → 目标 {w.restore_goal:.4f} "
                            f"(先退 {w.restore_back:.4f}, 原路)"
                        )
                    else:
                        seeking.append(w.rec.name)
                        seek_vels[w.rec.name] = w.seek_vel

                elif w.phase == "restore" and w.rec is not None:
                    name = w.rec.name
                    elapsed = time.monotonic() - w.restore_t0
                    # 先沿原路退到 back，再沿原路到 goal（连续插值，禁止最短弧）
                    split = max(0.25, abs(self.backoff) / rvel + 0.1)
                    if elapsed < split:
                        t1 = min(1.0, elapsed / max(split, 1e-3))
                        s = 3 * t1 * t1 - 2 * t1 * t1 * t1
                        tgt = w.restore_q0 + (w.restore_back - w.restore_q0) * s
                    else:
                        t2 = (elapsed - split) / max(w.restore_dur - split, 0.2)
                        t2 = min(1.0, max(0.0, t2))
                        s = 3 * t2 * t2 - 2 * t2 * t2 * t2
                        tgt = w.restore_back + (w.restore_goal - w.restore_back) * s
                    self.hold_q[name] = tgt
                    moving.append(name)
                    if elapsed >= w.restore_dur:
                        self.hold_q[name] = w.restore_goal
                        w.rec.restored_q = self.q.get(name, w.restore_goal)
                        if w.rec.status == "limit":
                            w.rec.status = "ok"
                        self.hold_names.add(name)
                        self.log(
                            f"[{name}] 复位完成 → {w.rec.restored_q:.4f} "
                            f"status={w.rec.status}"
                        )
                        start_next(w)

            self.publish_targets(
                self.hold_q,
                seeking=seeking,
                moving=moving,
                move_vel=rvel,
                seek_vels=seek_vels,
            )
            self.spin_for(dt)

        for w in workers:
            if w.phase == "restore" and w.rec is not None:
                # 中断时尽量退开
                try:
                    self._move_name(
                        w.rec.name,
                        w.restore_goal,
                        1.0,
                        ignore_stop=True,
                        vel=rvel,
                    )
                except Exception:
                    pass

    def run_parallel(self) -> int:
        """
        并行流水线:
          0) 左半身卸力并保持
          1) 腰转 90° 并保持
          2) 腿预备（踝/膝/髋）
          3) 手 + 头 + 腿 三车道同时寻限位并各自恢复
          4) 标定腰（从当前位置继续转到限位）
          5) 回初始姿态
        """
        need: List[str] = [PARALLEL_WAIST]
        for js in PARALLEL_LANES.values():
            need.extend(js)
        need.extend(LEFT_BODY_JOINTS)
        need = list(dict.fromkeys(need))
        missing = [n for n in need if n not in self.js_names]
        if missing:
            raise RuntimeError("关节不存在: " + ", ".join(missing))

        if self.args.dry_run:
            self.log("dry-run 并行流水线:")
            self.log(f"  参考限位 {len(self.limit_ref)} 轴: " + ", ".join(sorted(self.limit_ref)[:6]) + "...")
            self.log("  0) 左半身卸力")
            self.log("  1) 腰 -90°")
            self.log("  2) 腿预备(踝/膝/髋)")
            for k, js in PARALLEL_LANES.items():
                self.log(f"  3) 车道 [{k}] 同时: " + ", ".join(js))
            self.log(f"  4) 腰继续右转撞限位")
            self.log("  5) 回初始姿态（--no-write-zero 时不走计算零位）")
            return 0

        if not self.limit_ref:
            raise RuntimeError(
                f"未读到参考限位，请检查 {self.yaml_path} 中 "
                "right_arm_limit_q / waist_neck_limit_q / right_leg_limit_q"
            )
        if self.update_ref:
            self.log("限位扫描结果将覆写 standing_pose.yaml（--update-ref）")
        else:
            self.log(f"已加载参考限位 {len(self.limit_ref)} 轴（不加 --update-ref 则不覆写 yaml）")
        if getattr(self.args, "no_write_zero", False):
            self.log("不写零：完成后仅恢复初始姿态")
        else:
            self.log("写零: 限位完成后腰+上半身移到计算零位，再确认写零")
        self.log("左半身: 全程卸力（不位控）")

        self.group_cfg = GROUPS["right_leg"]  # 力矩/idle 以腿组为准
        self.group_joints = GROUPS["right_leg"]["joints"]
        self.group_name = "parallel"
        self.request_group(need_names=need)
        try:
            for name in self.ctrl_names:
                self.hold_q[name] = self.q.get(name, 0.0)
                self.hold_names.add(name)
            self.hold_loop(0.3)

            # ---- 0) 左半身卸力 ----
            self.log(">>>>>>>> [parallel] 0/5 左半身卸力 <<<<<<<<")
            self.setup_left_body_unload()

            self.run_origin = {
                n: float(self.q.get(n, 0.0)) for n in self.ctrl_names
            }
            self.log("已记录启动姿态（最终复位基准）")

            # ---- 1) 腰转 90° ----
            self.log(">>>>>>>> [parallel] 1/5 腰转 90° <<<<<<<<")
            self.refresh_left_body_unload()
            waist_delta = self._prep_delta(PARALLEL_WAIST, -90.0)
            w0 = self.q.get(PARALLEL_WAIST, 0.0)
            w1 = w0 + waist_delta * DEG
            self.prep_origin[PARALLEL_WAIST] = self.run_origin.get(PARALLEL_WAIST, w0)
            self.log(f"预备腰: {w0:.4f} → {w1:.4f} ({waist_delta:+.0f}°)")
            self.hold_names.add(PARALLEL_WAIST)
            self._move_names(
                {PARALLEL_WAIST: w1},
                duration=max(1.5, abs(waist_delta) * DEG / max(self.restore_vel, 0.2) + 0.3),
                vel=self.restore_vel,
            )
            self.hold_q[PARALLEL_WAIST] = w1
            self.refresh_left_body_unload()
            self.hold_loop(0.3)

            # ---- 2) 腿预备 ----
            self.log(">>>>>>>> [parallel] 2/5 腿预备 <<<<<<<<")
            self.refresh_left_body_unload()
            self.unload_until_sought = set(
                GROUPS["right_leg"].get("unload_until_sought", ())
            )
            for n in self.unload_until_sought:
                self.hold_names.discard(n)
            self.run_prep_steps(self._prep_steps_skip_waist())
            self.refresh_left_body_unload()

            # ---- 3) 手/头/腿并行 ----
            self.log(">>>>>>>> [parallel] 3/5 手+头+腿并行寻限位 <<<<<<<<")
            self.refresh_left_body_unload()
            self.seek_lanes_parallel(PARALLEL_LANES)
            self.refresh_left_body_unload()
            # 写臂/头/腿（头写入 waist_neck，腰稍后再写）
            self.write_yaml_group("right_arm", PARALLEL_LANES["arm"])
            head_names = list(PARALLEL_LANES["head"])
            self.write_yaml_group("waist_neck", head_names)  # 暂无腰，后面补
            self.write_yaml_group("right_leg", PARALLEL_LANES["leg"])

            # ---- 4) 腰继续向右撞限位，再恢复 ----
            if not self._stopping:
                self.log(
                    ">>>>>>>> [parallel] 4/5 腰继续向右撞限位后恢复 "
                    f"(seek_dir={PARALLEL_WAIST_SEEK_DIR:+.0f}) <<<<<<<<"
                )
                self.refresh_left_body_unload()
                self.hold_names.add(PARALLEL_WAIST)
                # 强制与预备右转同向，避免被其它组 dir 覆盖
                JOINT_META[PARALLEL_WAIST]["dir"] = float(PARALLEL_WAIST_SEEK_DIR)
                self.group_cfg = dict(self.group_cfg)
                dirs = dict(self.group_cfg.get("dir", {}))
                dirs[PARALLEL_WAIST] = float(PARALLEL_WAIST_SEEK_DIR)
                self.group_cfg["dir"] = dirs
                q_before = self.q.get(PARALLEL_WAIST, 0.0)
                self.log(
                    f"腰从当前 {q_before:.4f} rad 继续右转寻硬限位，"
                    "到位后先退开再回到本段起点，最后再全身恢复"
                )
                self.seek_lanes_parallel({"waist": (PARALLEL_WAIST,)})
                # 合并头+腰写 waist_neck
                self.write_yaml_group(
                    "waist_neck",
                    list(PARALLEL_LANES["head"]) + [PARALLEL_WAIST],
                )

            # ---- 5) 回初始姿态（不写零、不移动到计算零位）----
            self.log(">>>>>>>> [parallel] 5/5 恢复到标定初始姿态 <<<<<<<<")
            self.refresh_left_body_unload()

            origin_pose = {
                n: o
                for n, o in self.run_origin.items()
                if n in self.ctrl_names and n not in self.left_body_unload
            }
            self.move_hold_pose(
                origin_pose,
                note="限位标定完成：恢复到【开始标定】时的初始姿态",
            )
            self.refresh_left_body_unload()

            if getattr(self.args, "no_write_zero", False):
                self.log("已指定 --no-write-zero：跳过计算零位移动与写零")
                self.log("并行流水线完成")
                return 0

            homes_ub = self.upper_body_homes()
            if not homes_ub:
                self.warn("腰/上半身没有计算出任何零位，结束（检查 yaml 参考限位）")
                self.log("并行流水线完成")
                return 0

            self.move_joints_to_homes(
                homes_ub,
                note=(
                    "仅移动【腰+上半身】到计算零位"
                    "（q_home = 实测限位 - 参考限位；右腿不动）"
                ),
            )
            self.refresh_left_body_unload()
            self.hold_loop(0.3)

            do_zero = self.prompt_write_zero(homes_ub)
            if do_zero:
                ids = self.upper_body_zero_ids()
                self.log("用户确认写零，释放控制权后调用 /reset_zero …")
                self.release_group()
                self.call_reset_zero(ids)
            else:
                self.log("已跳过写零。腰+上半身仍保持在计算零位附近；腿未改。")
            self.log("并行流水线完成")
            return 0
        finally:
            if self.uuid:
                try:
                    self.release_group()
                except Exception:
                    pass

    def run_one_group(self) -> None:
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
                    self._move_name(
                        name, target, 1.0, ignore_stop=True, vel=self.restore_vel
                    )
                    if self.group_cfg.get("hold_after_restore"):
                        self.unload_until_sought.discard(name)
                        self.hold_q[name] = target
                        self.hold_names.add(name)
                except Exception:
                    pass
            if self._stopping:
                self.warn("已中断：完成本轴返回后停止后续关节")
                break
            self.hold_loop(0.2)
        self.restore_prep()
        # 本组结束后全身保持当前角，衔接下组
        for name in self.ctrl_names:
            self.hold_q[name] = float(self.q.get(name, self.hold_q.get(name, 0.0)))
            self.hold_names.add(name)

    def run(self) -> int:
        if getattr(self.args, "parallel", False):
            self.parallel_mode = True
            self.chain_hold = True
            if not self.wait_js():
                self.warn("未收到 /joint_states，请先启动 pi_plus_orin / midware")
                return 1
            return self.run_parallel()

        groups = list(getattr(self.args, "groups", None) or [self.args.group])
        self.chain_hold = len(groups) > 1
        if not self.wait_js():
            self.warn("未收到 /joint_states，请先启动 pi_plus_orin / midware")
            return 1

        all_need: List[str] = []
        for g in groups:
            all_need.extend(GROUPS[g]["joints"])
        missing = [n for n in all_need if n not in self.js_names]
        if missing:
            raise RuntimeError("关节不存在: " + ", ".join(sorted(set(missing))))

        if self.args.dry_run:
            self.log("dry-run: 不出力、不申请控制权")
            for g in groups:
                self.apply_group(g)
                self.log(f"---- {g}: {self.group_cfg['title']} ----")
                for step in self._flatten_prep_steps():
                    self.log(f"  预备 {step.get('note', step['name'])}  Δ={step['delta_deg']:+.0f}°")
                for name in self.seek_names:
                    mid = self.motor_id(name)
                    d = self.seek_dir(name)
                    q0 = self.q.get(name, float("nan"))
                    self.log(
                        f"  {name} id={mid} q={q0:.4f} URDF_dir={d:+.0f} "
                        f"τ_protect={self.tau_protect_of(name):.2f}"
                    )
            return 0

        self.apply_group(groups[0])
        self.request_group(need_names=all_need)
        try:
            for name in self.ctrl_names:
                self.hold_q[name] = self.q.get(name, 0.0)
                if self.chain_hold:
                    self.hold_names.add(name)
            self.hold_loop(0.3)
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

            for gi, g in enumerate(groups):
                if self._stopping:
                    break
                self.apply_group(g)
                self.log(
                    f">>>>>>>> [{g}] {self.group_cfg['title']} "
                    f"({gi + 1}/{len(groups)}) <<<<<<<<"
                )
                self.hold_loop(0.15)
                self.run_one_group()
                try:
                    self.write_yaml()
                except Exception as exc:
                    self.warn(f"写 yaml 失败: {exc}")
                if gi < len(groups) - 1 and not self._stopping:
                    self.log(f"组 [{g}] 完成，保持姿态切换下一组…")
                    self.hold_loop(0.2)
            return 0
        finally:
            try:
                if self.records:
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


def unwrap_near(q: float, ref: float) -> float:
    """把 q 拨到与 ref 同连续支路（±2π），避免跨 ±π 跳变。"""
    two_pi = 2.0 * math.pi
    while q - ref > math.pi:
        q -= two_pi
    while q - ref < -math.pi:
        q += two_pi
    return q


def align_goal_along_dir(q_now: float, q_goal: float, move_dir: float) -> float:
    """选择 q_goal+2πk，使 (goal-now) 与 move_dir 同向（原路/指定方向），而非圆上最短弧。"""
    if abs(move_dir) < 1e-9:
        return unwrap_near(q_goal, q_now)
    two_pi = 2.0 * math.pi
    sense = 1.0 if move_dir > 0 else -1.0
    best: Optional[float] = None
    best_abs = float("inf")
    for k in range(-4, 5):
        cand = q_goal + k * two_pi
        delta = cand - q_now
        if delta * sense < -1e-9:
            continue
        ad = abs(delta)
        if ad < best_abs:
            best_abs = ad
            best = cand
    if best is not None:
        return best
    # 极端情况：仍强制沿 sense 走一整圈以上
    return q_now + sense * abs(unwrap_near(q_goal, q_now) - q_now)


def quantize_limit_rad(v: float, step_deg: float = 0.5) -> float:
    """限位角按 step_deg（默认 0.5°）取整，再保留 rad 两位小数。"""
    step = step_deg * DEG
    q = round(v / step) * step
    return round(q, 2)


def fmt(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "null"
    return f"{quantize_limit_rad(float(v)):.2f}"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="限力矩寻硬限位后复位")
    p.add_argument("--group", choices=sorted(GROUPS), default="right_arm")
    p.add_argument(
        "--groups",
        nargs="+",
        choices=sorted(GROUPS),
        default=None,
        help="多组一次串行（同进程保持控制权，避免组间卸力卡顿）",
    )
    p.add_argument(
        "--parallel",
        action="store_true",
        help="并行流水线：腰先转90° → 手/头/腿同时寻限位 → 再标腰 → 标定0位写零",
    )
    p.add_argument(
        "--write-zero",
        dest="write_zero",
        action="store_true",
        default=True,
        help="（保留）允许在确认后写零；默认开，最终仍需命令行确认",
    )
    p.add_argument(
        "--no-write-zero",
        action="store_true",
        help="走到计算零位后不询问、不写零",
    )
    p.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="在计算零位处自动确认写零（不交互）",
    )
    p.add_argument(
        "--update-ref",
        action="store_true",
        help="允许覆盖 standing_pose.yaml 中的固定参考限位（默认不写）",
    )
    args_pre, _ = p.parse_known_args(argv)
    first_group = (args_pre.groups[0] if args_pre.groups else args_pre.group)
    group_joints = list(GROUPS[first_group]["joints"])

    p.add_argument("--joints", nargs="+", default=group_joints, help="要测的关节，默认按组远端先")
    p.add_argument("--tau-protect", type=float, default=None, help="覆盖组内所有轴保护力矩 N·m")
    p.add_argument("--tau-abort", type=float, default=8.0, help="中止力矩下限 N·m；各轴会自动抬到 protect+2")
    p.add_argument("--kp", type=float, default=8.0)
    p.add_argument("--kd", type=float, default=0.6)
    p.add_argument("--seek-vel", type=float, default=None, help="寻限位速度 rad/s；默认手部 2 / 腰头腿 1.5")
    p.add_argument("--restore-vel", type=float, default=None, help="撞限位后/预备复位速度 rad/s，默认 2.5")
    p.add_argument("--force-tau-sec", type=float, default=1.0, help="顶满保护力矩后强制返回秒数，默认 1.0")
    p.add_argument("--min-travel-deg", type=float, default=5.0)
    p.add_argument("--max-travel-deg", type=float, default=350.0)
    p.add_argument("--stall-vel", type=float, default=0.02)
    p.add_argument("--stall-hold", type=float, default=0.2)
    p.add_argument("--bias-sec", type=float, default=0.3)
    p.add_argument("--backoff-deg", type=float, default=4.0)
    p.add_argument("--restore-sec", type=float, default=1.0, help="复位最短时间 s")
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
    if getattr(args, "parallel", False):
        args.group = "right_leg"
    else:
        groups = list(args.groups) if args.groups else [args.group]
        if not args.groups:
            allowed = GROUPS[args.group]["dir"]
            unknown = [n for n in args.joints if n not in allowed]
            if unknown:
                print(f"不是组 {args.group} 的关节: " + ", ".join(unknown), file=sys.stderr)
                return 2
        args.group = groups[0]

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
            node.release_group()
        except Exception:
            pass
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
