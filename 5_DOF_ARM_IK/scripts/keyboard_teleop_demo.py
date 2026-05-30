#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制右臂末端位姿 → IK → 关节角；可选下发实机。

坐标系: base_link（整机器人，右手定则）
  X 前, Y 左, Z 上

平移:
  W/S 前/后 (X±)    A/D 左/右 (Y±)    R 上 (Z+)
  方向键: ↑↓ 前/后  ←→ 左/右  PgUp/PgDn 上/下

旋转 (RPY 增量, 绕 base_link 固定轴):
  7/8  Roll±(X)  4/5  Pitch±(Y)  1/2  Yaw±(Z)
  I/K  Roll±     J/L  Pitch±     U/O  Yaw±

夹爪（实机 r_claw_joint）:
  F     开合切换

其它:
  空格  以当前关节角重置 IK 目标（不位移）
  H     帮助   Q/ESC  退出
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import time
import termios
import tty
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arm_ik import IKTaskMode, RightArmIKSolver  # noqa: E402
from arm_ik.robot_bridge import RightArmRobotBridge, load_robot_config  # noqa: E402
from arm_ik.transforms import rpy_matrix  # noqa: E402

FRAME_NAME = "base_link"


def matrix_to_rpy_deg(r: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(float(r[0, 0] ** 2 + r[1, 0] ** 2))
    if sy > 1e-6:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = math.atan2(float(-r[1, 2]), float(r[1, 1]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def apply_delta_pose(
    t: np.ndarray,
    dpos: np.ndarray,
    drot_rpy: np.ndarray,
) -> np.ndarray:
    out = t.copy()
    out[:3, 3] += dpos
    r_delta = rpy_matrix(drot_rpy[0], drot_rpy[1], drot_rpy[2])
    out[:3, :3] = r_delta @ out[:3, :3]
    return out


def fk_in_base(solver: RightArmIKSolver, q: np.ndarray, q_waist: float) -> np.ndarray:
    t_torso = solver.fk(q)
    if solver.torso_mount is None:
        return t_torso
    return solver.torso_mount.compute(q_waist) @ t_torso


def ik_from_base_target(
    solver: RightArmIKSolver,
    target_base: np.ndarray,
    q_seed: np.ndarray,
    q_waist: float,
    mode: IKTaskMode,
):
    if solver.torso_mount is None:
        return solver.ik(target_base, q_seed=q_seed, mode=mode)
    target_torso = solver.torso_mount.target_in_torso_frame(target_base, q_waist)
    return solver.ik(target_torso, q_seed=q_seed, mode=mode)


class KeyReader:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *args):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self) -> str:
        if not select.select([sys.stdin], [], [], 0.05)[0]:
            return ""
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        if not select.select([sys.stdin], [], [], 0.02)[0]:
            return ch
        seq = sys.stdin.read(2)
        return ch + seq


ARROW_MAP = {
    "[A": "left",
    "[B": "down",
    "[C": "right",
    "[D": "up",
    "[5": "pgup",
    "[6": "pgdown",
}

HELP = __doc__


def run_teleop(
    solver: RightArmIKSolver,
    pos_step: float,
    rot_step_deg: float,
    ik_mode: IKTaskMode,
    q_waist: float = 0.0,
    robot: Optional[RightArmRobotBridge] = None,
    default_q: Optional[np.ndarray] = None,
) -> None:
    if robot is not None:
        q = robot.get_arm_q().copy()
    elif default_q is not None:
        q = np.asarray(default_q, dtype=float).copy()
    else:
        q = np.zeros(solver.dof)

    target_base = fk_in_base(solver, q, q_waist)
    target_cmd = target_base.copy()
    rot_step = math.radians(rot_step_deg)
    last_key_t = 0.0
    key_debounce_s = 0.12

    def sync_target_from_q() -> None:
        nonlocal target_base, target_cmd
        target_base = fk_in_base(solver, q, q_waist)
        target_cmd = target_base.copy()

    def push_robot(smooth: bool = True) -> None:
        if robot is not None:
            robot.set_arm_goal(q, smooth=smooth)

    def do_ik() -> bool:
        nonlocal q
        res = ik_from_base_target(solver, target_cmd, q, q_waist, ik_mode)
        if not res.success:
            print(
                f"\rIK FAIL err={res.position_error_m*1000:.1f}mm "
                f"({res.message})",
                end="",
                flush=True,
            )
            return False
        q = res.q.copy()
        push_robot()
        rpy_cmd = matrix_to_rpy_deg(target_cmd[:3, :3])
        claw_s = ""
        if robot is not None:
            claw_s = f" claw={'开' if robot.claw_is_open else '合'}"
        print(
            f"\rIK OK   "
            f"cmd=[{target_cmd[0,3]:+.3f},{target_cmd[1,3]:+.3f},{target_cmd[2,3]:+.3f}] "
            f"rpy=[{rpy_cmd[0]:+.0f},{rpy_cmd[1]:+.0f},{rpy_cmd[2]:+.0f}]° "
            f"err={res.position_error_m*1000:.1f}mm "
            f"q=[{', '.join(f'{v:+.2f}' for v in q)}]{claw_s}",
            end="",
            flush=True,
        )
        return True

    def move(dpos=None, drot=None) -> None:
        nonlocal target_cmd
        dpos = np.zeros(3) if dpos is None else np.asarray(dpos, float)
        drot = np.zeros(3) if drot is None else np.asarray(drot, float)
        target_cmd = apply_delta_pose(target_cmd, dpos, drot)
        do_ik()

    def accept_key() -> bool:
        nonlocal last_key_t
        now = time.time()
        if now - last_key_t < key_debounce_s:
            return False
        last_key_t = now
        return True

    print(HELP)
    if solver.torso_mount is None:
        print("警告: URDF 无腰-躯干链，base 与 torso 视为同一系")
    if robot is None:
        mode_note = "仅 IK（加 --robot 或去掉 --sim-only 才发实机）"
    elif getattr(robot, "_dry_run", False):
        mode_note = "联调 --dry-run（不发布）"
    else:
        backend = getattr(robot, "effective_backend", "?")
        mode_note = f"实机 → {backend}"
    print(f"{FRAME_NAME} 末端键盘控制已启动 — {mode_note}")
    sync_target_from_q()
    if robot is not None:
        robot.hold_current_pose()
    print(
        f"\n当前姿态为 IK 起点 q=[{', '.join(f'{v:+.2f}' for v in q)}] "
        f"(不自动解 IK，按空格可重新对齐)",
        flush=True,
    )

    with KeyReader() as keys:
        while True:
            k = keys.read_key()
            if not k:
                continue
            if k in ("\x03", "\x1b", "q", "Q"):
                if k == "\x1b":
                    print("\n退出")
                break
            if k in ("h", "H", "?"):
                print(HELP)
                continue
            if k == " ":
                sync_target_from_q()
                if robot is not None:
                    robot.hold_current_pose()
                print("\n已用当前关节角重置 IK 目标", end="", flush=True)
                continue
            if k in ("f", "F"):
                if robot is None:
                    print("\n夹爪需实机模式（默认 run 脚本已 --robot）", end="", flush=True)
                    continue
                opened = robot.toggle_claw()
                print(f"\n夹爪 → {'张开' if opened else '闭合'}", end="", flush=True)
                continue

            if not accept_key():
                continue

            arrow = ARROW_MAP.get(k[1:] if k.startswith("\x1b") else "", "")
            if arrow == "up":
                move(dpos=[pos_step, 0, 0])
            elif arrow == "down":
                move(dpos=[-pos_step, 0, 0])
            elif arrow == "left":
                move(dpos=[0, pos_step, 0])
            elif arrow == "right":
                move(dpos=[0, -pos_step, 0])
            elif arrow == "pgup":
                move(dpos=[0, 0, pos_step])
            elif arrow == "pgdown":
                move(dpos=[0, 0, -pos_step])
            elif k in ("w", "W"):
                move(dpos=[pos_step, 0, 0])
            elif k in ("s", "S"):
                move(dpos=[-pos_step, 0, 0])
            elif k in ("a", "A"):
                move(dpos=[0, pos_step, 0])
            elif k in ("d", "D"):
                move(dpos=[0, -pos_step, 0])
            elif k in ("r", "R"):
                move(dpos=[0, 0, pos_step])
            elif k in ("7", "i", "I"):
                move(drot=[rot_step, 0, 0])
            elif k in ("8", "k", "K"):
                move(drot=[-rot_step, 0, 0])
            elif k in ("4", "j", "J"):
                move(drot=[0, rot_step, 0])
            elif k in ("5", "l", "L"):
                move(drot=[0, -rot_step, 0])
            elif k in ("1", "u", "U"):
                move(drot=[0, 0, rot_step])
            elif k in ("2", "o", "O"):
                move(drot=[0, 0, -rot_step])

    if robot is not None:
        robot.stop()


def main():
    parser = argparse.ArgumentParser(description="键盘末端 IK（可选实机）")
    parser.add_argument(
        "--config",
        default=str(ROOT / "config" / "right_arm.yaml"),
    )
    parser.add_argument("--pos-step", type=float, default=0.01, help="平移步长 m")
    parser.add_argument(
        "--rot-step-deg", type=float, default=5.0, help="旋转步长 deg",
    )
    parser.add_argument(
        "--q-waist",
        type=float,
        default=0.0,
        help="当前 waist_yaw_joint 角 (rad)，用于 base↔torso 变换",
    )
    parser.add_argument(
        "--ik-mode",
        choices=("tool_z", "full", "position"),
        default="tool_z",
        help="IK: full=位置+姿态, tool_z=位置+末端Z, position=仅位置",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--robot",
        action="store_true",
        help="向实机发布（run_keyboard_demo.sh 默认已开启）",
    )
    mode.add_argument(
        "--sim-only",
        action="store_true",
        help="仅 IK，不连接 ROS / 不发实机",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="连接 ROS 但不发布 JointState",
    )
    parser.add_argument(
        "--no-fsm",
        action="store_true",
        help="不检查 FSM=EXEC_DEFAULT(5)",
    )
    parser.add_argument(
        "--develop",
        action="store_true",
        help="等待 FSM=EXEC_DEVELOP(16) 再连 lowlevel（完整控臂，推荐）",
    )
    args = parser.parse_args()

    mode_map = {
        "tool_z": IKTaskMode.POSITION_TOOL_Z,
        "full": IKTaskMode.POSITION_ORIENTATION,
        "position": IKTaskMode.POSITION,
    }
    solver = RightArmIKSolver.from_config(args.config)
    with open(args.config, encoding="utf-8") as f:
        cfg_all = yaml.safe_load(f) or {}
    default_q = cfg_all.get("default_arm_q")

    robot_bridge: Optional[RightArmRobotBridge] = None
    use_robot = args.robot or not args.sim_only
    if use_robot:
        import rospy

        rospy.init_node("arm_ik_keyboard_teleop", anonymous=True)
        rb_cfg = load_robot_config(args.config)
        robot_bridge = RightArmRobotBridge(
            solver.joint_names,
            rb_cfg,
            dry_run=args.dry_run,
            check_fsm=not args.no_fsm,
            prefer_develop=args.develop,
        )
        ready = robot_bridge.wait_for_ready()
        if not args.dry_run:
            be = robot_bridge.effective_backend
            if ready:
                if be == "lowlevel":
                    print("实机就绪: lowlevel 双臂；按 F 切换夹爪")
                else:
                    print(
                        "实机就绪: 仅右臂 → /pi_plus_absolute；"
                        "请先手柄 Start 进入站立 (fsm_state=5)",
                    )
            else:
                print(
                    "请先 Start 站立: rostopic echo /fsm_state 应为 5",
                )
        else:
            print("dry-run: 已连 ROS，不发布关节指令")

    try:
        run_teleop(
            solver,
            pos_step=args.pos_step,
            rot_step_deg=args.rot_step_deg,
            ik_mode=mode_map[args.ik_mode],
            q_waist=args.q_waist,
            robot=robot_bridge,
            default_q=default_q,
        )
    finally:
        if robot_bridge is not None:
            robot_bridge.stop()


if __name__ == "__main__":
    main()
