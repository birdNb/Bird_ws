#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
右手肩关节 pitch / roll / yaw 力矩实时监控。

优先订阅:
  /error_joint_states  (sensor_msgs/JointState.effort, N·m)

默认关节:
  pitch → r_shoulder_pitch_joint  (绕 Y)
  roll  → r_shoulder_roll_joint   (绕 X)
  yaw   → r_upper_arm_joint       (绕 Z)

用法:
  ./run_monitor.sh
  ./run_monitor.sh --print-hz 20
  ./run_monitor.sh --plot --save shoulder_tau.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple


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
from sensor_msgs.msg import JointState

try:
    from livelybot_serial.msg import MotorState
except ImportError:
    MotorState = None  # type: ignore

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("MPLBACKEND"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


DEFAULT_JOINT_STATE_TOPIC = "/error_joint_states"
DEFAULT_PREFIX = "/livelybot_real_real"
PITCH_JOINT = "r_shoulder_pitch_joint"
ROLL_JOINT = "r_shoulder_roll_joint"
YAW_JOINT = "r_upper_arm_joint"
AXES = ("pitch", "roll", "yaw")


class TorqueMonitor:
    """采集右肩 pitch/roll/yaw 力矩。"""

    def __init__(
        self,
        joints: Dict[str, str],
        *,
        joint_state_topic: str,
        motor_prefix: str,
        source: str,
        window_sec: float,
    ):
        self.joints = dict(joints)  # label -> joint_name
        self.window_sec = max(float(window_sec), 1.0)
        self.source = source
        self._lock = threading.Lock()
        self.t0 = time.time()

        self.win_t: Deque[float] = deque()
        self.win: Dict[str, Deque[float]] = {a: deque() for a in AXES}
        self.all_t: List[float] = []
        self.all: Dict[str, List[float]] = {a: [] for a in AXES}

        self._latest: Dict[str, Optional[float]] = {a: None for a in AXES}
        self._msg_count = 0
        self._last_sample_t = -1.0
        self._min_dt = 0.01
        self._warned_missing = False

        if source == "joint_state":
            rospy.Subscriber(
                joint_state_topic,
                JointState,
                self._on_joint_state,
                queue_size=20,
                tcp_nodelay=True,
            )
            rospy.loginfo("[torque] 订阅 %s (JointState.effort)", joint_state_topic)
            rospy.loginfo(
                "[torque] joints: pitch=%s  roll=%s  yaw=%s",
                joints["pitch"],
                joints["roll"],
                joints["yaw"],
            )
        else:
            if MotorState is None:
                raise RuntimeError("无法 import livelybot_serial.MotorState，请 source sim2real")
            for label, joint in joints.items():
                topic = f"{motor_prefix.rstrip('/')}/{joint}_controller/state"
                rospy.Subscriber(
                    topic,
                    MotorState,
                    self._make_motor_cb(label),
                    queue_size=50,
                    tcp_nodelay=True,
                )
                rospy.loginfo("[torque] 订阅 %s (%s)", topic, label)

    def _ready(self) -> bool:
        return all(self._latest[a] is not None for a in AXES)

    def _append_sample(self, vals: Dict[str, float]) -> None:
        now = time.time() - self.t0
        if now - self._last_sample_t < self._min_dt:
            return
        self._last_sample_t = now
        self.win_t.append(now)
        self.all_t.append(now)
        for a in AXES:
            self.win[a].append(vals[a])
            self.all[a].append(vals[a])
        cutoff = now - self.window_sec
        while self.win_t and self.win_t[0] < cutoff:
            self.win_t.popleft()
            for a in AXES:
                self.win[a].popleft()

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        idxs = {a: name_to_i.get(self.joints[a]) for a in AXES}
        if any(i is None for i in idxs.values()):
            if not self._warned_missing:
                self._warned_missing = True
                missing = [self.joints[a] for a in AXES if idxs[a] is None]
                rospy.logwarn(
                    "[torque] JointState 缺少: %s；有: %s",
                    missing,
                    ", ".join(msg.name[:24]),
                )
            return
        if not msg.effort or max(idxs.values()) >= len(msg.effort):  # type: ignore[type-var]
            if not self._warned_missing:
                self._warned_missing = True
                rospy.logwarn("[torque] JointState.effort 为空，无法读力矩")
            return
        vals = {a: float(msg.effort[idxs[a]]) for a in AXES}  # type: ignore[index]
        with self._lock:
            self._latest.update(vals)
            self._msg_count += 1
            self._append_sample(vals)

    def _make_motor_cb(self, label: str):
        def _cb(msg: MotorState) -> None:
            with self._lock:
                self._latest[label] = float(msg.tau)
                self._msg_count += 1
                if not self._ready():
                    return
                self._append_sample({a: float(self._latest[a]) for a in AXES})

        return _cb

    def window_snapshot(
        self,
    ) -> Tuple[List[float], Dict[str, List[float]], int]:
        with self._lock:
            return (
                list(self.win_t),
                {a: list(self.win[a]) for a in AXES},
                self._msg_count,
            )

    def history_snapshot(
        self,
    ) -> Tuple[List[float], Dict[str, List[float]]]:
        with self._lock:
            return list(self.all_t), {a: list(self.all[a]) for a in AXES}

    def latest(self) -> Dict[str, Optional[float]]:
        with self._lock:
            return dict(self._latest)


def save_csv(
    path: str,
    t: List[float],
    series: Dict[str, List[float]],
    joints: Dict[str, str],
) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["t_sec"]
            + [f"tau_{joints[a]}_Nm" for a in AXES]
        )
        for i in range(len(t)):
            w.writerow(
                [f"{t[i]:.4f}"]
                + [f"{series[a][i]:.6f}" for a in AXES]
            )
    rospy.loginfo("[torque] 已保存 CSV: %s (%d 点)", path, len(t))


def _auto_ylim(ax, series: List[float]) -> None:
    if not series:
        return
    lo, hi = min(series), max(series)
    pad = max(0.05, 0.1 * (hi - lo) if hi > lo else 0.2)
    ax.set_ylim(lo - pad, hi + pad)


def _diagnose(topic: str) -> None:
    pubs = dict(rospy.get_published_topics())
    if topic in pubs:
        rospy.loginfo("[torque] 话题存在: %s  type=%s", topic, pubs[topic])
    else:
        matched = [t for t in pubs if "error_joint" in t or "livelybot_real" in t]
        rospy.logwarn(
            "[torque] 未找到 %s。相关话题: %s",
            topic,
            matched[:20] if matched else "(无)",
        )


def run(args: argparse.Namespace) -> None:
    rospy.init_node("r_shoulder_torque_monitor", anonymous=True)

    joints = {
        "pitch": args.pitch_joint,
        "roll": args.roll_joint,
        "yaw": args.yaw_joint,
    }

    if args.source == "auto":
        pubs = dict(rospy.get_published_topics())
        if args.joint_state_topic in pubs:
            source = "joint_state"
        else:
            rospy.logwarn(
                "[torque] %s 不存在，回退到 MotorState",
                args.joint_state_topic,
            )
            source = "motor_state"
    else:
        source = args.source

    if source == "joint_state":
        _diagnose(args.joint_state_topic)
    else:
        _diagnose(f"{args.prefix.rstrip('/')}/{args.pitch_joint}_controller/state")

    monitor = TorqueMonitor(
        joints,
        joint_state_topic=args.joint_state_topic,
        motor_prefix=args.prefix,
        source=source,
        window_sec=args.window,
    )

    rospy.loginfo("[torque] 等待力矩数据… (source=%s)", source)
    t_wait = time.time()
    last_print = 0.0
    while not rospy.is_shutdown():
        latest = monitor.latest()
        if all(latest[a] is not None for a in AXES):
            rospy.loginfo(
                "[torque] 首帧 pitch=%+.3f  roll=%+.3f  yaw=%+.3f N*m",
                latest["pitch"],
                latest["roll"],
                latest["yaw"],
            )
            break
        now = time.time()
        if now - last_print > 2.0:
            last_print = now
            rospy.loginfo(
                "[torque] 仍在等待… pitch=%s roll=%s yaw=%s (已 %.1fs)",
                latest["pitch"],
                latest["roll"],
                latest["yaw"],
                now - t_wait,
            )
        if now - t_wait > args.wait_timeout:
            rospy.logerr(
                "[torque] 超时无数据。请检查:\n"
                "  rostopic hz %s\n"
                "  rostopic echo %s -n 1",
                args.joint_state_topic,
                args.joint_state_topic,
            )
            return
        time.sleep(0.05)

    if args.plot:
        threading.Thread(
            target=_run_plot,
            args=(monitor, args, source, joints),
            daemon=True,
        ).start()

    print_hz = max(float(args.print_hz), 0.1)
    dt = 1.0 / print_hz
    stop_at = time.time() + args.duration if args.duration > 0 else None
    rospy.loginfo("[torque] 开始持续打印 pitch/roll/yaw (%.1f Hz)，Ctrl+C 退出", print_hz)
    try:
        while not rospy.is_shutdown():
            latest = monitor.latest()
            if all(latest[a] is not None for a in AXES):
                t_rel = time.time() - monitor.t0
                print(
                    "t=%7.2fs  pitch=%+.4f  roll=%+.4f  yaw=%+.4f  N*m"
                    % (t_rel, latest["pitch"], latest["roll"], latest["yaw"]),
                    flush=True,
                )
            if stop_at is not None and time.time() >= stop_at:
                rospy.loginfo("[torque] 到达 --duration，结束")
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        rospy.loginfo("[torque] 用户中断")

    if args.save:
        ht, hs = monitor.history_snapshot()
        if ht:
            save_csv(args.save, ht, hs, joints)
        else:
            rospy.logwarn("[torque] 无数据可保存到 %s", args.save)


def _run_plot(
    monitor: TorqueMonitor,
    args: argparse.Namespace,
    source: str,
    joints: Dict[str, str],
) -> None:
    if not os.environ.get("DISPLAY") and matplotlib.get_backend().lower() == "agg":
        rospy.logwarn("[torque] 无 DISPLAY，跳过绘图")
        return

    for style in ("seaborn-v0_8-whitegrid", "seaborn-whitegrid", "ggplot"):
        if style in plt.style.available:
            plt.style.use(style)
            break

    colors = {"pitch": "#1f77b4", "roll": "#2ca02c", "yaw": "#d62728"}
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle(
        "Right Shoulder Torque (%s)"
        % ("JointState.effort" if source == "joint_state" else "MotorState.tau"),
        fontsize=13,
    )
    lines = {}
    texts = {}
    for ax, a in zip(axes, AXES):
        (line,) = ax.plot([], [], color=colors[a], lw=1.5, label=a)
        lines[a] = line
        ax.set_ylabel("tau %s [N*m]" % a)
        ax.set_title(joints[a])
        ax.legend(loc="upper right")
        texts[a] = ax.text(0.01, 0.90, "", transform=ax.transAxes, fontsize=9)
    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()

    def update(_frame):
        if rospy.is_shutdown():
            plt.close(fig)
            return tuple(lines.values())
        t, series, count = monitor.window_snapshot()
        if t:
            for a in AXES:
                lines[a].set_data(t, series[a])
                _auto_ylim(axes[{"pitch": 0, "roll": 1, "yaw": 2}[a]], series[a])
                texts[a].set_text("tau=%+.3f N*m  n=%d" % (series[a][-1], count))
            axes[0].set_xlim(max(0.0, t[-1] - args.window), max(t[-1], 1.0))
        return tuple(lines.values())

    interval_ms = max(33, int(1000.0 / max(args.hz, 1.0)))
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)
    try:
        plt.show()
    finally:
        if getattr(ani, "event_source", None) is not None:
            ani.event_source.stop()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="监控右手肩 pitch/roll/yaw 力矩")
    p.add_argument(
        "--source",
        choices=("auto", "joint_state", "motor_state"),
        default="auto",
        help="数据源: auto 优先 /error_joint_states",
    )
    p.add_argument("--joint-state-topic", default=DEFAULT_JOINT_STATE_TOPIC)
    p.add_argument("--prefix", default=DEFAULT_PREFIX)
    p.add_argument("--pitch-joint", default=PITCH_JOINT)
    p.add_argument("--roll-joint", default=ROLL_JOINT)
    p.add_argument("--yaw-joint", default=YAW_JOINT)
    p.add_argument("--window", type=float, default=20.0)
    p.add_argument("--hz", type=float, default=20.0, help="绘图刷新频率")
    p.add_argument("--print-hz", type=float, default=10.0, help="终端打印频率")
    p.add_argument("--plot", action="store_true", help="同时弹曲线")
    p.add_argument("--duration", type=float, default=0.0, help="0=直到 Ctrl+C")
    p.add_argument("--wait-timeout", type=float, default=15.0)
    p.add_argument("--save", default="", help="退出后保存 CSV")
    return p


def main() -> None:
    args = build_parser().parse_args()
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
