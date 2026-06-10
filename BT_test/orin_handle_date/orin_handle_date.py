#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Orin 2.4G 无线手柄接收器指令监听（与 sim2real / hand_identify_cpp 同一套 /joy 协议）。

默认订阅:
  /joy          joy_node 原始输出（2.4G USB 接收器 -> /dev/input/js0）
  /joy_input    robot_driver 滤波前输入（launch 里 remap 时）
  /cmd_vel      摇杆映射后的底盘速度（joy_teleop walk）
  /joy_msg      按键组合映射（sim2real_msg/Joy，若已安装）

用法:
  ./start.sh
  python3 orin_handle_date.py --joy-only
  python3 orin_handle_date.py --topic /joy
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, TextIO, Tuple

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# 与 hand_identify_cpp / joy.yaml 一致
JOY_ACTIVE_THRESH = 0.15
JOY_TRIGGER_AXIS_LT = 2
JOY_TRIGGER_AXIS_RT = 5
JOY_TRIGGER_REST = 1.0
JOY_TRIGGER_ACTIVE_MARGIN = 0.35

AXIS_LABELS = {
    0: "左摇杆X (linear.y)",
    1: "左摇杆Y (linear.x)",
    2: "LT 扳机",
    3: "右摇杆X (angular.z)",
    4: "右摇杆Y",
    5: "RT 扳机",
    6: "十字键X",
    7: "十字键Y",
}

BUTTON_LABELS = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "LB",
    5: "RB",
    6: "Back",
    7: "Start",
    8: "Center",
    9: "L3",
    10: "R3",
}


def ts() -> str:
    return time.strftime("%H:%M:%S")


def log(msg: str, raw_logger: Optional["RawDataLogger"] = None) -> None:
    line = f"[{ts()}] {msg}"
    print(line, flush=True)
    if raw_logger is not None:
        raw_logger.write_event("console", {"line": msg})


class RawDataLogger:
    """将手柄原始 ROS 消息按行写入 JSON 日志（jsonl）。"""

    def __init__(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._fp: TextIO = open(path, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self._count = 0
        self.write_event(
            "session_start",
            {
                "pid": os.getpid(),
                "cwd": os.getcwd(),
                "log_file": os.path.abspath(path),
            },
        )

    def close(self) -> None:
        with self._lock:
            self.write_event_unlocked("session_end", {"records": self._count})
            self._fp.flush()
            self._fp.close()

    def write_event(self, kind: str, payload: Dict[str, Any]) -> None:
        with self._lock:
            self.write_event_unlocked(kind, payload)

    def write_event_unlocked(self, kind: str, payload: Dict[str, Any]) -> None:
        record = {
            "ts": time.strftime("%Y-%m-%d %H:%M:%S")
            + f".{int(time.time() * 1000) % 1000:03d}",
            "kind": kind,
            **payload,
        }
        self._fp.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._fp.flush()
        self._count += 1

    def log_joy(self, topic: str, msg: Joy) -> None:
        self.write_event(
            "joy_raw",
            {
                "topic": topic,
                "header_seq": int(msg.header.seq),
                "header_stamp": f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}",
                "header_frame_id": msg.header.frame_id,
                "axes": [round(float(v), 6) for v in msg.axes],
                "buttons": [int(b) for b in msg.buttons],
            },
        )

    def log_twist(self, topic: str, msg: Twist) -> None:
        self.write_event(
            "cmd_vel_raw",
            {
                "topic": topic,
                "linear": {
                    "x": round(float(msg.linear.x), 6),
                    "y": round(float(msg.linear.y), 6),
                    "z": round(float(msg.linear.z), 6),
                },
                "angular": {
                    "x": round(float(msg.angular.x), 6),
                    "y": round(float(msg.angular.y), 6),
                    "z": round(float(msg.angular.z), 6),
                },
            },
        )

    def log_sim_joy(self, topic: str, msg) -> None:
        data = {
            "topic": topic,
            "a": float(msg.a),
            "b": float(msg.b),
            "x": float(msg.x),
            "y": float(msg.y),
            "lb": float(msg.lb),
            "rb": float(msg.rb),
            "lt": float(msg.lt),
            "rt": float(msg.rt),
            "l_horizontal": float(msg.l_horizontal),
            "l_vertical": float(msg.l_vertical),
            "r_horizontal": float(msg.r_horizontal),
            "r_vertical": float(msg.r_vertical),
            "dpad_horizontal": float(msg.dpad_horizontal),
            "dpad_vertical": float(msg.dpad_vertical),
            "back": float(msg.back),
            "start": float(msg.start),
            "center": float(msg.center),
            "L": float(msg.L),
            "R": float(msg.R),
        }
        self.write_event("joy_msg_raw", data)


def axis_active(idx: int, val: float) -> bool:
    if idx in (JOY_TRIGGER_AXIS_LT, JOY_TRIGGER_AXIS_RT):
        return val < (JOY_TRIGGER_REST - JOY_TRIGGER_ACTIVE_MARGIN)
    return abs(val) > JOY_ACTIVE_THRESH


def format_joy_summary(msg: Joy) -> str:
    parts: List[str] = []
    for i, v in enumerate(msg.axes):
        if axis_active(i, v):
            label = AXIS_LABELS.get(i, f"axis{i}")
            parts.append(f"{label}={v:+.2f}")
    for i, b in enumerate(msg.buttons):
        if b != 0:
            label = BUTTON_LABELS.get(i, f"btn{i}")
            parts.append(f"{label}=1")
    return " | ".join(parts) if parts else "(空闲)"


class JoyState:
    def __init__(
        self,
        name: str,
        axis_deadband: float = 0.05,
        raw_logger: Optional[RawDataLogger] = None,
    ):
        self.name = name
        self.axis_deadband = axis_deadband
        self.raw_logger = raw_logger
        self.last_axes: List[float] = []
        self.last_buttons: List[int] = []
        self.last_active_ts = 0.0
        self.msg_count = 0

    def on_joy(self, msg: Joy) -> None:
        self.msg_count += 1
        if self.raw_logger is not None:
            self.raw_logger.log_joy(self.name, msg)
        events: List[str] = []

        n_axes = max(len(msg.axes), len(self.last_axes))
        for i in range(n_axes):
            v = float(msg.axes[i]) if i < len(msg.axes) else 0.0
            old = float(self.last_axes[i]) if i < len(self.last_axes) else 0.0
            if abs(v - old) >= self.axis_deadband and (
                axis_active(i, v) or axis_active(i, old)
            ):
                label = AXIS_LABELS.get(i, f"axis{i}")
                events.append(f"{label}: {old:+.2f} -> {v:+.2f}")

        n_btn = max(len(msg.buttons), len(self.last_buttons))
        for i in range(n_btn):
            b = int(msg.buttons[i]) if i < len(msg.buttons) else 0
            old = int(self.last_buttons[i]) if i < len(self.last_buttons) else 0
            if b != old:
                label = BUTTON_LABELS.get(i, f"btn{i}")
                action = "按下" if b else "松开"
                events.append(f"{label} {action}")

        self.last_axes = list(msg.axes)
        self.last_buttons = list(msg.buttons)

        active = any(axis_active(i, float(v)) for i, v in enumerate(msg.axes))
        active = active or any(int(b) != 0 for b in msg.buttons)
        if active:
            self.last_active_ts = time.time()

        if events:
            log(f"[{self.name}] " + "; ".join(events), self.raw_logger)
            log(f"[{self.name}] 当前: {format_joy_summary(msg)}", self.raw_logger)
        elif self.msg_count == 1:
            log(
                f"[{self.name}] 已连接，轴数={len(msg.axes)} 键数={len(msg.buttons)}",
                self.raw_logger,
            )


class CmdVelState:
    def __init__(self, raw_logger: Optional[RawDataLogger] = None):
        self.last: Optional[Tuple[float, float, float]] = None
        self.raw_logger = raw_logger

    def on_twist(self, msg: Twist) -> None:
        if self.raw_logger is not None:
            self.raw_logger.log_twist("/cmd_vel", msg)
        cur = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self.last is None or any(abs(a - b) > 0.02 for a, b in zip(cur, self.last)):
            log(
                f"[cmd_vel] linear.x={cur[0]:+.3f} linear.y={cur[1]:+.3f} "
                f"angular.z={cur[2]:+.3f}",
                self.raw_logger,
            )
            self.last = cur


def try_import_sim2real_joy():
    try:
        from sim2real_msg.msg import Joy as SimJoy  # type: ignore

        return SimJoy
    except ImportError:
        return None


def format_sim_joy(msg) -> str:
    fields = [
        ("a", msg.a), ("b", msg.b), ("x", msg.x), ("y", msg.y),
        ("lb", msg.lb), ("rb", msg.rb), ("lt", msg.lt), ("rt", msg.rt),
        ("l_h", msg.l_horizontal), ("l_v", msg.l_vertical),
        ("r_h", msg.r_horizontal), ("r_v", msg.r_vertical),
    ]
    parts = [f"{k}={v:+.1f}" for k, v in fields if abs(v) > 0.05]
    return " | ".join(parts) if parts else "(无按键)"


def check_js_device(raw_logger: Optional[RawDataLogger] = None) -> None:
    dev = "/dev/input/js0"
    if os.path.exists(dev):
        log(f"检测到手柄设备 {dev}（2.4G 接收器 USB）", raw_logger)
    else:
        log(f"[warn] 未找到 {dev}，请插入 2.4G 接收器或检查 joy_node", raw_logger)


def main() -> int:
    parser = argparse.ArgumentParser(description="2.4G 手柄 /joy 指令监听")
    parser.add_argument("--topic", default="/joy", help="主监听话题 (默认 /joy)")
    parser.add_argument(
        "--also-joy-input",
        action="store_true",
        default=True,
        help="同时监听 /joy_input（默认开）",
    )
    parser.add_argument(
        "--no-joy-input",
        action="store_true",
        help="不监听 /joy_input",
    )
    parser.add_argument(
        "--joy-only",
        action="store_true",
        help="仅监听 /joy，不看 cmd_vel / joy_msg",
    )
    parser.add_argument(
        "--watch-cmd-vel",
        action="store_true",
        help="同时打印 /cmd_vel",
    )
    parser.add_argument(
        "--watch-joy-msg",
        action="store_true",
        help="同时打印 /joy_msg (需 sim2real_msg)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=2.0,
        help="空闲时心跳打印间隔 (0=关闭)",
    )
    parser.add_argument(
        "--log-file",
        default="",
        help="原始数据日志路径 (jsonl)，start.sh 默认自动创建",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="不写日志文件",
    )
    args = parser.parse_args()

    raw_logger: Optional[RawDataLogger] = None
    if not args.no_log and args.log_file:
        raw_logger = RawDataLogger(args.log_file)

    rospy.init_node("orin_handle_date", anonymous=True)

    check_js_device(raw_logger)
    log(">>> 2.4G 手柄指令监听 <<<", raw_logger)
    log(f"    主话题: {args.topic}", raw_logger)
    if not args.no_joy_input and args.also_joy_input:
        log("    副话题: /joy_input", raw_logger)
    if not args.joy_only:
        log("    扩展: /cmd_vel /joy_msg (可用参数单独开关)", raw_logger)
    if raw_logger is not None:
        log(f"    原始日志: {os.path.abspath(args.log_file)}", raw_logger)
    log("    操作手柄，终端显示变化；日志记录每条原始消息", raw_logger)
    print()

    states: Dict[str, JoyState] = {}

    def make_cb(name: str):
        st = JoyState(name, raw_logger=raw_logger)

        def _cb(msg: Joy):
            st.on_joy(msg)

        states[name] = st
        return _cb

    rospy.Subscriber(args.topic, Joy, make_cb(args.topic), queue_size=10)

    if not args.no_joy_input and args.also_joy_input and args.topic != "/joy_input":
        rospy.Subscriber("/joy_input", Joy, make_cb("/joy_input"), queue_size=10)

    cmd_state = None
    if args.watch_cmd_vel or not args.joy_only:
        cmd_state = CmdVelState(raw_logger=raw_logger)
        rospy.Subscriber("/cmd_vel", Twist, cmd_state.on_twist, queue_size=10)

    SimJoy = try_import_sim2real_joy()
    last_sim_joy = {"t": 0.0, "snap": ""}

    if SimJoy is not None and (args.watch_joy_msg or not args.joy_only):

        def on_joy_msg(msg):
            if raw_logger is not None:
                raw_logger.log_sim_joy("/joy_msg", msg)
            snap = format_sim_joy(msg)
            now = time.time()
            if snap != last_sim_joy["snap"] or now - last_sim_joy["t"] > 1.0:
                log(f"[joy_msg] {snap}", raw_logger)
                last_sim_joy["snap"] = snap
                last_sim_joy["t"] = now

        rospy.Subscriber("/joy_msg", SimJoy, on_joy_msg, queue_size=10)
        log("    已订阅 /joy_msg (sim2real_msg)", raw_logger)
    elif not args.joy_only:
        log("    [tip] 未安装 sim2real_msg，跳过 /joy_msg", raw_logger)

    if args.rate_hz > 0:
        def heartbeat(_evt):
            idle = True
            for st in states.values():
                if st.last_active_ts > 0 and time.time() - st.last_active_ts < 5.0:
                    idle = False
            if not idle:
                return True
            log(
                f"[心跳] 等待手柄输入... (已收 {sum(s.msg_count for s in states.values())} 条 /joy)",
                raw_logger,
            )
            return True

        rospy.Timer(rospy.Duration(1.0 / args.rate_hz), heartbeat)

    try:
        rospy.spin()
    finally:
        if raw_logger is not None:
            raw_logger.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
