#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE 文本指令 → ROS 话题桥接（对齐小程序遥控协议）。

摇杆:  X:0.00,Y:0.00,Z:0.00  → /cmd_vel
模式:  M_default / M_init / M_protect / M_resetzero / M_tech
动作:  LT+RT+start(起立) / LT+RT+RB(蹲下) / LT+RT+B(卸力)
       经 /joy → joy_teleop → /joy_msg（与实体手柄一致）
"""

from __future__ import annotations

import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set, Tuple, Union

# joy.yaml walk 缩放
SCALE_LINEAR_X = 1.5
SCALE_LINEAR_Y = 0.7
SCALE_ANGULAR_Z = 1.57

JOY_PUBLISH_HZ = 50
COMBO_HOLD_SEC = 1.0
MODE_PULSE_SEC = 0.45
STEP_GAP_SEC = 0.12
MENU_STEP_GAP_SEC = 0.35  # master joyMsgCallback 每步后 sleep 200ms
CMD_VEL_HZ = 20
CMD_VEL_TIMEOUT_SEC = 0.20
STICK_FILTER_ALPHA = 0.45
# 与小程序死区 10（-100~100 刻度）对齐 → |v| < 0.10 归零
STICK_DEADBAND = 0.10
STICK_XY_LIMIT = 1.0
STICK_Z_LIMIT = 1.5
ACTION_COOLDOWN_SEC = 1.0
MODE_COOLDOWN_SEC = 0.8
# 挥双手：短脉冲触发（勿 1s 长按，松开会被固件当成第二次触发）
CHEER_PULSE_SEC = 0.35
CHEER_COOLDOWN_SEC = 8.0
CHEER_RELEASE_FRAMES = 20

JOY_AXES_COUNT = 8
JOY_BUTTONS_COUNT = 11
AXIS_LT = 2
AXIS_RT = 5
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7
BTN_A = 0
BTN_B = 1
BTN_LB = 4
BTN_RB = 5
BTN_BACK = 6
BTN_START = 7
BTN_CENTER = 8

STICK_RE = re.compile(
    r"X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)"
    r"(?:\s*,\s*N:\s*\d+)?",
    re.IGNORECASE,
)

# 小程序模式指令标签（具体按键序列见 _build_mode_steps，依 FSM 状态生成）
MODE_LABELS: Dict[str, str] = {
    "m_default": "默认模式",
    "m_init": "初始化",
    "m_protect": "保护模式",
    "m_resetzero": "调零模式",
    "m_tech": "示教模式",
}

# sim2real_master joyMsgCallback 实测（反汇编）:
# - center==1 → DefaultModeEvt（无需按 LT/RT）
# - dpad/a/b 仅在 LT+RT 同时按下时有效；back 字段被忽略
# - dpad_h==1 → LastOption, ==-1 → NextOption, a==1 → Confirm
FSM_INIT = 0
FSM_PROTECTION = 8
FSM_CANDIDATE_DEFAULT = 2
FSM_CANDIDATE_CUSTOM = 3
FSM_CANDIDATE_REMOTE = 4
FSM_CANDIDATE_CALIBRATION = 9
FSM_CANDIDATE_TEACHING = 13
FSM_CANDIDATE_DEVELOP = 15
MENU_CANDIDATES = (
    FSM_CANDIDATE_DEFAULT,
    FSM_CANDIDATE_CUSTOM,
    FSM_CANDIDATE_REMOTE,
    FSM_CANDIDATE_CALIBRATION,
    FSM_CANDIDATE_TEACHING,
    FSM_CANDIDATE_DEVELOP,
)

# 预选动作（长按 1s / 顶栏按钮）
ACTION_COMMANDS: Dict[str, Tuple[str, str]] = {
    "lt+rt+start": ("起立", "lt+rt+start"),
    "lt+rt+rb": ("蹲下", "lt+rt+rb"),
    "lt+rt+b": ("卸力", "lt+rt+b"),
    "lt+rt+lb": ("步态启停", "lt+rt+lb"),
    "rt+a": ("挥双手", "rt+a"),
}

BUTTON_PRESS = 1.0
BUTTON_RELEASE = 0.0
TRIGGER_PRESS = -1.0
TRIGGER_RELEASE = 1.0

LogFn = Callable[[str], None]

FSM_STATE_NAMES = {
    0: "INIT",
    1: "ERROR",
    2: "CANDIDATE_DEFAULT",
    3: "CANDIDATE_CUSTOM",
    4: "CANDIDATE_REMOTE",
    5: "EXEC_DEFAULT",
    6: "EXEC_CUSTOM",
    7: "EXEC_REMOTE",
    8: "PROTECTION_SHUTDOWN",
    9: "CANDIDATE_CALIBRATION",
    10: "EXEC_CALIBRATING",
    11: "EXEC_CALIB_OK",
    12: "EXEC_CALIB_FAILED",
    13: "CANDIDATE_TEACHING",
    14: "EXEC_TEACHING",
    15: "CANDIDATE_DEVELOP",
    16: "EXEC_DEVELOP",
}


def _bootstrap_ros_python_path() -> None:
    """sudo 下 PYTHONPATH 可能丢失，补全 ROS / sim2real_msg 路径。"""
    extra = [
        "/opt/ros/noetic/lib/python3/dist-packages",
        os.path.expanduser("~/sim2real/devel/lib/python3/dist-packages"),
        "/home/nvidia/sim2real/devel/lib/python3/dist-packages",
        os.path.expanduser("~/sim2real/install/lib/python3/dist-packages"),
    ]
    for p in extra:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def _norm_token(text: str) -> str:
    return re.sub(r"\s+", "", text.strip().lower())


def _joy_key_value(key: str, pressed: bool) -> float:
    if key in ("lt", "rt"):
        return TRIGGER_PRESS if pressed else TRIGGER_RELEASE
    return BUTTON_PRESS if pressed else BUTTON_RELEASE


def _parse_key_combo(combo: str) -> Set[str]:
    return {p.strip().lower() for p in combo.split("+") if p.strip()}


@dataclass
class StickCommand:
    x: float
    y: float
    z: float


@dataclass
class ParsedCommand:
    stick: Optional[StickCommand] = None
    mode: Optional[str] = None
    action: Optional[str] = None


class BleCommandParser:
    """解析小程序 BLE 文本。"""

    def parse(self, text: str) -> ParsedCommand:
        text = text.strip()
        if not text:
            return ParsedCommand()

        result = ParsedCommand()
        m = STICK_RE.search(text)
        if m:
            z_raw = m.group(3)
            result.stick = StickCommand(
                x=_clamp_axis(float(m.group(1)), STICK_XY_LIMIT),
                y=_clamp_axis(float(m.group(2)), STICK_XY_LIMIT),
                z=_clamp_axis(float(z_raw), STICK_Z_LIMIT) if z_raw is not None else 0.0,
            )

        token = _norm_token(text)
        if token in MODE_LABELS:
            result.mode = token
            return result

        action = self._extract_action(text)
        if action is not None:
            result.action = action
        return result

    def _extract_action(self, text: str) -> Optional[str]:
        norm = _norm_token(text)
        if norm in ACTION_COMMANDS:
            return norm
        rest = STICK_RE.sub("", text)
        rest = re.sub(r"[,;|]", " ", rest)
        norm2 = _norm_token(rest)
        if norm2 in ACTION_COMMANDS:
            return norm2
        return None


def _clamp_axis(v: float, limit: float = STICK_XY_LIMIT) -> float:
    v = max(-limit, min(limit, v))
    if abs(v) < STICK_DEADBAND:
        return 0.0
    return v


def _stick_to_twist(stick: StickCommand):
    """BLE 摇杆 → /cmd_vel，与 joy.yaml 实体手柄一致。

    协议约定（小程序写入）:
      X 前后（前 +）  Y 左右（右 +）  Z 右转 +
    joy.yaml:
      axis1→linear.x  axis0→linear.y  axis3→angular.z
    Linux js0 前推 axis1 为负，故 linear.x 取反；右转 axis3 为正，Z 取反与现场一致。
    """
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.linear.x = -stick.x * SCALE_LINEAR_X
    msg.linear.y = stick.y * SCALE_LINEAR_Y
    msg.angular.z = -stick.z * SCALE_ANGULAR_Z
    return msg


def _fill_joy_neutral(msg) -> None:
    msg.lt = TRIGGER_RELEASE
    msg.rt = TRIGGER_RELEASE


def _joy_from_keys(keys: Set[str], pressed: bool):
    from sim2real_msg.msg import Joy

    msg = Joy()
    _fill_joy_neutral(msg)
    field_map = {
        "a": "a",
        "b": "b",
        "x": "x",
        "y": "y",
        "lb": "lb",
        "rb": "rb",
        "back": "back",
        "start": "start",
        "lt": "lt",
        "rt": "rt",
        "l": "L",
        "r": "R",
        "center": "center",
        "→": "dpad_horizontal",
        "←": "dpad_horizontal",
        "↑": "dpad_vertical",
        "↓": "dpad_vertical",
    }
    for key in keys:
        attr = field_map.get(key)
        if attr is None:
            continue
        if key == "→":
            setattr(msg, attr, -1.0 if pressed else 0.0)
        elif key == "←":
            setattr(msg, attr, 1.0 if pressed else 0.0)
        elif key == "↑":
            setattr(msg, attr, 1.0 if pressed else 0.0)
        elif key == "↓":
            setattr(msg, attr, -1.0 if pressed else 0.0)
        else:
            setattr(msg, attr, _joy_key_value(key, pressed))
    return msg


def _sensor_joy_from_token(token: str, pressed: bool):
    """单键或组合键 → sensor_msgs/Joy。"""
    if "+" in token:
        return _sensor_joy_from_keys(_parse_key_combo(token), pressed)
    return _sensor_joy_from_keys({token}, pressed)


def _sensor_joy_from_keys(keys: Set[str], pressed: bool):
    from sensor_msgs.msg import Joy

    msg = Joy()
    msg.axes = [0.0] * JOY_AXES_COUNT
    msg.buttons = [0] * JOY_BUTTONS_COUNT
    msg.axes[AXIS_LT] = TRIGGER_RELEASE
    msg.axes[AXIS_RT] = TRIGGER_RELEASE
    if not pressed:
        return msg

    for key in keys:
        if key == "lt":
            msg.axes[AXIS_LT] = TRIGGER_PRESS
        elif key == "rt":
            msg.axes[AXIS_RT] = TRIGGER_PRESS
        elif key == "start":
            msg.buttons[BTN_START] = 1
        elif key == "rb":
            msg.buttons[BTN_RB] = 1
        elif key == "b":
            msg.buttons[BTN_B] = 1
        elif key == "a":
            msg.buttons[BTN_A] = 1
        elif key == "lb":
            msg.buttons[BTN_LB] = 1
        elif key == "back":
            msg.buttons[BTN_BACK] = 1
        elif key == "center":
            msg.buttons[BTN_CENTER] = 1
        elif key in ("→", "right"):
            msg.axes[AXIS_DPAD_X] = -1.0
        elif key in ("←", "left"):
            msg.axes[AXIS_DPAD_X] = 1.0
        elif key in ("↑", "up"):
            msg.axes[AXIS_DPAD_Y] = 1.0
        elif key in ("↓", "down"):
            msg.axes[AXIS_DPAD_Y] = -1.0
    return msg


class BleRosBridge:
    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._parser = BleCommandParser()
        self._lock = threading.Lock()
        self._stick_target: Optional[StickCommand] = None
        self._stick_filtered: Optional[StickCommand] = None
        self._last_stick_ts = 0.0
        self._last_action_ts: Dict[str, float] = {}
        self._last_mode_ts: Dict[str, float] = {}
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._ros_thread: Optional[threading.Thread] = None
        self._cmd_pub = None
        self._sensor_joy_pub = None
        self._joy_msg_pub = None
        self._has_joy = False
        self._Twist = None
        self._cheer_running = False

    @property
    def ready(self) -> bool:
        return self._ready.is_set()

    def start(self) -> bool:
        if self._ros_thread is not None:
            return self.ready
        self._ros_thread = threading.Thread(target=self._ros_main, daemon=True)
        self._ros_thread.start()
        return self._ready.wait(timeout=8.0)

    def stop(self) -> None:
        self._stop.set()
        self._publish_zero_twist()
        if self._ros_thread is not None:
            self._ros_thread.join(timeout=2.0)

    def on_disconnect(self) -> None:
        with self._lock:
            self._stick_target = None
            self._stick_filtered = None
            self._last_stick_ts = 0.0
        self._cheer_running = False
        self._publish_zero_twist()
        self._flush_joy_release("rt+a")
        self._log("[ros] 断连急停：零速")

    def handle_command(self, kind: str, text: str) -> None:
        if kind == "stick":
            self.handle_text(text)
        elif kind == "mode":
            self.handle_text(text)
        elif kind == "action":
            self._trigger_action(text)

    def handle_text(self, text: str) -> None:
        if not self.ready:
            self._log("[ros] 桥接未就绪，忽略指令")
            return

        cmd = self._parser.parse(text)
        if cmd.stick is not None:
            with self._lock:
                self._stick_target = cmd.stick
                self._last_stick_ts = time.monotonic()
        if cmd.mode is not None:
            self._trigger_mode(cmd.mode)
        if cmd.action is not None:
            self._trigger_action(cmd.action)

    def _ros_main(self) -> None:
        _bootstrap_ros_python_path()
        try:
            import rospy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import Joy as SensorJoy
            from std_msgs.msg import Int32
        except ImportError as e:
            self._log(f"[ros] 未找到 rospy/sim2real_msg: {e}")
            self._log("[ros] 请用 ./start.sh 启动（内部 run_ble_with_ros.sh 会 source ROS）")
            return

        self._Twist = Twist
        try:
            rospy.init_node("ble_command_bridge", anonymous=True, disable_signals=True)
        except Exception as e:
            self._log(f"[ros] 初始化失败: {e}")
            return

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._sensor_joy_pub = rospy.Publisher("/joy", SensorJoy, queue_size=1)
        try:
            from sim2real_msg.msg import Joy  # noqa: F401

            self._joy_msg_pub = rospy.Publisher("/joy_msg", Joy, queue_size=1)
            self._has_joy = True
            self._log("[ros] 已启用 /cmd_vel + /joy + /joy_msg")
            self._log("[ros] 模式: M_default/M_init/M_protect/M_resetzero/M_tech")
            self._log("[ros] 动作: LT+RT+start/RB/B")
        except ImportError:
            self._has_joy = False
            self._log("[ros] 已启用 /cmd_vel + /joy（无 sim2real_msg）")

        self._last_fsm_state: Optional[int] = None

        def _on_fsm(msg: Int32) -> None:
            self._last_fsm_state = int(msg.data)

        rospy.Subscriber("/fsm_state", Int32, _on_fsm, queue_size=1)
        self._ready.set()
        self._log(f"[ros] /cmd_vel {CMD_VEL_HZ}Hz | 超时 {CMD_VEL_TIMEOUT_SEC}s | EMA={STICK_FILTER_ALPHA}")
        interval = 1.0 / CMD_VEL_HZ
        while not self._stop.is_set() and not rospy.is_shutdown():
            self._tick_cmd_vel()
            time.sleep(interval)

    def _ema_stick(
        self, target: StickCommand, prev: Optional[StickCommand]
    ) -> StickCommand:
        if prev is None:
            return target
        a = STICK_FILTER_ALPHA
        b = 1.0 - a
        return StickCommand(
            x=a * target.x + b * prev.x,
            y=a * target.y + b * prev.y,
            z=a * target.z + b * prev.z,
        )

    def _tick_cmd_vel(self) -> None:
        if self._cmd_pub is None:
            return
        now = time.monotonic()
        with self._lock:
            target = self._stick_target
            age = now - self._last_stick_ts
        if target is None or age > CMD_VEL_TIMEOUT_SEC:
            with self._lock:
                self._stick_filtered = None
            self._publish_zero_twist()
            return
        with self._lock:
            filtered = self._ema_stick(target, self._stick_filtered)
            self._stick_filtered = filtered
        self._cmd_pub.publish(_stick_to_twist(filtered))

    def _publish_zero_twist(self) -> None:
        if self._cmd_pub is None or self._Twist is None:
            return
        self._cmd_pub.publish(self._Twist())

    def _steps_enter_init_menu(self, fsm: Optional[int]) -> List[str]:
        """进入 FSM Init 菜单（候选模式选择界面）。"""
        if fsm == FSM_INIT:
            return []
        if fsm == FSM_PROTECTION:
            return ["lt+rt+b"]
        if fsm in MENU_CANDIDATES:
            return []
        return ["lt+rt+b", "lt+rt+b"]

    def _steps_navigate_to_candidate(
        self, fsm: Optional[int], target: int
    ) -> List[str]:
        """在 Init/候选菜单中导航到目标项并 A 确认（须 LT+RT+方向/A）。"""
        steps: List[str] = []
        cur = fsm

        if cur == FSM_INIT or cur is None or cur == FSM_PROTECTION:
            steps.append("lt+rt+→")
            cur = FSM_CANDIDATE_DEFAULT
        elif cur not in MENU_CANDIDATES:
            steps.append("lt+rt+→")
            cur = FSM_CANDIDATE_DEFAULT

        idx_cur = MENU_CANDIDATES.index(cur)
        idx_tgt = MENU_CANDIDATES.index(target)
        if idx_tgt > idx_cur:
            steps.extend(["lt+rt+→"] * (idx_tgt - idx_cur))
        elif idx_tgt < idx_cur:
            steps.extend(["lt+rt+←"] * (idx_cur - idx_tgt))
        steps.append("lt+rt+a")
        return steps

    def _build_mode_steps(self, mode_key: str) -> Tuple[str, List[str]]:
        label = MODE_LABELS[mode_key]
        fsm = self._last_fsm_state

        if mode_key == "m_default":
            return label, ["center"]
        if mode_key == "m_protect":
            return label, ["lt+rt+b"]
        if mode_key == "m_init":
            return label, self._steps_enter_init_menu(fsm)
        if mode_key == "m_resetzero":
            prefix = self._steps_enter_init_menu(fsm)
            start = FSM_INIT if prefix else fsm
            return label, prefix + self._steps_navigate_to_candidate(
                start, FSM_CANDIDATE_CALIBRATION
            )
        if mode_key == "m_tech":
            prefix = self._steps_enter_init_menu(fsm)
            start = FSM_INIT if prefix else fsm
            return label, prefix + self._steps_navigate_to_candidate(
                start, FSM_CANDIDATE_TEACHING
            )
        return label, []

    def _trigger_mode(self, mode_key: str) -> None:
        if mode_key not in MODE_LABELS:
            return
        now = time.monotonic()
        if now - self._last_mode_ts.get(mode_key, 0.0) < MODE_COOLDOWN_SEC:
            return
        self._last_mode_ts[mode_key] = now

        label, steps = self._build_mode_steps(mode_key)
        fsm = self._last_fsm_state
        fsm_name = FSM_STATE_NAMES.get(fsm, str(fsm)) if fsm is not None else "?"
        self._log(f"[ros] 模式 {label}: {mode_key} | 起始FSM={fsm}({fsm_name})")
        self._log(f"[ros]   序列({len(steps)}步): {steps}")
        if not steps:
            self._log(f"[ros]   已在目标菜单状态，无需操作")
            return

        self._publish_zero_twist()
        with self._lock:
            self._stick_target = None
            self._stick_filtered = None

        gap = MENU_STEP_GAP_SEC if len(steps) > 1 else STEP_GAP_SEC
        threading.Thread(
            target=self._run_steps,
            args=(steps, label, MODE_PULSE_SEC, gap),
            daemon=True,
        ).start()

    def _trigger_action(self, action_key: str) -> None:
        if action_key not in ACTION_COMMANDS:
            return
        if action_key == "rt+a":
            self._trigger_cheer()
            return

        now = time.monotonic()
        if now - self._last_action_ts.get(action_key, 0.0) < ACTION_COOLDOWN_SEC:
            return
        self._last_action_ts[action_key] = now

        label, combo = ACTION_COMMANDS[action_key]
        self._log(f"[ros] 动作 {label}: {combo} → /joy ({COMBO_HOLD_SEC}s)")
        self._publish_zero_twist()
        with self._lock:
            self._stick_target = None
            self._stick_filtered = None

        threading.Thread(
            target=self._run_steps,
            args=([combo], label, COMBO_HOLD_SEC),
            daemon=True,
        ).start()

    def _trigger_cheer(self) -> None:
        now = time.monotonic()
        if self._cheer_running:
            self._log("[ros] 挥双手执行中，忽略重复指令")
            return
        if now - self._last_action_ts.get("rt+a", 0.0) < CHEER_COOLDOWN_SEC:
            self._log("[ros] 挥双手冷却中，忽略")
            return
        self._last_action_ts["rt+a"] = now
        self._cheer_running = True
        self._log(f"[ros] 挥双手: RT+A 短脉冲 {CHEER_PULSE_SEC}s（单次触发）")
        self._publish_zero_twist()
        with self._lock:
            self._stick_target = None
            self._stick_filtered = None
        threading.Thread(target=self._run_cheer_once, daemon=True).start()

    def _run_cheer_once(self) -> None:
        try:
            self._run_steps(["rt+a"], "挥双手", CHEER_PULSE_SEC, step_gap=0.0)
            self._flush_joy_release("rt+a", CHEER_RELEASE_FRAMES)
        finally:
            self._cheer_running = False

    def _wait_joy_path(self) -> None:
        import rospy

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not self._stop.is_set():
            if self._has_joy and self._joy_msg_pub:
                if self._joy_msg_pub.get_num_connections() > 0:
                    return
            elif self._sensor_joy_pub and self._sensor_joy_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.05)
        topic = "/joy_msg" if self._has_joy else "/joy"
        self._log(f"[ros][warn] {topic} 无订阅者(sim2real_master)，指令可能无效")

    def _flush_joy_release(self, combo: str, frames: int = 10) -> None:
        """连发松开帧，避免释放沿被固件当成第二次触发。"""
        interval = 1.0 / JOY_PUBLISH_HZ
        for _ in range(frames):
            if self._stop.is_set():
                break
            self._publish_token(combo, False)
            time.sleep(interval)

    def _publish_token(self, token: str, pressed: bool) -> None:
        """模式/动作优先直发 /joy_msg，避免与 joy_node 抢占 /joy。"""
        keys = _parse_key_combo(token) if "+" in token else {token}
        joy = _joy_from_keys(keys, pressed)
        if self._has_joy and self._joy_msg_pub is not None:
            self._joy_msg_pub.publish(joy)
        elif self._sensor_joy_pub is not None:
            sensor = _sensor_joy_from_token(token, pressed)
            self._sensor_joy_pub.publish(sensor)

    def _run_steps(
        self,
        steps: List[str],
        label: str,
        hold_sec: float,
        step_gap: float = STEP_GAP_SEC,
    ) -> None:
        self._wait_joy_path()
        interval = 1.0 / JOY_PUBLISH_HZ
        n_hold = max(1, int(hold_sec * JOY_PUBLISH_HZ))
        for step in steps:
            if self._stop.is_set():
                break
            for _ in range(n_hold):
                if self._stop.is_set():
                    break
                self._publish_token(step, True)
                time.sleep(interval)
            for _ in range(3):
                if self._stop.is_set():
                    break
                self._publish_token(step, False)
                time.sleep(interval)
            time.sleep(step_gap)
        fsm = self._last_fsm_state
        if fsm is not None:
            name = FSM_STATE_NAMES.get(fsm, str(fsm))
            self._log(f"[ros] 完成: {label} | 当前 FSM={fsm} ({name})")
        else:
            self._log(f"[ros] 完成: {label}")
