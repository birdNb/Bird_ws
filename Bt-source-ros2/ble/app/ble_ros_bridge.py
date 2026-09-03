#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE 文本指令 → ROS2 量产算法桥接（Foxy / rclpy）。

优先走非手柄服务（与量产文档一致）：
  /hightorque_controller/change_fsm_state   底层 FSM
  /hightorque_controller/change_state       default_bt 上层状态
  /hightorque_controller/switch_policy      bfm → amp
失败时回退到 /joy 组合键，保留 ROS1 小程序操作习惯。

摇杆:  X,Y,Z → /joy（AMP）；BFM 运行时改发 /cmd_vel（符号+阈值方向映射）
起立:  LT+RT+START（服务 standing / 手柄兜底）
步态:  GAIT ON/OFF → 保持当前 walk 策略(bfm/amp/amp_lower) 后 toggle_policy
坐下:  LT+RT+RB / ST:sit（running 时先自动 toggle→standby，再 siting；BFM 不可直接蹲）
加速:  LT ON → LT 扳机 axes[2]=-1
急停:  LT+RT+B → 服务 protect，失败则手柄
"""

from __future__ import annotations

import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

JOY_PUBLISH_HZ = 50
STICK_FILTER_ALPHA = 0.45
STICK_DEADBAND = 0.10
STICK_XY_LIMIT = 1.8
STICK_Z_LIMIT = 1.5
STICK_TIMEOUT_SEC = 0.20
COMBO_HOLD_SEC = 1.00
COMBO_PULSE_SEC = 0.35
AMP_SWITCH_SEC = 0.50
ESTOP_HOLD_SEC = 0.60
POWER_ON_SETTLE_SEC = 2.5
SERVICE_WAIT_SEC = 0.8
SERVICE_CALL_SEC = 2.5
STATE_WAIT_SEC = 8.0
POLICY_WAIT_SEC = 8.0
STANDING_WAIT_SEC = 20.0
JOINT_STATES_WAIT_SEC = 15.0
IMU_WAIT_SEC = 15.0
IMU_STALE_SEC = 1.0
AMP_POLICY_NAME = "amp"
WALK_POLICIES = frozenset({"amp", "amp_lower", "bfm"})

# BFM motion source：只用符号/阈值选方向，示例幅值 0.8；激活 0.3 / 释放 0.25
BFM_CMD_LEVEL = 0.8
BFM_ACTIVATE_THRESH = 0.3
BFM_RELEASE_THRESH = 0.25

# BLE POL:* / 量产手柄可切策略（与 input_arbiter_walk allowed_policies + 编舞一致）
ALLOWED_POLICIES = frozenset(
    {
        "bfm",
        "amp",
        "amp_lower",
        "byd_small_kick",
        "byd_power",
        "byd_bb",
        "byd_zzx",
        "pi_plus_shanggouquan",
        "pi_plus_zhidengtui",
        "pi_plus_zhongquan",
        "pi_plus_zoo",
        "pi_plus_guanjun",
        "SP8",
    }
)

# 手柄误触禁止切 BFM/amp_lower；显式 POL:* 不受此限
BLOCKED_POLICY_COMBOS = frozenset(
    {
        "lt+y",
        "lt+rt+x",
        "lt+rs",
        "lt+rt+rs",
        "lt+rb+ls",
    }
)

JOY_AXES_COUNT = 8
JOY_BUTTONS_COUNT = 11
AXIS_LX = 0
AXIS_LY = 1
AXIS_LT = 2
AXIS_RX = 3
AXIS_RY = 4
AXIS_RT = 5
AXIS_HAT_X = 6
AXIS_HAT_Y = 7

BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5
BTN_BACK = 6
BTN_START = 7
BTN_GUIDE = 8
BTN_LS = 9
BTN_RS = 10

TRIGGER_PRESS = -1.0
TRIGGER_RELEASE = 1.0

_BTN_MAP = {
    "a": BTN_A,
    "b": BTN_B,
    "x": BTN_X,
    "y": BTN_Y,
    "lb": BTN_LB,
    "rb": BTN_RB,
    "back": BTN_BACK,
    "start": BTN_START,
    "center": BTN_GUIDE,
    "guide": BTN_GUIDE,
    "ls": BTN_LS,
    "l3": BTN_LS,
    "rs": BTN_RS,
    "r3": BTN_RS,
}

SWITCH_POLICY_CANDIDATES = (
    "/hightorque_controller/switch_policy",
    "/default_bt/switch_policy",
    "/switch_policy",
)
CHANGE_STATE_CANDIDATES = (
    "/hightorque_controller/change_state",
    "/default_bt/change_state",
)
CHANGE_FSM_CANDIDATES = (
    "/hightorque_controller/change_fsm_state",
    "/change_fsm_state",
)
START_POLICY_CANDIDATES = (
    "/hightorque_controller/start_policy",
    "/default_bt/start_policy",
)
STOP_POLICY_CANDIDATES = (
    "/hightorque_controller/stop_policy",
    "/default_bt/stop_policy",
)
CONTROLLER_STATE_TOPICS = (
    "/hightorque_controller/state",
    "/hightorque_controller/controller_state",
    "/controller_state",
)
FSM_STATE_TOPICS = ("/fsm_state",)

FSM_INIT = 0
FSM_ERROR = 1
FSM_CANDIDATE_DEFAULT = 2
FSM_EXEC_DEFAULT = 5
FSM_PROTECT = 8
FSM_CANDIDATE_RESET_ZERO = 9
FSM_EXEC_RESET_ZERO = 10
FSM_RESET_OK = 11
FSM_RESET_FAIL = 12

FSM_STATE_NAMES = {
    FSM_INIT: "INIT",
    FSM_ERROR: "ERROR",
    FSM_CANDIDATE_DEFAULT: "CANDIDATE_DEFAULT",
    FSM_EXEC_DEFAULT: "EXEC_DEFAULT",
    FSM_PROTECT: "PROTECTION_SHUTDOWN",
    FSM_CANDIDATE_RESET_ZERO: "CANDIDATE_RESET_ZERO",
    FSM_EXEC_RESET_ZERO: "EXEC_RESET_ZERO",
    FSM_RESET_OK: "EXEC_RESET_ZERO_SUCCESSFULLY",
    FSM_RESET_FAIL: "EXEC_RESET_ZERO_FAILED",
}

# 小程序模式菜单发的是手柄组合键，不是 M_* 文本
FSM_MENU_STATES = frozenset(
    {FSM_INIT, FSM_ERROR, FSM_CANDIDATE_DEFAULT, FSM_CANDIDATE_RESET_ZERO}
)
FSM_PUBLISH_PERIOD_SEC = 0.5

STICK_RE = re.compile(
    r"X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)"
    r"(?:\s*,\s*N:\s*\d+)?",
    re.IGNORECASE,
)

MODE_LABELS: Dict[str, str] = {
    "m_init": "初始化",
    "m_protect": "保护模式",
    "m_resetzero": "调零模式",
    "m_tech": "示教模式",
}

_POWER_GATED_KINDS = frozenset(
    {"stick", "action", "gait", "sprint", "upper_state", "policy"}
)

from ble_neck_bridge import NeckController
from ble_motor_power_manager import MotorPowerController, POWER_TOPIC

try:
    from ble_gamepad import (
        is_hold_combo,
        label_for_combo,
        parse_gamepad_combo,
    )
except ImportError:
    is_hold_combo = None  # type: ignore
    label_for_combo = lambda c: c  # type: ignore
    parse_gamepad_combo = None  # type: ignore

LogFn = Callable[[str], None]


def _bootstrap_ros_python_path() -> None:
    """sudo 下 PYTHONPATH 可能丢失，补全 ROS2 Foxy / colcon 路径。"""
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


def _norm_token(text: str) -> str:
    return re.sub(r"\s+", "", text.strip().lower())


def _norm_label(text: Optional[str]) -> str:
    return (text or "").strip().lower()


def _canonical_policy(name: Optional[str]) -> Optional[str]:
    raw = (name or "").strip()
    if not raw:
        return None
    if raw.upper() == "SP8":
        return "SP8"
    low = raw.lower()
    for p in ALLOWED_POLICIES:
        if p.lower() == low:
            return p
    return None


def _clamp_axis(v: float, limit: float = STICK_XY_LIMIT) -> float:
    v = max(-limit, min(limit, v))
    if abs(v) < STICK_DEADBAND:
        return 0.0
    return v


def _to_unit(v: float, limit: float) -> float:
    if limit <= 0:
        return 0.0
    return max(-1.0, min(1.0, v / limit))


def _combo_is_hold(combo: str) -> bool:
    if is_hold_combo is not None:
        try:
            return bool(is_hold_combo(combo))
        except Exception:
            pass
    return combo in {
        "lt+rt+start",
        "lt+rt+rb",
        "lt+rt+b",
        "lt+rt+lb",
        "lt+ls",
        "lt+rt+ls",
    }


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
        if token == "m_default":
            return result
        if token in MODE_LABELS:
            result.mode = token
            return result

        action = self._extract_action(text)
        if action is not None:
            result.action = action
        return result

    def _extract_action(self, text: str) -> Optional[str]:
        if parse_gamepad_combo is not None:
            combo = parse_gamepad_combo(text)
            if combo is not None:
                return combo
        rest = STICK_RE.sub("", text)
        rest = re.sub(r"[,;|]", " ", rest)
        if parse_gamepad_combo is not None:
            combo2 = parse_gamepad_combo(rest)
            if combo2 is not None:
                return combo2
        return None


class BleRosBridge:
    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._parser = BleCommandParser()
        self._lock = threading.Lock()
        self._stick_target: Optional[StickCommand] = None
        self._stick_filtered: Optional[StickCommand] = None
        self._last_stick_ts = 0.0
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._ros_thread: Optional[threading.Thread] = None
        self._node = None
        self._executor = None
        self._joy_pub = None
        self._ht_joy_pub = None
        self._cmd_vel_pub = None
        self._Joy = None
        self._Twist = None
        self._HightorqueJoy = None
        self._bfm_dir = (0, 0, 0)  # linear.x / linear.y / angular.z 符号 -1/0/1
        self._bfm_cmd_active = False
        self._neck = NeckController(log=log)
        self._motor_power = MotorPowerController(log=log)
        self._battery_pct: Optional[int] = None
        self._sprint_enabled = False
        self._policy_on = False
        self._combo_parts: List[str] = []
        self._combo_until = 0.0
        self._combo_zero_stick = False
        self._action_listener = None
        self._gait_listener = None
        self._mode_listener = None
        self._sprint_listener = None
        self._power_ready_at = 0.0
        self._SwitchPolicy = None
        self._ChangeState = None
        self._Common = None
        self._policy_clients = []
        self._state_clients = []
        self._fsm_clients = []
        self._start_policy_clients = []
        self._stop_policy_clients = []
        self._current_policy = ""
        self._current_state = ""
        self._current_mode = ""
        self._fsm_value: Optional[int] = FSM_EXEC_DEFAULT
        self._fsm_pub = None
        self._Int32 = None
        self._last_fsm_pub_ts = 0.0
        self._last_mode_wire = ""
        self._gait_lock = threading.Lock()
        self._gait_busy = False
        self._joint_states_received = False
        self._joint_state_count = 0
        self._joint_state_names: set = set()
        self._imu_received = False
        self._last_imu_ts = 0.0
        self._stand_lock = threading.Lock()
        self._stand_busy = False

    @property
    def ready(self) -> bool:
        return self._ready.is_set()

    def get_motor_power_wire(self) -> Optional[str]:
        return self._motor_power.get_state_wire()

    def get_motor_power_intent_wire(self) -> Optional[str]:
        return self._motor_power.get_intent_wire()

    def set_motor_power_listener(self, fn) -> None:
        self._motor_power.set_state_listener(fn)

    def set_action_listener(self, fn) -> None:
        self._action_listener = fn

    def set_gait_listener(self, fn) -> None:
        self._gait_listener = fn

    def set_mode_listener(self, fn) -> None:
        self._mode_listener = fn

    def set_sprint_listener(self, fn) -> None:
        self._sprint_listener = fn

    def get_battery_pct(self) -> Optional[int]:
        pct = self._battery_pct
        if pct is None:
            return None
        return max(0, min(100, int(pct)))

    def get_gait_wire(self) -> str:
        if _norm_label(self._current_state) == "running":
            return "ON"
        if self._policy_on:
            return "ON"
        return "OFF"

    def get_fsm_value(self) -> Optional[int]:
        if self._fsm_value is None:
            return FSM_EXEC_DEFAULT
        return self._fsm_value

    def get_mode_wire(self) -> str:
        """小程序主显示字段 mode:M_* 。以控制器 current_mode 为准，避免本地残留 fsm 覆盖。"""
        fsm = self.get_fsm_value()
        mode = _norm_label(self._current_mode)
        if mode == "protect" or fsm == FSM_PROTECT:
            return "M_protect"
        if mode == "reset_zero" or fsm in (
            FSM_CANDIDATE_RESET_ZERO,
            FSM_EXEC_RESET_ZERO,
            FSM_RESET_OK,
            FSM_RESET_FAIL,
        ):
            return "M_resetzero"
        # default_bt 已执行默认模式：勿被残留 INIT/ERROR 打回 M_init
        if mode == "default_bt":
            return "M_default"
        if mode == "init" or fsm in (FSM_INIT, FSM_ERROR, FSM_CANDIDATE_DEFAULT):
            return "M_init"
        return "M_default"

    def start(self) -> bool:
        if self._ros_thread is not None:
            return self.ready
        self._ros_thread = threading.Thread(target=self._ros_main, daemon=True)
        self._ros_thread.start()
        if self._ready.wait(timeout=0.05):
            return True
        self._log("[ros] 后台等待 ROS2 量产算法（不阻塞 BLE 广播）")
        return False

    def stop(self) -> None:
        self._stop.set()
        if self._ros_thread is not None:
            self._ros_thread.join(timeout=2.0)

    def _publish_zero_twist(self) -> None:
        """兼容 GATT 停拖拽时的回中调用。"""
        self.on_disconnect()

    def on_disconnect(self) -> None:
        with self._lock:
            self._stick_target = None
            self._stick_filtered = None
            self._last_stick_ts = 0.0
            self._sprint_enabled = False
            self._combo_parts = []
            self._combo_until = 0.0
            self._combo_zero_stick = False
        self._set_sprint(False, log=False)
        self._publish_cmd_vel(0.0, 0.0, 0.0)
        self._bfm_dir = (0, 0, 0)
        self._bfm_cmd_active = False
        self._log("[ros] 断连急停：摇杆回中")

    def handle_command(self, kind: str, text: str) -> None:
        if kind in _POWER_GATED_KINDS:
            ok, reason = self._check_motion_allowed(kind)
            if not ok:
                self._log(f"[ros] 拒绝 {kind}: {reason}")
                return
        if kind == "stick":
            self.handle_text(text)
        elif kind == "mode":
            # 直接走模式入口，避免二次 parse 失败导致“指令无效果”
            key = (text or "").strip().lower()
            if key in MODE_LABELS:
                self._trigger_mode(key)
            else:
                self.handle_text(text)
        elif kind == "action":
            self._trigger_action(text)
        elif kind == "neck":
            self._neck.enqueue(text)
        elif kind == "motor_power":
            self._handle_motor_power(text.strip().upper())
        elif kind == "gait":
            self._trigger_gait(text.strip().upper())
        elif kind == "sprint":
            self._set_sprint(text.strip().upper() == "ON")
        elif kind == "upper_state":
            self._trigger_upper_state(text)
        elif kind == "fsm":
            self._trigger_fsm(text)
        elif kind == "policy":
            self._trigger_policy(text)

    def _joy_path_alive(self) -> bool:
        for pub in (self._joy_pub, self._ht_joy_pub):
            try:
                if pub is not None and int(pub.get_subscription_count()) > 0:
                    return True
            except Exception:
                continue
        return False

    def _services_alive(self) -> bool:
        for group in (self._policy_clients, self._state_clients, self._fsm_clients):
            for client in group:
                try:
                    if client.service_is_ready():
                        return True
                except Exception:
                    continue
        return False

    def _check_motion_allowed(self, kind: str) -> Tuple[bool, str]:
        wire = self._motor_power.get_intent_wire()
        if wire == "OFF":
            return False, "电机电源未开启(mp=OFF)，请先 MP ON"
        if wire == "ON" and not self._motor_power_ready_for_motion():
            left = max(0.0, self._power_ready_at - time.monotonic())
            return False, f"上电未就绪，请再等 {left:.1f}s"
        if kind in ("gait", "action", "upper_state") and self.ready:
            if not self._joy_path_alive() and not self._services_alive():
                return False, "量产控制器未启动，无法起立/行走。请重启 ROS2 启动脚本"
        return True, ""

    def _handle_motor_power(self, action: str) -> None:
        if action == "OFF":
            with self._lock:
                self._stick_target = None
                self._stick_filtered = None
                self._sprint_enabled = False
                self._combo_parts = ["lt", "rt"]
                self._combo_until = time.monotonic() + ESTOP_HOLD_SEC
                self._combo_zero_stick = True
            self._motor_power.apply_immediate("OFF", soft=True)
            self._power_ready_at = 0.0
            self._policy_on = False
            return
        if action == "ON":
            self._motor_power.apply_immediate("ON")
            self._power_ready_at = time.monotonic() + POWER_ON_SETTLE_SEC
            self._log(f"[MP] 上电完成，再等 {POWER_ON_SETTLE_SEC:.1f}s 后允许走路/进策略")

    def _motor_power_ready_for_motion(self) -> bool:
        wire = self._motor_power.get_intent_wire()
        if wire != "ON":
            return wire is None
        return time.monotonic() >= self._power_ready_at

    def handle_text(self, text: str) -> None:
        if not self.ready:
            self._log("[ros] 桥接未就绪（等待 ROS2），忽略指令")
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
        import traceback

        delay = 2.0
        while not self._stop.is_set():
            try:
                self._ros_main_impl()
                return
            except Exception as e:
                self._ready.clear()
                self._log(f"[ros] 控制桥异常: {e}")
                self._log(traceback.format_exc())
                if self._stop.wait(timeout=delay):
                    return
                self._log(f"[ros] {delay:.0f}s 后重试 ROS2 控制桥…")
                delay = min(delay * 1.5, 30.0)

    def _wait_dds_domain_ready(self, timeout: float = 60.0) -> bool:
        """探测 CycloneDDS 能否建域；失败只 sleep，避免 create_node 狂刷打满 CPU。"""
        _bootstrap_ros_python_path()
        deadline = time.monotonic() + timeout
        attempt = 0
        while not self._stop.is_set() and time.monotonic() < deadline:
            attempt += 1
            try:
                import rclpy
                from rclpy.node import Node

                if not rclpy.ok():
                    rclpy.init(args=None)
                n = Node("ble_dds_probe")
                n.destroy_node()
                return True
            except Exception as e:
                if attempt == 1 or attempt % 5 == 0:
                    self._log(f"[ros] 等待 DDS… ({e})")
                if self._stop.wait(timeout=1.5):
                    return False
        return False

    def _wait_ros2_stack_ready(self, timeout: float = 180.0) -> bool:
        """等 controller + midware 进程就绪再建桥（与 bird-ble-boot 一致）。"""
        deadline = time.monotonic() + timeout
        warned = False
        while not self._stop.is_set() and time.monotonic() < deadline:
            try:
                import subprocess

                ctrl = subprocess.run(
                    ["pgrep", "-f", "hightorque_controller_node"],
                    capture_output=True,
                    check=False,
                )
                mid = subprocess.run(
                    ["pgrep", "-f", "hightorque_midware_node"],
                    capture_output=True,
                    check=False,
                )
                if ctrl.returncode == 0 and mid.returncode == 0:
                    return not self._stop.wait(timeout=2.0)
            except Exception:
                pass
            if not warned:
                self._log("[ros] 等待 hightorque_controller_node + hightorque_midware_node…")
                warned = True
            if self._stop.wait(timeout=1.0):
                return False
        self._log("[ros][warn] 等待 ROS2 核心超时，仍尝试建桥")
        return True

    def _ros_main_impl(self) -> None:
        _bootstrap_ros_python_path()
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import JointState, Joy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Int32, UInt8
        except ImportError as e:
            self._log(f"[ros] 未找到 rclpy/sensor_msgs: {e}")
            self._log("[ros] 请用 ./start.sh 启动（内部 ros_env.sh 会 source ROS2 Foxy）")
            return

        # 等 DDS + 量产核心（含 midware）再 create_node，避免抢 Participant / 无电机
        if not self._wait_dds_domain_ready():
            raise RuntimeError("CycloneDDS 未就绪（wlan0/UDP）")
        if not self._wait_ros2_stack_ready():
            raise RuntimeError("ROS2 核心未就绪（controller/midware）")

        if not rclpy.ok():
            rclpy.init(args=None)

        self._Joy = Joy
        self._Twist = Twist
        node = Node("ble_command_bridge")
        self._node = node
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._joy_pub = node.create_publisher(Joy, "/joy", qos)
        self._cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", qos)
        try:
            from hightorque_msgs.msg import HightorqueJoy

            self._HightorqueJoy = HightorqueJoy
            self._ht_joy_pub = node.create_publisher(HightorqueJoy, "/hightorque_joy", qos)
        except Exception as e:
            self._log(f"[ros][warn] 无法发布 /hightorque_joy: {e}")
        self._neck.attach(
            node,
            abs_pub=node.create_publisher(JointState, "/pi_plus_absolute", 1),
            clock=node.get_clock(),
        )
        try:
            from hightorque_power.msg import PowerSwitch

            mp_pub = node.create_publisher(PowerSwitch, POWER_TOPIC, 10)
            com_pub = node.create_publisher(UInt8, "/com_power_control", 10)
            self._motor_power.attach_publisher(mp_pub, com_pub)

            def _on_power_state(msg) -> None:
                self._motor_power.update_state_from_hardware(bool(msg.power_switch))

            node.create_subscription(
                PowerSwitch, "/power_switch_state", _on_power_state, 1
            )
        except Exception as e:
            self._log(f"[MP] 电源话题初始化失败（仍继续 /joy）: {e}")

        def _on_battery(msg: UInt8) -> None:
            self._battery_pct = max(0, min(100, int(msg.data)))

        node.create_subscription(UInt8, "/battery_level", _on_battery, 1)

        def _on_joint_states(_msg) -> None:
            names = list(getattr(_msg, "name", []) or [])
            with self._lock:
                self._joint_states_received = bool(names)
                self._joint_state_count = len(names)
                self._joint_state_names = set(names)

        try:
            from sensor_msgs.msg import JointState

            node.create_subscription(JointState, "/joint_states", _on_joint_states, 1)
        except Exception as e:
            self._log(f"[ros][warn] 无法订阅 /joint_states: {e}")

        def _on_imu(_msg) -> None:
            with self._lock:
                self._imu_received = True
                self._last_imu_ts = time.monotonic()

        try:
            from sensor_msgs.msg import Imu

            node.create_subscription(Imu, "/imu", _on_imu, 1)
        except Exception as e:
            self._log(f"[ros][warn] 无法订阅 /imu: {e}")

        self._Int32 = Int32
        self._fsm_pub = node.create_publisher(Int32, "/fsm_state", 1)

        def _on_fsm(msg: Int32) -> None:
            self._set_fsm_local(int(msg.data), reason="", announce=False)

        for topic in FSM_STATE_TOPICS:
            node.create_subscription(Int32, topic, _on_fsm, 1)

        self._init_controller_ifaces(node)
        self._publish_fsm(force=True)
        self._emit_mode_display()

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        self._executor = executor
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        self._log("[ros] 已启用 ROS2 量产控制桥")
        self._log("[ros] 摇杆 X/Y/Z → AMP:/joy；BFM running → /cmd_vel 三轴±0.8(含斜向/自转)")
        self._log("[ros] GAIT → 保持 walk 策略(bfm/amp/amp_lower) + toggle_policy")
        self._log("[ros] 起立 change_state(standing)；失败再发手柄组合")
        self._log("[ros] 起立 LT+RT+START | 坐下 LT+RT+RB | 加速 LT 扳机 | 急停 protect")
        self._ready.set()
        self._log(f"[ros] /joy+/cmd_vel {JOY_PUBLISH_HZ}Hz | 超时 {STICK_TIMEOUT_SEC}s")

        interval = 1.0 / JOY_PUBLISH_HZ
        while not self._stop.is_set() and rclpy.ok():
            self._neck.tick()
            self._motor_power.tick()
            self._tick_joy()
            self._tick_cmd_vel()
            now = time.monotonic()
            if now - self._last_fsm_pub_ts >= FSM_PUBLISH_PERIOD_SEC:
                self._publish_fsm()
            time.sleep(interval)

        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass

    def _init_controller_ifaces(self, node) -> None:
        try:
            from hightorque_msgs.srv import ChangeState, Common, SwitchPolicy

            self._SwitchPolicy = SwitchPolicy
            self._ChangeState = ChangeState
            self._Common = Common
            self._policy_clients = [
                node.create_client(SwitchPolicy, name) for name in SWITCH_POLICY_CANDIDATES
            ]
            self._state_clients = [
                node.create_client(ChangeState, name) for name in CHANGE_STATE_CANDIDATES
            ]
            self._fsm_clients = [
                node.create_client(ChangeState, name) for name in CHANGE_FSM_CANDIDATES
            ]
            self._start_policy_clients = [
                node.create_client(Common, name) for name in START_POLICY_CANDIDATES
            ]
            self._stop_policy_clients = [
                node.create_client(Common, name) for name in STOP_POLICY_CANDIDATES
            ]
            self._log(
                "[ros] 已注册 SwitchPolicy / ChangeState / start_policy / stop_policy "
                "(/hightorque_controller/switch_policy|change_state|change_fsm_state)"
            )
        except Exception as e:
            self._log(f"[ros][warn] 量产服务接口不可用，将只用 /joy 组合键: {e}")
            self._start_policy_clients = []
            self._stop_policy_clients = []
            self._Common = None

        try:
            from hightorque_msgs.msg import ControllerState

            def _on_ctrl(msg) -> None:
                self._current_mode = str(getattr(msg, "current_mode", "") or "")
                self._current_state = str(getattr(msg, "current_state", "") or "")
                self._current_policy = str(getattr(msg, "current_policy", "") or "")
                if _norm_label(self._current_state) == "running":
                    self._policy_on = True
                elif _norm_label(self._current_state) in ("standby", "init", "siting", "sitting"):
                    self._policy_on = False
                mapped = self._fsm_from_controller_mode()
                if mapped is not None:
                    self._set_fsm_local(mapped, reason="controller_state", announce=False)
                self._emit_mode_display()

            for topic in CONTROLLER_STATE_TOPICS:
                node.create_subscription(ControllerState, topic, _on_ctrl, 1)
            self._log("[ros] 已订阅 /hightorque_controller/state")
        except Exception as e:
            self._log(f"[ros][warn] 无法订阅 ControllerState: {e}")

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

    def _current_stick(self) -> Optional[StickCommand]:
        now = time.monotonic()
        with self._lock:
            target = self._stick_target
            age = now - self._last_stick_ts
            if target is None or age > STICK_TIMEOUT_SEC:
                self._stick_filtered = None
                return None
            filtered = self._ema_stick(target, self._stick_filtered)
            self._stick_filtered = filtered
            return filtered

    def _bfm_cmd_vel_mode(self) -> bool:
        """BFM 方向输入仅在 policy=bfm 且 state=running 时生效。"""
        return (
            _norm_label(self._current_policy) == "bfm"
            and _norm_label(self._current_state) == "running"
        )

    @staticmethod
    def _hysteresis_dir(value: float, prev: int) -> int:
        mag = abs(value)
        if prev == 0:
            if mag >= BFM_ACTIVATE_THRESH:
                return 1 if value > 0.0 else -1
            return 0
        if mag < BFM_RELEASE_THRESH:
            return 0
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    def _stick_to_bfm_twist(self, stick: Optional[StickCommand]) -> Tuple[float, float, float]:
        """
        BLE X前+/Y右+/Z右转+ → cmd_vel linear.x前+/y左+/angular.z左转+
        （与 /joy 映射一致：ly=+X, lx=-Y, rx=-Z），再按阈值输出 ±0.8。
        """
        if stick is None:
            self._bfm_dir = (0, 0, 0)
            return 0.0, 0.0, 0.0
        raw_x = float(stick.x)
        raw_y = float(-stick.y)
        raw_z = float(-stick.z)
        dx, dy, dz = self._bfm_dir
        dx = self._hysteresis_dir(raw_x, dx)
        dy = self._hysteresis_dir(raw_y, dy)
        dz = self._hysteresis_dir(raw_z, dz)
        self._bfm_dir = (dx, dy, dz)
        return (
            dx * BFM_CMD_LEVEL,
            dy * BFM_CMD_LEVEL,
            dz * BFM_CMD_LEVEL,
        )

    def _publish_cmd_vel(self, lx: float, ly: float, az: float) -> None:
        if self._cmd_vel_pub is None or self._Twist is None:
            return
        msg = self._Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(az)
        self._cmd_vel_pub.publish(msg)

    def _tick_cmd_vel(self) -> None:
        """BFM running：按 XYZ 独立滞回发离散三轴 /cmd_vel（±0.8），支持斜向与原地转向。"""
        in_bfm = self._bfm_cmd_vel_mode()
        if not in_bfm:
            if self._bfm_cmd_active:
                self._publish_cmd_vel(0.0, 0.0, 0.0)
                self._bfm_dir = (0, 0, 0)
                self._bfm_cmd_active = False
            return

        # 使用 _tick_joy 已更新的滤波摇杆，避免二次 EMA
        stick = self._peek_stick()
        lx, ly, az = self._stick_to_bfm_twist(stick)
        moving = abs(lx) > 1e-6 or abs(ly) > 1e-6 or abs(az) > 1e-6
        if not moving:
            if self._bfm_cmd_active:
                self._publish_cmd_vel(0.0, 0.0, 0.0)
                self._bfm_cmd_active = False
            return
        self._publish_cmd_vel(lx, ly, az)
        self._bfm_cmd_active = True

    def _peek_stick(self) -> Optional[StickCommand]:
        now = time.monotonic()
        with self._lock:
            if self._stick_filtered is None:
                return None
            if now - self._last_stick_ts > STICK_TIMEOUT_SEC:
                return None
            return self._stick_filtered

    def _tick_joy(self) -> None:
        if self._joy_pub is None or self._Joy is None or self._node is None:
            return
        msg = self._Joy()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.axes = [0.0] * JOY_AXES_COUNT
        msg.buttons = [0] * JOY_BUTTONS_COUNT
        msg.axes[AXIS_LT] = TRIGGER_RELEASE
        msg.axes[AXIS_RT] = TRIGGER_RELEASE

        now = time.monotonic()
        with self._lock:
            sprint = self._sprint_enabled
            combo_parts = list(self._combo_parts) if now < self._combo_until else []
            zero_stick = bool(self._combo_zero_stick and combo_parts)
            if now >= self._combo_until:
                self._combo_parts = []
                self._combo_zero_stick = False

        use_bfm_cmd = self._bfm_cmd_vel_mode()
        # BFM：方向只走 /cmd_vel（X/Y/Z→前进/左移/左转），不把摇杆写入 /joy，
        # 避免 joy_mapper 与物理手柄零速互相覆盖，导致只剩前后能用。
        if use_bfm_cmd:
            if not zero_stick:
                self._current_stick()  # 刷新 EMA，供 _tick_cmd_vel 使用
            if not combo_parts:
                return
            stick = None
        else:
            stick = None if zero_stick else self._current_stick()

        if stick is not None:
            # AMP：连续摇杆
            msg.axes[AXIS_LX] = _to_unit(-stick.y, STICK_XY_LIMIT)
            msg.axes[AXIS_LY] = _to_unit(stick.x, STICK_XY_LIMIT)
            msg.axes[AXIS_RX] = _to_unit(-stick.z, STICK_Z_LIMIT)

        if sprint and not use_bfm_cmd:
            msg.axes[AXIS_LT] = TRIGGER_PRESS
        self._apply_combo_parts(msg, combo_parts)

        self._joy_pub.publish(msg)
        if self._ht_joy_pub is not None and self._HightorqueJoy is not None:
            try:
                self._ht_joy_pub.publish(self._joy_to_hightorque(msg))
            except Exception:
                pass

    def _apply_combo_parts(self, msg, parts: Sequence[str]) -> None:
        for part in parts:
            p = part.strip().lower()
            if p == "lt":
                msg.axes[AXIS_LT] = TRIGGER_PRESS
            elif p == "rt":
                msg.axes[AXIS_RT] = TRIGGER_PRESS
            elif p in ("dpl", "left", "←"):
                msg.axes[AXIS_HAT_X] = -1.0
            elif p in ("dpr", "right", "→"):
                msg.axes[AXIS_HAT_X] = 1.0
            elif p in ("dpu", "up", "↑"):
                msg.axes[AXIS_HAT_Y] = -1.0
            elif p in ("dpd", "down", "↓"):
                msg.axes[AXIS_HAT_Y] = 1.0
            elif p in _BTN_MAP:
                msg.buttons[_BTN_MAP[p]] = 1

    def _joy_to_hightorque(self, joy):
        ht = self._HightorqueJoy()
        axes = list(joy.axes)
        buttons = list(joy.buttons)

        def axis(i, default=0.0):
            return float(axes[i]) if i < len(axes) else default

        def btn(i):
            return float(buttons[i]) if i < len(buttons) else 0.0

        ht.l_horizontal = axis(AXIS_LX)
        ht.l_vertical = axis(AXIS_LY)
        ht.lt = axis(AXIS_LT, TRIGGER_RELEASE)
        ht.r_horizontal = axis(AXIS_RX)
        ht.r_vertical = axis(AXIS_RY)
        ht.rt = axis(AXIS_RT, TRIGGER_RELEASE)
        ht.dpad_horizontal = axis(AXIS_HAT_X)
        ht.dpad_vertical = axis(AXIS_HAT_Y)
        ht.a = btn(BTN_A)
        ht.b = btn(BTN_B)
        ht.x = btn(BTN_X)
        ht.y = btn(BTN_Y)
        ht.lb = btn(BTN_LB)
        ht.rb = btn(BTN_RB)
        ht.back = btn(BTN_BACK)
        ht.start = btn(BTN_START)
        ht.center = btn(BTN_GUIDE)
        ht.l = btn(BTN_LS)
        ht.r = btn(BTN_RS)
        return ht

    def _motion_consumer_alive(self) -> bool:
        return self._joy_path_alive() or self._services_alive()

    def _stand_up_worker(self) -> None:
        with self._stand_lock:
            if self._stand_busy:
                self._log("[ros] ST:standing 忽略：上一次尚未完成")
                return
            self._stand_busy = True
        try:
            self._log(f"[ros] ST:standing 开始 | {self._snapshot()}")
            ok = self._stand_up(notify=True)
            if ok:
                self._log(f"[ros] ST:standing 完成 | {self._snapshot()}")
            else:
                self._log(
                    "[ros][warn] ST:standing 失败：未进入 standby。"
                    "若刚执行 ensure_midware，请重启 bfm_real/控制器；"
                    "并确认 MP ON、/joint_states 含 waist_yaw_joint"
                )
        finally:
            with self._stand_lock:
                self._stand_busy = False

    def _joint_states_ready(self, *, wait: bool = False) -> Tuple[bool, str]:
        def ok() -> bool:
            with self._lock:
                if not self._joint_states_received or self._joint_state_count <= 0:
                    return False
                return "waist_yaw_joint" in self._joint_state_names

        if ok():
            return True, ""
        if wait:
            if self._wait_until(ok, JOINT_STATES_WAIT_SEC, "joint_states"):
                return True, ""
        with self._lock:
            n = self._joint_state_count
            has_waist = "waist_yaw_joint" in self._joint_state_names
        if n > 0 and not has_waist:
            return (
                False,
                f"/joint_states 缺 waist_yaw_joint（仅 {n} 关节；midware 未完整或控制器需重启）",
            )
        return (
            False,
            "/joint_states 无数据（电机栈 hightorque_midware_node 未运行或未上电）",
        )

    def _motor_power_hardware_on(self) -> bool:
        return self._motor_power.get_state_wire() == "ON"

    def _prepare_for_standing(self) -> None:
        """退出 running/卡住的 standing，避免行为树一直 Waiting for motor control。"""
        st = _norm_label(self._current_state)
        if st == "running":
            self._log("[ros] 起立前：running → stop_policy")
            self._call_common_policy(False, "stop_policy")
            self._wait_state("standby", timeout=STATE_WAIT_SEC)
            st = _norm_label(self._current_state)
        if st in ("standing", "siting"):
            self._log(f"[ros] 起立前：复位上层 state={st} → init")
            self._call_change_state("init")
            self._wait_state("init", "standby", timeout=8.0)
            time.sleep(0.3)

    def _imu_live(self) -> bool:
        with self._lock:
            if not self._imu_received:
                return False
            return (time.monotonic() - self._last_imu_ts) <= IMU_STALE_SEC

    def _imu_ready(self, *, wait: bool = False) -> Tuple[bool, str]:
        if self._imu_live():
            return True, ""
        if wait:
            ok = self._wait_until(self._imu_live, IMU_WAIT_SEC, "imu")
            if ok:
                return True, ""
        return (
            False,
            "/imu 无数据或已过期（yesense_imu 未就绪，检查 /dev/ttyUSB0 与 bringup 日志）",
        )

    def _stand_up(self, *, notify: bool = True) -> bool:
        try:
            self._neck.release_control(reason="起立前让权")
        except Exception:
            pass
        if not self._ensure_default_bt():
            self._log("[ros][warn] 起立失败：未进入 default_bt（请先 FSM:default 或 M_init）")
            return False
        if not self._motor_power_hardware_on():
            self._log(
                "[ros][warn] 起立失败：/power_switch_state 未确认 ON（请先 MP ON 并等待硬件反馈）"
            )
            return False
        ok_js, js_reason = self._joint_states_ready(wait=True)
        if not ok_js:
            self._log(f"[ros][warn] 起立失败：{js_reason}")
            return False

        self._prepare_for_standing()

        sent = self._call_change_state("standing")
        if sent:
            self._log(f"[ros] change_state(standing) 已发送 | {self._snapshot()}")
            time.sleep(COMBO_HOLD_SEC + 0.2)
        elif self._motion_consumer_alive():
            self._start_combo("lt+rt+start", COMBO_HOLD_SEC, "起立")
            time.sleep(COMBO_HOLD_SEC + 0.2)
        else:
            self._log(
                "[ros][warn] 起立未执行：量产控制器未启动（无 change_state / 无 /hightorque_joy 订阅）"
            )
            return False

        if self._wait_state("standby", timeout=STANDING_WAIT_SEC):
            if notify:
                self._notify_action("auto_stand")
            return True

        self._log(
            f"[ros][warn] 起立超时：服务已响应但未进入 standby | {self._snapshot()}"
        )
        if self._motion_consumer_alive():
            self._start_combo("lt+rt+start", COMBO_HOLD_SEC, "起立回退")
            time.sleep(COMBO_HOLD_SEC + 0.2)
            if self._wait_state("standby", timeout=STANDING_WAIT_SEC):
                if notify:
                    self._notify_action("auto_stand")
                return True
        return False

    def _request_stand(self, *, notify: bool = False) -> bool:
        return self._stand_up(notify=notify)

    def _request_sit(self) -> bool:
        """蹲下。BFM/AMP running 时控制器拒绝直接 siting，须先回 standby。"""
        with self._gait_lock:
            if self._gait_busy:
                self._log("[ros] 忽略坐下：步态/坐下切换仍在进行")
                return False
            self._gait_busy = True
        threading.Thread(
            target=self._sit_worker,
            daemon=True,
            name="ble-sit",
        ).start()
        return True

    def _sit_worker(self) -> None:
        try:
            self._sit_worker_impl()
        finally:
            with self._gait_lock:
                self._gait_busy = False

    def _ensure_standby_for_sit(self) -> bool:
        """running 不可直接蹲：停速 → toggle → standby，并回执 GAIT OFF。"""
        st = _norm_label(self._current_state)
        if st in ("standby", "siting", "sitting"):
            return True
        if st == "running":
            pol = _norm_label(self._current_policy)
            self._log(
                f"[ros] 坐下前：running → standby"
                f"（{pol or 'walk'} 不可直接蹲）| {self._snapshot()}"
            )
            with self._lock:
                self._stick_target = None
                self._stick_filtered = None
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            self._bfm_dir = (0, 0, 0)
            self._bfm_cmd_active = False
            self._toggle_upper("坐下前 → STANDBY")
            if not self._wait_state("standby", timeout=STATE_WAIT_SEC):
                self._log(f"[ros][warn] 坐下前未进入 standby | {self._snapshot()}")
                return False
            self._policy_on = False
            self._notify_gait("OFF")
            return True
        if st == "standing":
            self._log(f"[ros] 坐下前等待 standing→standby | {self._snapshot()}")
            return self._wait_state("standby", timeout=STANDING_WAIT_SEC)
        self._log(f"[ros][warn] 当前 state={st}，无法蹲下 | {self._snapshot()}")
        return False

    def _sit_worker_impl(self) -> None:
        try:
            self._neck.release_control(reason="坐下前让权")
        except Exception:
            pass
        if not self._ensure_standby_for_sit():
            self._log("[ros][warn] 坐下中止：未就绪 standby")
            return
        if self._call_change_state_any(("siting", "sitting", "sit")):
            if self._wait_state("siting", "sitting", timeout=STATE_WAIT_SEC):
                self._log(f"[ros] 坐下完成 | {self._snapshot()}")
            else:
                self._log(f"[ros] 已发 siting（状态回读超时）| {self._snapshot()}")
            self._notify_action("squat")
            return
        if self._motion_consumer_alive():
            self._start_combo("lt+rt+rb", COMBO_HOLD_SEC, "坐下")
            self._notify_action("squat")
            return
        self._log("[ros][warn] 坐下未执行：量产控制器未启动")

    def _trigger_upper_state(self, cmd_key: str) -> None:
        key = _norm_label(cmd_key)
        wire = f"ST:{key}"
        if key == "standing":
            threading.Thread(
                target=self._stand_up_worker,
                daemon=True,
                name="ble-stand",
            ).start()
            return
        if key in ("sit", "siting", "sitting"):
            self._request_sit()
            return
        if key in ("toggle", "toggle_policy"):
            self._toggle_upper(wire)
            return
        if key == "start":
            self._call_common_policy(True, "start_policy")
            return
        if key == "stop":
            self._call_common_policy(False, "stop_policy")
            return
        self._log(f"[ros] 未知上层状态指令: {wire}")

    def _trigger_policy(self, policy_name: str) -> None:
        """POL:{name} → /hightorque_controller/switch_policy。"""
        name = _canonical_policy(policy_name)
        wire = f"POL:{policy_name}"
        if not name:
            self._log(f"[ros] 拒绝未知策略: {wire}")
            return
        wire = f"POL:{name}"
        if _norm_label(self._current_policy) == _norm_label(name):
            self._log(f"[ros] 已是策略 {name} | {self._snapshot()}")
            if name == "bfm":
                self._ensure_bfm_running(wire)
            return

        self._log(f"[ros] {wire} 开始 | {self._snapshot()}")
        if not self._ensure_default_bt():
            self._log("[ros][warn] 未进入 default_bt，策略切换可能失败")

        st = _norm_label(self._current_state)
        if st == "running":
            self._toggle_upper(f"{wire} 前切 STANDBY")
            self._wait_state("standby", timeout=STATE_WAIT_SEC)

        if self._call_switch_policy(name) and self._wait_policy(
            name, timeout=POLICY_WAIT_SEC
        ):
            self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            if name == "bfm":
                self._ensure_bfm_running(wire)
            return
        self._log(f"[ros][warn] {wire} 未确认 current_policy | {self._snapshot()}")

    def _ensure_bfm_running(self, reason: str) -> None:
        """BFM 方向输入要求 state=running；切策略后自动 toggle 进入。"""
        if _norm_label(self._current_policy) != "bfm":
            return
        if _norm_label(self._current_state) == "running":
            self._policy_on = True
            self._notify_gait("ON")
            self._log(f"[ros] BFM 已在 RUNNING，摇杆走 /cmd_vel | {self._snapshot()}")
            return
        st = _norm_label(self._current_state)
        if st not in ("standby", "init", ""):
            self._log(
                f"[ros][warn] BFM 当前 state={st}，请先 ST:standing 到 standby 再 GAIT ON"
            )
            return
        self._toggle_upper(f"{reason} → RUNNING")
        if self._wait_state("running", timeout=STATE_WAIT_SEC):
            self._policy_on = True
            self._notify_gait("ON")
            self._log(f"[ros] BFM 已进 RUNNING，推摇杆即可行走 | {self._snapshot()}")
        else:
            self._log(f"[ros][warn] BFM 未进 RUNNING，请发 GAIT ON | {self._snapshot()}")

    def _call_common_policy(self, enable: bool, label: str) -> bool:
        clients = self._start_policy_clients if enable else self._stop_policy_clients
        if self._Common is None or not clients:
            self._log(f"[ros][warn] {label} 服务不可用")
            return False
        req = self._Common.Request()
        req.enable = bool(enable)
        req.str = ""
        return self._try_clients(clients, req, label)

    def _start_combo(
        self,
        combo: str,
        duration: float,
        reason: str,
        zero_stick: bool = True,
    ) -> None:
        parts = [p for p in _norm_token(combo).split("+") if p]
        if not parts:
            return
        with self._lock:
            self._combo_parts = parts
            self._combo_until = time.monotonic() + duration
            self._combo_zero_stick = zero_stick
        self._log(f"[ros] {reason} → /joy {combo} {duration:.2f}s")

    def _notify_gait(self, state: str) -> None:
        if self._gait_listener is None:
            return
        try:
            self._gait_listener(state)
        except Exception:
            pass

    def _notify_action(self, action: str) -> None:
        if self._action_listener is None:
            return
        try:
            self._action_listener(action)
        except Exception:
            pass

    def _notify_mode(self, mode_key: str, *, voice: bool = False) -> None:
        """通知 UI/语音。默认不播报（被动同步）；显式 M_* 指令传 voice=True。"""
        if self._mode_listener is None:
            return
        try:
            self._mode_listener(mode_key, voice=voice)
        except TypeError:
            try:
                self._mode_listener(mode_key)
            except Exception:
                pass
        except Exception:
            pass

    def _snapshot(self) -> str:
        fsm = self._fsm_value
        fsm_name = FSM_STATE_NAMES.get(fsm, str(fsm)) if fsm is not None else "?"
        return (
            f"mode={self._current_mode or '-'} "
            f"state={self._current_state or '-'} "
            f"policy={self._current_policy or '-'} "
            f"fsm={fsm_name}"
        )

    def _wait_until(
        self,
        pred: Callable[[], bool],
        timeout: float,
        label: str,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop.is_set():
            if pred():
                return True
            time.sleep(0.1)
        ok = pred()
        if not ok:
            self._log(f"[ros][warn] 等待 {label} 超时（{timeout:.0f}s）| {self._snapshot()}")
        return ok

    def _wait_mode(self, target: str, timeout: float = STATE_WAIT_SEC) -> bool:
        want = _norm_label(target)
        return self._wait_until(
            lambda: _norm_label(self._current_mode) == want, timeout, f"mode={want}"
        )

    def _wait_state(self, *targets: str, timeout: float = STATE_WAIT_SEC) -> bool:
        wants = {_norm_label(t) for t in targets}
        return self._wait_until(
            lambda: _norm_label(self._current_state) in wants,
            timeout,
            f"state in {sorted(wants)}",
        )

    def _wait_policy(self, target: str, timeout: float = POLICY_WAIT_SEC) -> bool:
        want = _norm_label(target)
        return self._wait_until(
            lambda: _norm_label(self._current_policy) == want, timeout, f"policy={want}"
        )

    def _wait_fsm_value(self, *values: int, timeout: float = STATE_WAIT_SEC) -> bool:
        wants = set(values)
        return self._wait_until(
            lambda: self._fsm_value in wants, timeout, f"fsm in {sorted(wants)}"
        )

    def _publish_fsm(self, force: bool = False) -> None:
        if self._fsm_pub is None or self._Int32 is None:
            return
        if self._services_alive():
            return
        msg = self._Int32()
        msg.data = int(self.get_fsm_value())
        self._fsm_pub.publish(msg)
        self._last_fsm_pub_ts = time.monotonic()

    def _set_fsm_local(self, value: int, reason: str = "", announce: bool = True) -> None:
        prev = self._fsm_value
        self._fsm_value = int(value)
        self._publish_fsm(force=True)
        if announce and prev != self._fsm_value:
            name = FSM_STATE_NAMES.get(self._fsm_value, str(self._fsm_value))
            extra = f" {reason}" if reason else ""
            self._log(f"[ros] FSM → {self._fsm_value} ({name}){extra}")
        self._emit_mode_display()

    def _fsm_from_controller_mode(self) -> Optional[int]:
        mode = _norm_label(self._current_mode)
        if mode == "protect":
            return FSM_PROTECT
        if mode == "reset_zero":
            return FSM_EXEC_RESET_ZERO
        if mode == "init":
            return FSM_INIT
        if mode == "default_bt":
            # 仅保留「候选菜单」本地态；INIT/ERROR 残留必须纠正为 EXEC_DEFAULT，
            # 否则 get_mode_wire 会把已成功的 default 显示成 M_init（模式切换假失效）
            if self._fsm_value in (FSM_CANDIDATE_DEFAULT, FSM_CANDIDATE_RESET_ZERO):
                return None
            return FSM_EXEC_DEFAULT
        if mode == "none":
            return None
        return None

    def _emit_mode_display(self) -> None:
        """同步小程序 mode 显示；被动状态变化不播报。"""
        wire = self.get_mode_wire()
        if wire == self._last_mode_wire:
            return
        self._last_mode_wire = wire
        self._notify_mode(wire.lower(), voice=False)

    def _in_fsm_menu(self) -> bool:
        if self.get_fsm_value() in FSM_MENU_STATES:
            return True
        return _norm_label(self._current_mode) in ("init", "none")

    def _next_candidate(self) -> int:
        if self.get_fsm_value() == FSM_CANDIDATE_DEFAULT:
            return FSM_CANDIDATE_RESET_ZERO
        return FSM_CANDIDATE_DEFAULT

    def _prev_candidate(self) -> int:
        if self.get_fsm_value() == FSM_CANDIDATE_RESET_ZERO:
            return FSM_CANDIDATE_DEFAULT
        return FSM_CANDIDATE_RESET_ZERO

    def _handle_fsm_nav(self, combo: str) -> bool:
        """小程序模式菜单：center / start+B / LT+RT+方向 / LT+RT+A。"""
        if combo == "center":
            if self._call_fsm("default"):
                self._set_fsm_local(FSM_EXEC_DEFAULT, "center → default")
            else:
                self._start_combo("center", COMBO_PULSE_SEC, "回 DEFAULT", zero_stick=True)
                self._set_fsm_local(FSM_EXEC_DEFAULT, "center 手柄兜底")
            return True
        if combo == "start+b":
            if self._call_fsm("init"):
                self._set_fsm_local(FSM_INIT, "start+B → init")
            else:
                self._log("[ros] start+B：FSM 服务不可用")
                self._set_fsm_local(FSM_INIT, "start+B 本地")
            return True
        if combo == "lt+rt+dpl":
            self._log("[ros] 忽略 LT+RT+DPL：ROS2 已删除 TEACH")
            return True
        if combo in ("lt+rt+dpr", "lt+rt+dpu", "lt+rt+a") and not self._in_fsm_menu():
            return False
        if combo == "lt+rt+dpr":
            nxt = self._next_candidate()
            if self._call_fsm("next"):
                self._set_fsm_local(nxt, "next 候选")
            else:
                self._start_combo("lt+rt+dpr", COMBO_PULSE_SEC, "FSM next", zero_stick=True)
                self._set_fsm_local(nxt, "next 手柄兜底")
            return True
        if combo == "lt+rt+dpu":
            nxt = self._prev_candidate()
            if self._call_fsm("prev"):
                self._set_fsm_local(nxt, "prev 候选")
            else:
                self._start_combo("lt+rt+dpu", COMBO_PULSE_SEC, "FSM prev", zero_stick=True)
                self._set_fsm_local(nxt, "prev 手柄兜底")
            return True
        if combo == "lt+rt+a":
            cur = self.get_fsm_value()
            if self._call_fsm("confirm"):
                if cur == FSM_CANDIDATE_RESET_ZERO:
                    self._set_fsm_local(FSM_EXEC_RESET_ZERO, "confirm 调零")
                else:
                    self._set_fsm_local(FSM_EXEC_DEFAULT, "confirm DEFAULT")
            else:
                self._start_combo("lt+rt+a", COMBO_PULSE_SEC, "FSM confirm", zero_stick=True)
            return True
        return False

    def _trigger_fsm(self, cmd_key: str) -> None:
        threading.Thread(
            target=self._fsm_worker,
            args=(cmd_key,),
            daemon=True,
            name="ble-fsm-cmd",
        ).start()

    def _fsm_worker(self, cmd_key: str) -> None:
        if not self.ready:
            self._log("[ros] 桥接未就绪（等待 ROS2），忽略 FSM 指令")
            return
        key = _norm_label(cmd_key)
        wire = f"FSM:{key}"
        self._log(f"[ros] {wire} 开始 | {self._snapshot()}")

        if key == "default":
            if _norm_label(self._current_mode) == "default_bt" or self._fsm_value == FSM_EXEC_DEFAULT:
                self._log(f"[ros] {wire} 已在 default_bt/FSM=5")
            elif self._call_fsm("default"):
                self._wait_mode("default_bt", timeout=STATE_WAIT_SEC) or self._wait_fsm_value(
                    5, timeout=3.0
                )
                self._set_fsm_local(FSM_EXEC_DEFAULT, wire)
            else:
                self._start_combo("center", COMBO_PULSE_SEC, wire, zero_stick=True)
                self._set_fsm_local(FSM_EXEC_DEFAULT, f"{wire} 手柄兜底")
            self._notify_mode("m_default")
            self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            return

        if key == "init":
            if self._call_fsm("init"):
                self._wait_mode("init", timeout=3.0) or self._wait_fsm_value(0, timeout=2.0)
                self._set_fsm_local(FSM_INIT, wire)
                self._notify_mode("m_init")
                self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            else:
                self._log(f"[ros][warn] {wire} 失败：change_fsm_state 服务不可用")
            return

        if key == "protect":
            if _norm_label(self._current_mode) == "protect" or self._fsm_value == FSM_PROTECT:
                self._log(f"[ros] {wire} 已在 PROTECT")
                return
            if self._call_fsm("protect"):
                self._wait_mode("protect", timeout=3.0) or self._wait_fsm_value(8, timeout=2.0)
                self._set_fsm_local(FSM_PROTECT, wire)
                with self._lock:
                    self._stick_target = None
                    self._stick_filtered = None
                    self._sprint_enabled = False
                self._policy_on = False
                self._notify_mode("m_protect")
                self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            else:
                with self._lock:
                    self._stick_target = None
                    self._stick_filtered = None
                    self._sprint_enabled = False
                self._policy_on = False
                self._start_combo("lt+rt+b", ESTOP_HOLD_SEC, wire)
                self._log(f"[ros][warn] {wire} 服务不可用，已发手柄兜底")
            return

        if key == "next":
            if not self._in_fsm_menu():
                self._log(f"[ros][warn] {wire} 忽略：当前不在 FSM 候选菜单")
                return
            nxt = self._next_candidate()
            if self._call_fsm("next"):
                self._set_fsm_local(nxt, wire)
            else:
                self._start_combo("lt+rt+dpr", COMBO_PULSE_SEC, wire, zero_stick=True)
                self._set_fsm_local(nxt, f"{wire} 手柄兜底")
            self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            return

        if key == "prev":
            if not self._in_fsm_menu():
                self._log(f"[ros][warn] {wire} 忽略：当前不在 FSM 候选菜单")
                return
            nxt = self._prev_candidate()
            if self._call_fsm("prev"):
                self._set_fsm_local(nxt, wire)
            else:
                self._start_combo("lt+rt+dpu", COMBO_PULSE_SEC, wire, zero_stick=True)
                self._set_fsm_local(nxt, f"{wire} 手柄兜底")
            self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            return

        if key == "confirm":
            cur = self.get_fsm_value()
            if self._call_fsm("confirm"):
                if cur == FSM_CANDIDATE_RESET_ZERO:
                    self._set_fsm_local(FSM_EXEC_RESET_ZERO, wire)
                else:
                    self._set_fsm_local(FSM_EXEC_DEFAULT, wire)
            else:
                self._start_combo("lt+rt+a", COMBO_PULSE_SEC, wire, zero_stick=True)
            self._log(f"[ros] {wire} 完成 | {self._snapshot()}")
            return

        self._log(f"[ros] 未知 FSM 指令: {wire}")

    def _trigger_mode(self, mode_key: str) -> None:
        label = MODE_LABELS.get(mode_key, mode_key)
        # 仅小程序/指令下发的 M_* 播报模式音
        self._notify_mode(mode_key, voice=True)
        threading.Thread(
            target=self._mode_worker,
            args=(mode_key, label),
            daemon=True,
            name="ble-fsm-mode",
        ).start()

    def _mode_worker(self, mode_key: str, label: str) -> None:
        if mode_key == "m_protect":
            if _norm_label(self._current_mode) == "protect" or self._fsm_value == 8:
                self._log(f"[ros] 已在 PROTECT，忽略重复 {label}")
                return
            if self._call_fsm("protect"):
                self._wait_mode("protect", timeout=3.0) or self._wait_fsm_value(8, timeout=2.0)
                self._set_fsm_local(FSM_PROTECT, "protect")
                with self._lock:
                    self._stick_target = None
                    self._stick_filtered = None
                    self._sprint_enabled = False
                self._policy_on = False
                return
            with self._lock:
                self._stick_target = None
                self._stick_filtered = None
                self._sprint_enabled = False
            self._policy_on = False
            self._start_combo("lt+rt+b", ESTOP_HOLD_SEC, "保护模式/急停")
            return
        if mode_key == "m_init":
            if self._call_fsm("init"):
                self._wait_mode("init", timeout=3.0) or self._wait_fsm_value(0, timeout=2.0)
                self._set_fsm_local(FSM_INIT, "M_init")
                return
            self._log("[ros] M_init：FSM 服务不可用，不连按 LT+RT+B（避免误进保护）")
            return
        if mode_key == "m_resetzero":
            if self._enter_reset_zero():
                return
            self._log("[ros] M_resetzero：FSM 服务不可用，已忽略手柄循环切换")
            return
        self._log(f"[ros] 忽略模式 {mode_key}（{label}）")

    def _enter_reset_zero(self) -> bool:
        """文档：init 重置候选 → prev → confirm，进入 EXEC_RESET_ZERO。"""
        if not self._fsm_clients or self._ChangeState is None:
            return False
        if not self._call_fsm("init"):
            return False
        self._set_fsm_local(FSM_INIT, "resetzero init")
        self._wait_fsm_value(0, timeout=3.0)
        if not self._call_fsm("prev"):
            return False
        self._set_fsm_local(FSM_CANDIDATE_RESET_ZERO, "resetzero prev")
        time.sleep(0.25)
        if not self._call_fsm("confirm"):
            return False
        self._set_fsm_local(FSM_EXEC_RESET_ZERO, "resetzero confirm")
        self._wait_fsm_value(10, 11, 12, timeout=6.0)
        return True

    def _trigger_action(self, action_key: str) -> None:
        combo = action_key
        if parse_gamepad_combo is not None:
            parsed = parse_gamepad_combo(action_key)
            if parsed is not None:
                combo = parsed
        combo = _norm_token(combo)
        if not combo:
            return
        if combo in BLOCKED_POLICY_COMBOS:
            self._log(f"[ros] 已禁用 BFM/amp_lower 组合 {combo}，保持 AMP")
            return
        if self._handle_fsm_nav(combo):
            return

        if combo == "lt+rt+start":
            self._request_stand()
            return
        if combo == "lt+rt+rb":
            self._request_sit()
            return
        if combo == "lt+rt+b":
            with self._lock:
                self._stick_target = None
                self._stick_filtered = None
                self._sprint_enabled = False
            self._policy_on = False
            if not self._call_fsm("protect"):
                self._start_combo(combo, ESTOP_HOLD_SEC, "卸力/急停")
            else:
                self._set_fsm_local(FSM_PROTECT, "LT+RT+B")
            return
        if combo == "lt+rt+lb":
            self._trigger_gait("OFF" if self._policy_on else "ON")
            return

        label = label_for_combo(combo)
        duration = COMBO_HOLD_SEC if _combo_is_hold(combo) else COMBO_PULSE_SEC
        self._start_combo(combo, duration, f"动作 {label}", zero_stick=False)
        self._notify_action(combo)

    def _set_sprint(self, enabled: bool, log: bool = True) -> None:
        with self._lock:
            prev = self._sprint_enabled
            self._sprint_enabled = enabled
        if log:
            if enabled == prev:
                self._log(f"[ros] 加速已{'开启' if enabled else '关闭'}（重复指令）")
            else:
                self._log(
                    f"[ros] 加速{'开启' if enabled else '关闭'} → "
                    f"/joy LT扳机(axes[2])={'按下' if enabled else '松开'}"
                )
                if self._sprint_listener is not None:
                    try:
                        self._sprint_listener(enabled)
                    except Exception:
                        pass

    def _trigger_gait(self, state: str) -> None:
        if state not in ("ON", "OFF"):
            return
        with self._gait_lock:
            if self._gait_busy:
                self._log(f"[ros] 忽略 GAIT {state}：上一次切换仍在进行")
                return
            self._gait_busy = True
        threading.Thread(
            target=self._gait_worker,
            args=(state == "ON",),
            daemon=True,
            name="ble-gait-amp",
        ).start()

    def _gait_worker(self, want_on: bool) -> None:
        try:
            self._gait_worker_impl(want_on)
        finally:
            with self._gait_lock:
                self._gait_busy = False

    def _gait_worker_impl(self, want_on: bool) -> None:
        """
        量产顺序：
          default_bt + STANDBY
            -> 保持/切到 walk 策略（已是 bfm/amp/amp_lower 则不强制改 amp）
            -> toggle_policy 进入 RUNNING（GAIT ON）
        RUNNING 中先 toggle 回 STANDBY。
        """
        # 脖子占头电机时全身策略无法接管 → 先释放，避免新功能挡旧行走
        try:
            self._neck.release_control(reason="GAIT 前让权")
        except Exception:
            pass
        self._log(f"[ros] GAIT {'ON' if want_on else 'OFF'} 开始 | {self._snapshot()}")
        if want_on:
            ok_imu, imu_reason = self._imu_ready(wait=True)
            if not ok_imu:
                # 等待期间若已是 RUNNING（例如手柄已切步态），仍回执 ON，避免误报拒绝
                st_now = _norm_label(self._current_state)
                pol_now = _norm_label(self._current_policy)
                if st_now == "running" and pol_now in WALK_POLICIES | {""}:
                    self._policy_on = True
                    self._notify_gait("ON")
                    self._log(
                        f"[ros][warn] /imu 未就绪但仍保持 RUNNING | {imu_reason} | {self._snapshot()}"
                    )
                    return
                self._log(f"[ros][warn] GAIT ON 拒绝：{imu_reason}")
                return
        if not self._ensure_default_bt():
            self._log("[ros][warn] 未进入 EXEC_DEFAULT/default_bt，步态可能无效")

        st = _norm_label(self._current_state)
        if want_on and st in ("", "init"):
            self._stand_up(notify=False)
            st = _norm_label(self._current_state)

        if not self._ensure_walk_policy(prefer_standby=True):
            self._log("[ros][warn] walk 策略未确认，仍尝试步态切换")

        st = _norm_label(self._current_state)
        pol = _norm_label(self._current_policy)
        if want_on:
            if st == "running" and pol in WALK_POLICIES | {""}:
                self._policy_on = True
                self._notify_gait("ON")
                self._log(f"[ros] GAIT ON 已在 RUNNING | {self._snapshot()}")
                return
            if st != "running":
                self._toggle_upper("GAIT ON → RUNNING")
                if self._wait_state("running", timeout=STATE_WAIT_SEC):
                    self._policy_on = True
                    self._notify_gait("ON")
                    self._log(f"[ros] GAIT ON 完成 | {self._snapshot()}")
                    return
            self._policy_on = True
            self._notify_gait("ON")
            self._log(f"[ros][warn] GAIT ON 未等到 RUNNING，已按请求回执 | {self._snapshot()}")
            return

        with self._lock:
            self._stick_target = None
            self._stick_filtered = None
        self._publish_cmd_vel(0.0, 0.0, 0.0)
        self._bfm_dir = (0, 0, 0)
        self._bfm_cmd_active = False
        if st == "running":
            self._toggle_upper("GAIT OFF → STANDBY")
            self._wait_state("standby", timeout=STATE_WAIT_SEC)
        self._policy_on = False
        self._notify_gait("OFF")
        self._log(f"[ros] GAIT OFF 完成 | {self._snapshot()}")

    def _ensure_default_bt(self) -> bool:
        if _norm_label(self._current_mode) == "default_bt" or self._fsm_value == 5:
            return True
        if _norm_label(self._current_mode) == "protect" or self._fsm_value == 8:
            self._log("[ros] 当前 PROTECT，不自动进 DEFAULT（请先 M_init 或解除保护）")
            return False
        if self._call_fsm("default"):
            return self._wait_mode("default_bt", timeout=STATE_WAIT_SEC) or self._wait_fsm_value(
                5, timeout=2.0
            )
        return False

    def _toggle_upper(self, reason: str) -> None:
        if self._call_change_state_any(("toggle_policy", "toggle")):
            time.sleep(0.2)
            return
        self._start_combo("lt+rt+lb", COMBO_HOLD_SEC, reason)
        time.sleep(COMBO_HOLD_SEC + 0.25)

    def _ensure_walk_policy(self, prefer_standby: bool = True) -> bool:
        """已是 bfm/amp/amp_lower 则保留；否则默认切到 amp。"""
        pol = _norm_label(self._current_policy)
        if pol in WALK_POLICIES:
            return True
        return self._switch_to_amp(prefer_standby=prefer_standby)

    def _switch_to_amp(self, prefer_standby: bool = True) -> bool:
        if _norm_label(self._current_policy) == AMP_POLICY_NAME:
            return True
        st = _norm_label(self._current_state)
        if prefer_standby and st == "running":
            self._toggle_upper("切 STANDBY 再换 AMP")
            self._wait_state("standby", timeout=STATE_WAIT_SEC)
        if self._call_switch_policy(AMP_POLICY_NAME):
            if self._wait_policy(AMP_POLICY_NAME, timeout=POLICY_WAIT_SEC):
                return True
        # 文档当前键位 LT+LS；建议键位 LT+RT+LS
        self._start_combo("lt+ls", AMP_SWITCH_SEC, "AMP 手柄回退 LT+LS", zero_stick=False)
        time.sleep(AMP_SWITCH_SEC + 0.3)
        if self._wait_policy(AMP_POLICY_NAME, timeout=3.0):
            return True
        self._start_combo("lt+rt+ls", AMP_SWITCH_SEC, "AMP 手柄回退 LT+RT+LS", zero_stick=False)
        time.sleep(AMP_SWITCH_SEC + 0.3)
        return self._wait_policy(AMP_POLICY_NAME, timeout=3.0)

    def _call_switch_policy(self, policy_name: str) -> bool:
        name = _canonical_policy(policy_name)
        if not name:
            self._log(f"[ros] 拒绝未知/禁用策略: {policy_name}")
            return False
        if self._SwitchPolicy is None or not self._policy_clients:
            return False
        req = self._SwitchPolicy.Request()
        req.policy_name = name
        return self._try_clients(self._policy_clients, req, f"SwitchPolicy({name})")

    def _call_change_state_any(self, commands: Sequence[str]) -> bool:
        for cmd in commands:
            if self._call_change_state(cmd):
                return True
        return False

    def _call_change_state(self, command: str) -> bool:
        if self._ChangeState is None or not self._state_clients:
            return False
        req = self._ChangeState.Request()
        req.states = [command]
        return self._try_clients(self._state_clients, req, f"change_state({command})")

    def _call_fsm(self, command: str) -> bool:
        if self._ChangeState is None or not self._fsm_clients:
            return False
        req = self._ChangeState.Request()
        req.states = [command]
        return self._try_clients(self._fsm_clients, req, f"change_fsm_state({command})")

    def _try_clients(self, clients, req, label: str) -> bool:
        for client in clients:
            name = getattr(client, "srv_name", "")
            try:
                if not client.service_is_ready():
                    continue
                future = client.call_async(req)
                deadline = time.monotonic() + SERVICE_CALL_SEC
                while time.monotonic() < deadline:
                    if future.done():
                        break
                    time.sleep(0.05)
                if not future.done():
                    self._log(f"[ros][warn] {label} 超时: {name}")
                    continue
                resp = future.result()
                ok = bool(
                    getattr(resp, "success", None)
                    if hasattr(resp, "success")
                    else getattr(resp, "result", False)
                )
                msg = getattr(resp, "message", "") or ""
                extra = getattr(resp, "current_policy", "") or ""
                if ok:
                    self._log(f"[ros] {label} 服务已响应 @ {name} {msg} {extra}".strip())
                    return True
                self._log(f"[ros][warn] {label} 失败 @ {name}: {msg}")
            except Exception as e:
                self._log(f"[ros][warn] {label} 异常 @ {name}: {e}")
        return False
