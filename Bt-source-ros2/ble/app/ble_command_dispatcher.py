#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FFE1 指令分类与分发：摇杆直通保活，模式/动作入队。"""

from __future__ import annotations

import os
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Optional, Tuple

# MTU 247 下可承载 WIFI 凭据；摇杆等短指令不受影响
MAX_PACKET_BYTES = 200
QUEUE_MAX = 32
TICK_INTERVAL_SEC = 0.05
STICK_XY_LIMIT = 1.8
STICK_Z_LIMIT = 1.5
# payload 内 SSID/密码分隔（不出现在明文日志 wire 中）
try:
    from ble_wifi_manager import WIFI_PAYLOAD_SEP
except ImportError:
    WIFI_PAYLOAD_SEP = "\x1e"

# 摇杆：XYZ 必填；可选 ,N:序号 供小程序 20Hz 保活（板端忽略 N 仅解析 XYZ）
STICK_RE = re.compile(
    r"^X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)"
    r"(?:\s*,\s*N:\s*\d+)?\s*$",
    re.IGNORECASE,
)
MODE_RE = re.compile(r"^M_(init|protect|resetzero|tech)$", re.IGNORECASE)
ST_RE = re.compile(r"^ST:(standing|sit|toggle|start|stop)$", re.IGNORECASE)
FSM_RE = re.compile(
    r"^FSM:(default|next|prev|confirm|init|protect)$", re.IGNORECASE
)
# 步态/策略：POL:bfm | POL:amp | POL:amp_lower | POL:byd_power | POL:SP8 …
POL_RE = re.compile(r"^POL:([A-Za-z][A-Za-z0-9_]{0,63})$")
HANDSHAKE_RE = re.compile(r"^M_default$", re.IGNORECASE)
HEARTBEAT_RE = re.compile(r"^1$", re.IGNORECASE)

ACTION_WIRE = {
    "lt+rt+start": "LT+RT+start",
    "lt+rt+rb": "LT+RT+RB",
    "lt+rt+b": "LT+RT+B",
    "lt+rt+a": "LT+RT+A",
    "center": "center",
    "start+b": "start+B",
    "rt+a": "RT+A",
    "rt+x": "RT+X",
    "a": "A",
    "x": "X",
    "lt+rt+dpu": "LT+RT+DPU",
    "lt+rt+dpr": "LT+RT+DPR",
    "lt+rt+dpl": "LT+RT+DPL",
    "lt+dpr": "LT+DPR",
    "lt+dpd": "LT+DPD",
    "lt+dpl": "LT+DPL",
}
ACTION_RE = re.compile(
    r"^(LT\+RT\+start|LT\+RT\+RB|LT\+RT\+B|LT\+RT\+A|center|start\+B|"
    r"RT\+A|RT\+X|"
    r"LT\+RT\+DPU|LT\+RT\+DPR|LT\+RT\+DPL|LT\+DPR|LT\+DPD|LT\+DPL|"
    r"A|X)$",
    re.IGNORECASE,
)
# 短脉冲自定义动作防连发；FSM 菜单键必须排除，否则 8s 冷却会挡住模式切换
PULSE_ACTION_COOLDOWN_SEC = 8.0
FSM_NAV_KEYS = frozenset({
    "center", "start+b", "lt+rt+a", "lt+rt+dpu", "lt+rt+dpr", "lt+rt+dpl",
})
PULSE_ACTION_KEYS = frozenset({
    "rt+a", "rt+x",
    "a", "x",
    "lt+dpr", "lt+dpd", "lt+dpl",
})
NECK_OFFSET_RE = re.compile(r"^[Pp]([+-]?\d+)[Yy]([+-]?\d+)$")
NECK_CENTER_RE = re.compile(r"^neck0$", re.IGNORECASE)
LOCATE_FACE_RE = re.compile(r"^locate_face\s+(ON|OFF)$", re.IGNORECASE)
MP_RE = re.compile(r"^MP\s+(ON|OFF)$", re.IGNORECASE)
GAIT_RE = re.compile(r"^GAIT\s+(ON|OFF)$", re.IGNORECASE)
SPRINT_RE = re.compile(r"^LT\s+(ON|OFF)$", re.IGNORECASE)
SOUND_RE = re.compile(r"^sound\s+(ON|OFF)(?:\s+(\d+))?$", re.IGNORECASE)
VOLUME_RE = re.compile(r"^V\s+(\d{1,3})$", re.IGNORECASE)
PULL_RE = re.compile(r"^PULL\s+(ON|OFF)$", re.IGNORECASE)
CONV_CODE_RE = re.compile(r"^[A-Z][A-Z0-9]{1,31}$")

try:
    from ble_device_name import RENAME_RE as _RENAME_RE
except ImportError:
    _RENAME_RE = re.compile(r"^rename\s+HT_(\d{8})$", re.IGNORECASE)

try:
    from ble_gamepad import combo_to_wire, parse_gamepad_combo
except ImportError:
    combo_to_wire = None  # type: ignore
    parse_gamepad_combo = None  # type: ignore


class CommandKind(str, Enum):
    HEARTBEAT = "heartbeat"
    HANDSHAKE = "handshake"
    STICK = "stick"
    MODE = "mode"
    UPPER_STATE = "upper_state"
    FSM = "fsm"
    POLICY = "policy"
    ACTION = "action"
    NECK = "neck"
    LOCATE_FACE = "locate_face"
    MOTOR_POWER = "motor_power"
    GAIT = "gait"
    SPRINT = "sprint"
    SOUND = "sound"
    VOLUME = "volume"
    RENAME = "rename"
    WIFI = "wifi"
    PULL = "pull"
    CONVERSATION = "conversation"
    UNKNOWN = "unknown"


@dataclass
class QueuedCommand:
    kind: CommandKind
    wire: str
    payload: str


LogFn = Callable[[str], None]
HandleFn = Callable[[CommandKind, str], None]
AckFn = Callable[[str], None]
EchoConfirmFn = Callable[[str], None]


def _sanitize_wire_text(text: str) -> str:
    """FFE1 文本清洗：去除 \\0/\\r/\\n，保留其余字符。"""
    return text.replace("\x00", "").replace("\r", "").replace("\n", "").strip()


def _conversation_code_exists(code: str) -> bool:
    """识别 conversation_bag 短码（如 LYJXD），避免未进 PYTHONPATH 时误判。"""
    if not CONV_CODE_RE.match(code.strip().upper()):
        return False
    try:
        from voice_remind import conversation_code_exists

        return bool(conversation_code_exists(code))
    except ImportError:
        ws = os.environ.get("BIRD_WS", "")
        if ws and ws not in sys.path:
            sys.path.insert(0, ws)
        try:
            from voice_remind import conversation_code_exists

            return bool(conversation_code_exists(code))
        except ImportError:
            return False


def _normalize_stick(x: float, y: float, z: float) -> str:
    x = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, x))
    y = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, y))
    z = max(-STICK_Z_LIMIT, min(STICK_Z_LIMIT, z))
    return f"X:{x:.2f},Y:{y:.2f},Z:{z:.2f}"


def classify_payload(text: str) -> Tuple[CommandKind, str, str]:
    raw = _sanitize_wire_text(text)
    if not raw:
        return CommandKind.UNKNOWN, "", ""

    if HEARTBEAT_RE.match(raw):
        return CommandKind.HEARTBEAT, "1", "1"

    if HANDSHAKE_RE.match(raw):
        return CommandKind.HANDSHAKE, "m_default", "M_default"

    m = STICK_RE.match(raw)
    if m:
        wire = _normalize_stick(float(m.group(1)), float(m.group(2)), float(m.group(3)))
        return CommandKind.STICK, wire, wire

    if MODE_RE.match(raw):
        suffix = raw.split("_", 1)[1].lower()
        wire = f"M_{suffix}"
        return CommandKind.MODE, wire.lower(), wire

    m_st = ST_RE.match(raw)
    if m_st:
        key = m_st.group(1).lower()
        wire = f"ST:{key}"
        return CommandKind.UPPER_STATE, key, wire

    m_fsm = FSM_RE.match(raw)
    if m_fsm:
        key = m_fsm.group(1).lower()
        wire = f"FSM:{key}"
        return CommandKind.FSM, key, wire

    m_pol = POL_RE.match(raw)
    if m_pol:
        name = m_pol.group(1)
        # SP8 等保留原文大小写；其余策略名统一小写以匹配控制器
        if name.upper() == "SP8":
            name = "SP8"
        else:
            name = name.lower()
        wire = f"POL:{name}"
        return CommandKind.POLICY, name, wire

    if ACTION_RE.match(raw):
        key = re.sub(r"\s+", "", raw).lower()
        return CommandKind.ACTION, key, ACTION_WIRE.get(key, raw)

    if parse_gamepad_combo is not None:
        combo = parse_gamepad_combo(raw)
        if combo is not None:
            wire = combo_to_wire(combo) if combo_to_wire is not None else raw
            return CommandKind.ACTION, combo, wire

    if NECK_CENTER_RE.match(raw):
        return CommandKind.NECK, "neck0", "neck0"

    m_neck = NECK_OFFSET_RE.match(raw)
    if m_neck:
        wire = f"P{m_neck.group(1)}Y{m_neck.group(2)}"
        return CommandKind.NECK, raw, wire

    m_lf = LOCATE_FACE_RE.match(raw)
    if m_lf:
        action = m_lf.group(1).upper()
        wire = f"locate_face {action}"
        return CommandKind.LOCATE_FACE, action, wire

    m_mp = MP_RE.match(raw)
    if m_mp:
        action = m_mp.group(1).upper()
        wire = f"MP {action}"
        return CommandKind.MOTOR_POWER, action, wire

    m_gait = GAIT_RE.match(raw)
    if m_gait:
        action = m_gait.group(1).upper()
        wire = f"GAIT {action}"
        return CommandKind.GAIT, action, wire

    m_sprint = SPRINT_RE.match(raw)
    if m_sprint:
        action = m_sprint.group(1).upper()
        wire = f"LT {action}"
        return CommandKind.SPRINT, action, wire

    m_sound = SOUND_RE.match(raw)
    if m_sound:
        action = m_sound.group(1).upper()
        wire = f"sound {action}"
        return CommandKind.SOUND, action, wire

    m_vol = VOLUME_RE.match(raw)
    if m_vol:
        pct = max(0, min(100, int(m_vol.group(1))))
        wire = f"V {pct}"
        return CommandKind.VOLUME, str(pct), wire

    m_rename = _RENAME_RE.match(raw)
    if m_rename:
        digits = m_rename.group(1)
        name = f"HT_{digits}"
        return CommandKind.RENAME, name, f"rename {name}"

    m_pull = PULL_RE.match(raw)
    if m_pull:
        action = m_pull.group(1).upper()
        wire = f"PULL {action}"
        return CommandKind.PULL, action, wire

    if CONV_CODE_RE.match(raw.strip().upper()) and _conversation_code_exists(raw):
        code = raw.strip().upper()
        return CommandKind.CONVERSATION, code, code

    if re.match(r"^WIFI\s+", raw, re.IGNORECASE):
        try:
            from ble_wifi_manager import parse_wifi_command
        except ImportError:
            parse_wifi_command = None  # type: ignore
        parsed = parse_wifi_command(raw) if parse_wifi_command else None
        if parsed is None:
            return CommandKind.UNKNOWN, raw, raw
        ssid, password = parsed
        payload = f"{ssid}{WIFI_PAYLOAD_SEP}{password}"
        return CommandKind.WIFI, payload, f"WIFI {ssid}"

    return CommandKind.UNKNOWN, raw, raw


def make_notify_reply(wire_text: str) -> bytes:
    return f"ACK:{wire_text}".encode("utf-8", errors="replace")[:180]


class CommandDispatcher:
    def __init__(
        self,
        handle: HandleFn,
        ack: Optional[AckFn] = None,
        echo_confirm: Optional[EchoConfirmFn] = None,
        log_rx: LogFn = print,
        log_warn: Optional[LogFn] = None,
    ) -> None:
        self._handle = handle
        self._ack = ack
        self._echo_confirm = echo_confirm
        self._log_rx = log_rx
        self._log_warn = log_warn or log_rx
        self._queue: Deque[QueuedCommand] = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._recent_action_ts: dict = {}
        self._msg_count = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def on_disconnect(self) -> None:
        with self._lock:
            self._queue.clear()

    def dispatch(self, data: bytes) -> None:
        if len(data) > MAX_PACKET_BYTES:
            return
        try:
            text = data.decode("utf-8", errors="ignore")
        except UnicodeDecodeError:
            return
        text = _sanitize_wire_text(text)
        if not text:
            return

        kind, payload, wire = classify_payload(text)
        if kind == CommandKind.UNKNOWN:
            self._log_rx(f"无法识别: {text!r}")
            return

        if kind == CommandKind.HEARTBEAT:
            return

        if kind == CommandKind.HANDSHAKE:
            self._log_rx("M_default")
            try:
                # 必须通知上层置位 default 标志，否则 ready 条件永远不满足。
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"握手处理失败: {e}")
                return
            if self._ack is not None:
                self._ack("M_default")
            return

        if kind == CommandKind.STICK:
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"摇杆处理失败: {e}")
            return

        if kind == CommandKind.NECK:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"脖子处理失败: {e}")
            return

        if kind == CommandKind.LOCATE_FACE:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"locate_face 处理失败: {e}")
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.MOTOR_POWER:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"MP 处理失败: {e}")
                return
            return

        if kind == CommandKind.GAIT:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"步态处理失败: {e}")
                return
            if self._echo_confirm is not None:
                self._echo_confirm(text)
            return

        if kind == CommandKind.SPRINT:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"疾跑处理失败: {e}")
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.SOUND:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"语音处理失败: {e}")
            return

        if kind == CommandKind.VOLUME:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"音量处理失败: {e}")
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.RENAME:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"rename 处理失败: {e}")
            return

        if kind == CommandKind.WIFI:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"WIFI 处理失败: {e}")
            return

        if kind == CommandKind.PULL:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"PULL 处理失败: {e}")
                return
            if self._echo_confirm is not None:
                self._echo_confirm(wire)
            return

        if kind == CommandKind.CONVERSATION:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"对话语音处理失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        # 模式/上层状态：同步处理并立即 ACK，避免握手 M_default 排队卡住
        if kind == CommandKind.MODE:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"模式处理失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.UPPER_STATE:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"上层状态处理失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.FSM:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"FSM 处理失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.POLICY:
            self._log_rx(wire)
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"策略切换失败: {e}")
                return
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.ACTION:
            now = time.monotonic()
            window = (
                0.35
                if payload in FSM_NAV_KEYS
                else PULSE_ACTION_COOLDOWN_SEC
                if payload in PULSE_ACTION_KEYS
                else 1.5
            )
            if now - self._recent_action_ts.get(payload, 0.0) < window:
                return
            self._recent_action_ts[payload] = now

        cmd = QueuedCommand(kind=kind, wire=wire, payload=payload)
        with self._lock:
            self._queue = deque(c for c in self._queue if c.kind != CommandKind.STICK)
            if kind == CommandKind.ACTION and any(c.payload == payload for c in self._queue):
                return
            self._queue.append(cmd)

    def _worker(self) -> None:
        while not self._stop.is_set():
            cmd: Optional[QueuedCommand] = None
            with self._lock:
                if self._queue:
                    cmd = self._queue.popleft()
            if cmd is None:
                time.sleep(TICK_INTERVAL_SEC)
                continue

            self._msg_count += 1
            self._log_rx(cmd.wire)

            try:
                self._handle(cmd.kind, cmd.payload)
            except Exception as e:
                self._log_warn(f"处理失败: {e}")
                time.sleep(TICK_INTERVAL_SEC)
                continue

            if self._ack is not None:
                self._ack(cmd.wire)

            time.sleep(TICK_INTERVAL_SEC)
