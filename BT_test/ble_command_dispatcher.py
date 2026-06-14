#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FFE1 指令分类与分发：摇杆直通保活，模式/动作入队。"""

from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Optional, Tuple

MAX_PACKET_BYTES = 64
QUEUE_MAX = 32
TICK_INTERVAL_SEC = 0.05
STICK_XY_LIMIT = 1.0
STICK_Z_LIMIT = 1.5

# 摇杆：XYZ 必填；可选 ,N:序号 供小程序 20Hz 保活（板端忽略 N 仅解析 XYZ）
STICK_RE = re.compile(
    r"^X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)"
    r"(?:\s*,\s*N:\s*\d+)?\s*$",
    re.IGNORECASE,
)
MODE_RE = re.compile(r"^M_(default|init|protect|resetzero|tech)$", re.IGNORECASE)

ACTION_WIRE = {
    "lt+rt+start": "LT+RT+start",
    "lt+rt+rb": "LT+RT+RB",
    "lt+rt+b": "LT+RT+B",
    "lt+rt+lb": "LT+RT+LB",
    "rt+a": "RT+A",
}
ACTION_RE = re.compile(
    r"^(LT\+RT\+start|LT\+RT\+RB|LT\+RT\+B|LT\+RT\+LB|RT\+A)$",
    re.IGNORECASE,
)
NECK_OFFSET_RE = re.compile(r"^[Pp]([+-]?\d+)Y([+-]?\d+)$")
NECK_CENTER_RE = re.compile(r"^neck0$", re.IGNORECASE)


class CommandKind(str, Enum):
    STICK = "stick"
    MODE = "mode"
    ACTION = "action"
    NECK = "neck"
    UNKNOWN = "unknown"


@dataclass
class QueuedCommand:
    kind: CommandKind
    wire: str
    payload: str


LogFn = Callable[[str], None]
HandleFn = Callable[[CommandKind, str], None]
AckFn = Callable[[str], None]


def _normalize_stick(x: float, y: float, z: float) -> str:
    x = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, x))
    y = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, y))
    z = max(-STICK_Z_LIMIT, min(STICK_Z_LIMIT, z))
    return f"X:{x:.2f},Y:{y:.2f},Z:{z:.2f}"


def classify_payload(text: str) -> Tuple[CommandKind, str, str]:
    raw = text.strip()
    if not raw:
        return CommandKind.UNKNOWN, "", ""

    m = STICK_RE.match(raw)
    if m:
        wire = _normalize_stick(float(m.group(1)), float(m.group(2)), float(m.group(3)))
        return CommandKind.STICK, wire, wire

    if MODE_RE.match(raw):
        suffix = raw.split("_", 1)[1].lower()
        wire = f"M_{suffix}"
        return CommandKind.MODE, wire.lower(), wire

    if ACTION_RE.match(raw):
        key = re.sub(r"\s+", "", raw).lower()
        return CommandKind.ACTION, key, ACTION_WIRE.get(key, raw)

    if NECK_CENTER_RE.match(raw):
        return CommandKind.NECK, "neck0", "neck0"

    m_neck = NECK_OFFSET_RE.match(raw)
    if m_neck:
        wire = f"P{m_neck.group(1)}Y{m_neck.group(2)}"
        return CommandKind.NECK, raw, wire

    return CommandKind.UNKNOWN, raw, raw


def make_notify_reply(wire_text: str) -> bytes:
    return f"ACK:{wire_text}".encode("utf-8", errors="replace")[:180]


class CommandDispatcher:
    def __init__(
        self,
        handle: HandleFn,
        ack: Optional[AckFn] = None,
        log_rx: LogFn = print,
        log_warn: Optional[LogFn] = None,
    ) -> None:
        self._handle = handle
        self._ack = ack
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
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            return
        if not text or "\n" in text or "\r" in text:
            return

        kind, payload, wire = classify_payload(text)
        if kind == CommandKind.UNKNOWN:
            self._log_rx(f"无法识别: {text!r}")
            return

        if kind == CommandKind.STICK:
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"摇杆处理失败: {e}")
            return

        if kind == CommandKind.NECK:
            self._log_rx(f"neck: {wire}")
            try:
                self._handle(kind, payload)
            except Exception as e:
                self._log_warn(f"脖子处理失败: {e}")
            if self._ack is not None:
                self._ack(wire)
            return

        if kind == CommandKind.ACTION:
            now = time.monotonic()
            window = 8.0 if payload == "rt+a" else 1.5
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
            self._log_rx(f"{cmd.kind.value}: {cmd.wire}")

            try:
                self._handle(cmd.kind, cmd.payload)
            except Exception as e:
                self._log_warn(f"处理失败: {e}")
                time.sleep(TICK_INTERVAL_SEC)
                continue

            if self._ack is not None:
                self._ack(cmd.wire)

            time.sleep(TICK_INTERVAL_SEC)
