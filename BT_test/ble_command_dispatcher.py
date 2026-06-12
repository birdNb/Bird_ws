#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFE1 纯文本指令：分类、摇杆去重/队列保护、50ms 匀速分发。
摇杆默认不写调试日志；模式/动作写日志并 ACK。
"""

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
STICK_EPS = 0.01
STICK_XY_LIMIT = 1.0
STICK_Z_LIMIT = 1.5
STICK_LOG_DEBUG = False

STICK_RE = re.compile(
    r"^X:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*Z:\s*([+-]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE,
)
MODE_RE = re.compile(r"^M_(default|init|protect|resetzero|tech)$", re.IGNORECASE)

# 固件约定键名大小写（匹配时归一化为小写键）
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


class CommandKind(str, Enum):
    STICK = "stick"
    MODE = "mode"
    ACTION = "action"
    UNKNOWN = "unknown"


@dataclass
class QueuedCommand:
    kind: CommandKind
    wire: str
    payload: str


LogFn = Callable[[str], None]
HandleFn = Callable[[CommandKind, str], None]
AckFn = Callable[[str], None]


def classify_payload(text: str) -> Tuple[CommandKind, str, str]:
    """返回 (类型, 处理用载荷, 线上原文/规范串)。"""
    raw = text.strip()
    if not raw:
        return CommandKind.UNKNOWN, "", ""

    m = STICK_RE.match(raw)
    if m:
        x, y, z = (float(m.group(i)) for i in (1, 2, 3))
        x = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, x))
        y = max(-STICK_XY_LIMIT, min(STICK_XY_LIMIT, y))
        z = max(-STICK_Z_LIMIT, min(STICK_Z_LIMIT, z))
        wire = f"X:{x:.2f},Y:{y:.2f},Z:{z:.2f}"
        return CommandKind.STICK, wire, wire

    if MODE_RE.match(raw):
        wire = raw if raw[0] == "M" else "M_" + raw[2:].lower()
        if not wire.startswith("M_"):
            wire = "M_" + wire[1:]
        # 统一 M_ 前缀 + 小写后缀
        suffix = raw.split("_", 1)[1].lower()
        wire = f"M_{suffix}"
        return CommandKind.MODE, wire.lower(), wire

    if ACTION_RE.match(raw):
        key = re.sub(r"\s+", "", raw).lower()
        wire = ACTION_WIRE.get(key, raw)
        return CommandKind.ACTION, key, wire

    return CommandKind.UNKNOWN, raw, raw


def make_notify_reply(wire_text: str) -> bytes:
    return f"ACK:{wire_text}".encode("utf-8", errors="replace")[:180]


class CommandDispatcher:
    def __init__(
        self,
        handle: HandleFn,
        ack: Optional[AckFn] = None,
        log: LogFn = print,
    ) -> None:
        self._handle = handle
        self._ack = ack
        self._log = log
        self._queue: Deque[QueuedCommand] = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_stick: Optional[str] = None
        self._msg_count = 0
        self._recent_action_ts: dict = {}

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
            self._last_stick = None

    def dispatch(self, data: bytes) -> None:
        if len(data) > MAX_PACKET_BYTES:
            self._log(f"[dispatcher] 包过长 {len(data)}，丢弃")
            return
        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            self._log("[dispatcher] 非 UTF-8，丢弃")
            return
        if "\n" in text or "\r" in text:
            self._log("[dispatcher] 含换行，丢弃")
            return

        kind, payload, wire = classify_payload(text)
        if kind == CommandKind.UNKNOWN:
            self._log(f"[dispatcher] 无法识别: {text!r}")
            return

        if kind == CommandKind.ACTION:
            now = time.monotonic()
            window = 8.0 if payload == "rt+a" else 1.5
            last = self._recent_action_ts.get(payload, 0.0)
            if now - last < window:
                return
            self._recent_action_ts[payload] = now

        cmd = QueuedCommand(kind=kind, wire=wire, payload=payload)
        with self._lock:
            if kind == CommandKind.STICK:
                if wire == self._last_stick:
                    return
                while len(self._queue) >= QUEUE_MAX:
                    if not self._drop_oldest_stick():
                        break
                self._queue.append(cmd)
            else:
                self._queue = deque(c for c in self._queue if c.kind != CommandKind.STICK)
                if kind == CommandKind.ACTION and any(
                    c.payload == payload for c in self._queue
                ):
                    return
                self._queue.append(cmd)

    def _drop_oldest_stick(self) -> bool:
        for i, item in enumerate(self._queue):
            if item.kind == CommandKind.STICK:
                del self._queue[i]
                return True
        return False

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
            if cmd.kind != CommandKind.STICK or STICK_LOG_DEBUG:
                self._log("=" * 56)
                self._log(f">>> 收到手机消息 #{self._msg_count}")
                self._log(f"    文本: {cmd.wire}")
                self._log(f"    分类: {cmd.kind.value}")
                self._log("=" * 56)

            try:
                self._handle(cmd.kind, cmd.payload)
            except Exception as e:
                self._log(f"[dispatcher] 处理失败: {e}")
                time.sleep(TICK_INTERVAL_SEC)
                continue

            if cmd.kind == CommandKind.STICK:
                with self._lock:
                    self._last_stick = cmd.wire
            elif self._ack is not None:
                self._ack(cmd.wire)

            time.sleep(TICK_INTERVAL_SEC)
