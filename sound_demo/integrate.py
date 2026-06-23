#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
供 BT_test/ble_gatt_server.py 调用的薄封装。
音频处理在独立线程，避免阻塞 BLE 主线程（影响起立/摇杆）。
"""

from __future__ import annotations

import queue
import re
import threading
from typing import Callable, Optional

from sound_demo.audio_session import VoiceSession
from sound_demo.protocol import DEFAULT_SAMPLE_RATE

LogFn = Callable[[str], None]

SOUND_CMD_RE = re.compile(r"^(ON|OFF)$", re.IGNORECASE)
AUDIO_CHAR_UUID = "0000ffe3-0000-1000-8000-00805f9b34fb"
_AUDIO_QUEUE_MAX = 96


class VoiceBleIntegration:
    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._session = VoiceSession(log=log)
        self._audio_q: queue.Queue[bytes] = queue.Queue(maxsize=_AUDIO_QUEUE_MAX)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._audio_worker, daemon=True)
        self._worker.start()

    @property
    def audio_char_uuid(self) -> str:
        return AUDIO_CHAR_UUID

    @property
    def session_active(self) -> bool:
        return self._session.active

    def on_sound_command(self, payload: str) -> Optional[str]:
        m = SOUND_CMD_RE.match(payload.strip())
        if not m:
            self._log(f"[sound] 无效指令: {payload!r}")
            return None
        action = m.group(1).upper()
        if action == "OFF":
            self._session.stop()
            self._drain_audio_queue()
            return "sound OFF"
        if self._session.start(sample_rate=DEFAULT_SAMPLE_RATE):
            return "sound ON"
        return None

    def on_voice_command(self, payload: str) -> Optional[str]:
        return self.on_sound_command(payload)

    def on_audio_write(self, data: bytes) -> None:
        if not data:
            return
        try:
            self._audio_q.put_nowait(data)
        except queue.Full:
            try:
                self._audio_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._audio_q.put_nowait(data)
            except queue.Full:
                pass

    def on_disconnect(self) -> None:
        self._drain_audio_queue()
        self._session.on_disconnect()

    def _drain_audio_queue(self) -> None:
        while True:
            try:
                self._audio_q.get_nowait()
            except queue.Empty:
                break

    def _audio_worker(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._audio_q.get(timeout=0.1)
            except queue.Empty:
                continue
            self._session.feed(data)
