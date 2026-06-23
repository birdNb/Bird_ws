#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sound ON 时终端实时电平条（stderr 单行刷新，不刷屏）。"""

from __future__ import annotations

from typing import Optional

import math
import struct
import sys
import threading
import time

_BAR = "▁▂▃▄▅▆▇█"
_WIDTH = 36


def pcm_peak(pcm: bytes) -> float:
    """0.0~1.0 峰值。"""
    if len(pcm) < 2:
        return 0.0
    n = len(pcm) // 2
    try:
        samples = struct.unpack(f"<{n}h", pcm[: n * 2])
    except struct.error:
        return 0.0
    if not samples:
        return 0.0
    peak = max(abs(s) for s in samples)
    return min(1.0, peak / 32768.0)


class AudioLevelMeter:
    def __init__(self, width: int = _WIDTH) -> None:
        self._width = width
        self._lock = threading.Lock()
        self._level = 0.0
        self._frames = 0
        self._pcm_bytes = 0
        self._last_seq: int = -1
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop.clear()
        with self._lock:
            self._level = 0.0
            self._frames = 0
            self._pcm_bytes = 0
            self._last_seq = -1
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        self._clear_line()

    def stop(self) -> None:
        self._stop.set()
        self._clear_line()

    def on_frame(self, seq: int, pcm: bytes) -> None:
        peak = pcm_peak(pcm)
        with self._lock:
            self._level = max(peak, self._level * 0.72)
            self._frames += 1
            self._pcm_bytes += len(pcm)
            self._last_seq = seq

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._draw()
            time.sleep(0.05)
        self._clear_line()

    def _draw(self) -> None:
        with self._lock:
            lvl = self._level
            frames = self._frames
            pcm_b = self._pcm_bytes
            seq = self._last_seq
            self._level *= 0.92
        idx = min(len(_BAR) - 1, int(lvl * len(_BAR)))
        ch = _BAR[idx] if lvl > 0.01 else " "
        filled = int(lvl * self._width)
        bar = "█" * filled + "░" * (self._width - filled)
        pct = lvl * 100.0
        line = (
            f"\r\033[36m[sound] |{bar}| {ch} {pct:5.1f}% "
            f"seq={seq:5d} 帧={frames:5d} {pcm_b:7d}B\033[0m"
        )
        sys.stderr.write(line)
        sys.stderr.flush()

    def _clear_line(self) -> None:
        sys.stderr.write("\r\033[K")
        sys.stderr.flush()
