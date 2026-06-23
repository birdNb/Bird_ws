#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCM 实时播放：PulseAudio pacat → 机器人扬声器。"""

from __future__ import annotations

import os
import subprocess
import threading
from typing import Callable, Optional

from sound_demo.protocol import DEFAULT_CHANNELS, DEFAULT_SAMPLE_RATE, DEFAULT_SAMPLE_WIDTH

LogFn = Callable[[str], None]


class PcmStreamPlayer:
    """向 pacat 持续写入 PCM，低延迟播放。"""

    def __init__(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        channels: int = DEFAULT_CHANNELS,
        log: LogFn = print,
    ) -> None:
        self._sample_rate = sample_rate
        self._channels = channels
        self._log = log
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._bytes_written = 0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def start(self) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return True
            env = os.environ.copy()
            env.setdefault("PULSE_SERVER", "unix:/run/user/1000/pulse/native")
            try:
                self._proc = subprocess.Popen(
                    [
                        "pacat",
                        f"--rate={self._sample_rate}",
                        "--format=s16le",
                        f"--channels={self._channels}",
                        "--latency-msec=60",
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    env=env,
                )
            except FileNotFoundError:
                self._log("[voice] 未找到 pacat，请安装 pulseaudio-utils")
                return False
            self._bytes_written = 0
            self._log(
                f"[voice] 播放已启动 {self._sample_rate}Hz "
                f"mono s16le → PulseAudio"
            )
            return True

    def write(self, pcm: bytes) -> bool:
        if not pcm:
            return True
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None or proc.stdin is None:
                return False
            try:
                proc.stdin.write(pcm)
                proc.stdin.flush()
                self._bytes_written += len(pcm)
                return True
            except (BrokenPipeError, OSError) as exc:
                self._log(f"[voice] 写入播放管道失败: {exc}")
                return False

    def stop(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return
        if proc.stdin is not None:
            try:
                proc.stdin.close()
            except OSError:
                pass
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        self._log(f"[voice] 播放已停止（已写入 {self._bytes_written} 字节 PCM）")
