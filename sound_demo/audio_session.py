#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音会话：FFE1 二进制流 → 缓冲 → 扬声器。"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from sound_demo.audio_meter import AudioLevelMeter
from sound_demo.audio_player import PcmStreamPlayer
from sound_demo.protocol import (
    DEFAULT_SAMPLE_RATE,
    FrameType,
    parse_frame_buffered,
    parse_mp_packet,
)

LogFn = Callable[[str], None]


class VoiceSession:
    """管理 sound ON … sound OFF 的实时播放。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._active = False
        self._sample_rate = DEFAULT_SAMPLE_RATE
        self._player = PcmStreamPlayer(sample_rate=self._sample_rate, log=log)
        self._meter = AudioLevelMeter()
        self._rx_buf = bytearray()
        self._frames_in = 0
        self._pcm_bytes = 0
        self._last_seq: Optional[int] = None
        self._started_at = 0.0
        self._dropped_inactive = 0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(self, sample_rate: int = DEFAULT_SAMPLE_RATE) -> bool:
        with self._lock:
            if self._active:
                self._log("[sound] 会话已在进行中")
                return True
            self._sample_rate = max(4000, min(16000, int(sample_rate)))
            self._player = PcmStreamPlayer(sample_rate=self._sample_rate, log=self._log)
            if not self._player.start():
                return False
            self._active = True
            self._rx_buf.clear()
            self._frames_in = 0
            self._pcm_bytes = 0
            self._last_seq = None
            self._dropped_inactive = 0
            self._started_at = time.monotonic()
        self._meter.start()
        self._log(
            "[sound] 会话开始 | 等待 FFE1 音频包 "
            "[0x0B seq_hi seq_lo pcm...] 8kHz mono s16le"
        )
        return True

    def stop(self) -> None:
        with self._lock:
            if not self._active:
                return
            self._active = False
            dropped = self._dropped_inactive
        self._meter.stop()
        self._player.stop()
        elapsed = time.monotonic() - self._started_at
        self._log(
            f"[sound] 会话结束 | 帧={self._frames_in} "
            f"PCM={self._pcm_bytes}B 时长≈{elapsed:.1f}s"
            + (f" (OFF前丢弃{dropped}包)" if dropped else "")
        )

    def on_disconnect(self) -> None:
        self.stop()

    def feed(self, data: bytes) -> None:
        if not data:
            return
        with self._lock:
            if not self._active:
                self._dropped_inactive += 1
                if self._dropped_inactive in (1, 10, 50):
                    self._log(
                        f"[sound] 收到音频 {len(data)}B 但会话未开启 "
                        f"(需先发 sound ON) x{self._dropped_inactive}"
                    )
                return

        # FFE1 每写一次通常即一整包
        frame = parse_mp_packet(data)
        if frame is None:
            with self._lock:
                self._rx_buf.extend(data)
                frame, _ = parse_frame_buffered(self._rx_buf)
        if frame is None:
            if data[0] != 0x0B:
                self._log(f"[sound] 非音频包 首字节=0x{data[0]:02x} len={len(data)}")
            return
        self._handle_frame(frame)

    def _handle_frame(self, frame) -> None:
        self._frames_in += 1
        if self._last_seq is not None and frame.seq != ((self._last_seq + 1) & 0xFFFF):
            if self._frames_in <= 3 or self._frames_in % 50 == 0:
                self._log(f"[sound] 序号跳变 {self._last_seq}→{frame.seq}")
        self._last_seq = frame.seq

        if frame.frame_type == FrameType.END:
            return
        if not frame.pcm:
            return

        self._pcm_bytes += len(frame.pcm)
        self._meter.on_frame(frame.seq, frame.pcm)
        if not self._player.write(frame.pcm):
            self._log("[sound] 播放写入失败，自动结束会话")
            self.stop()
