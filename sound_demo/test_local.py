#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地测试：模拟 FFE1 音频包 + 电平条。"""

from __future__ import annotations

import math
import os
import struct
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sound_demo.audio_session import VoiceSession
from sound_demo.protocol import build_mp_packet


def _tone_pcm(sample_rate: int, duration_sec: float, freq_hz: float = 440.0) -> bytes:
    n = int(sample_rate * duration_sec)
    out = bytearray()
    for i in range(n):
        t = i / sample_rate
        val = int(12000 * math.sin(2 * math.pi * freq_hz * t))
        out.extend(struct.pack("<h", val))
    return bytes(out)


def main() -> None:
    session = VoiceSession()
    session.start(8000)
    pcm = _tone_pcm(8000, 1.0, 523.25)
    chunk = 180
    seq = 0
    for off in range(0, len(pcm), chunk):
        session.feed(build_mp_packet(seq, pcm[off : off + chunk]))
        seq += 1
        time.sleep(0.02)
    time.sleep(0.5)
    session.stop()
    print("\n本地播放测试完成（应听到约 1 秒提示音，上方应有电平条）")


if __name__ == "__main__":
    main()
