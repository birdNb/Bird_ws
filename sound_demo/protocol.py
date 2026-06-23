#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 语音帧协议：小程序 FFE1 与可选 FFE3。"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

# 小程序 FFE1: [0x0B, seq_hi, seq_lo, pcm...]  每包约 180B PCM
FRAME_MAGIC_MP = 0x0B
HEADER_SIZE_MP = 3

# 可选 FFE3 旧格式
FRAME_MAGIC_LEGACY = 0xA5
FRAME_VERSION_LEGACY = 0x01
HEADER_SIZE_LEGACY = 5

MAX_PCM_PER_FRAME = 200

DEFAULT_SAMPLE_RATE = 8000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2  # s16le


class FrameType(IntEnum):
    PCM = 0x01
    END = 0x02


@dataclass
class AudioFrame:
    frame_type: FrameType
    seq: int
    pcm: bytes


def parse_mp_packet(data: bytes) -> Optional[AudioFrame]:
    """解析单条 FFE1 写入（小程序格式）。"""
    if len(data) < HEADER_SIZE_MP or data[0] != FRAME_MAGIC_MP:
        return None
    seq = (data[1] << 8) | data[2]
    return AudioFrame(FrameType.PCM, seq, data[HEADER_SIZE_MP:])


def parse_frame_buffered(buf: bytearray) -> Tuple[Optional[AudioFrame], None]:
    """从缓冲解析一帧；FFE1 通常一包一帧，直接取首包。"""
    if not buf:
        return None, None
    if buf[0] == FRAME_MAGIC_MP:
        frame = parse_mp_packet(bytes(buf))
        buf.clear()
        return frame, None
    if len(buf) >= HEADER_SIZE_LEGACY and buf[0] == FRAME_MAGIC_LEGACY:
        if buf[1] != FRAME_VERSION_LEGACY:
            buf.pop(0)
            return None, None
        ftype = FrameType(buf[2])
        seq = buf[3] | (buf[4] << 8)
        pcm = bytes(buf[HEADER_SIZE_LEGACY:])
        buf.clear()
        if ftype == FrameType.END:
            return AudioFrame(ftype, seq, b""), None
        return AudioFrame(ftype, seq, pcm), None
    if len(buf) > 512:
        buf.clear()
    return None, None


def build_mp_packet(seq: int, pcm: bytes) -> bytes:
    pcm = pcm[:180]
    return bytes([FRAME_MAGIC_MP, (seq >> 8) & 0xFF, seq & 0xFF]) + pcm
