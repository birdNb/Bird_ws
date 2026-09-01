#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 IPv4 转成中文朗读文本，并用 libespeak-ng 合成临时 WAV。"""

from __future__ import annotations

import ctypes
import os
import tempfile
import threading
import wave
from typing import Optional

_DIGIT_ZH = str.maketrans("0123456789.", "零一二三四五六七八九点")

_lock = threading.Lock()
_inited = False
_sample_rate = 22050
_lib = None
_CALLBACK = None
_chunks: list = []


def format_ip_zh(ip: str) -> str:
    """192.168.1.10 → 当前IP地址一九二点一六八点一点一零"""
    ip = (ip or "").strip()
    if not ip:
        return ""
    return "当前IP地址" + ip.translate(_DIGIT_ZH)


def format_lan_connected_ip_zh(ip: str) -> str:
    """检测到局域网 IP 后的整句播报。"""
    body = format_ip_zh(ip)
    if not body:
        return ""
    return "局域网已连接，" + body


def _ensure_espeak() -> bool:
    global _inited, _sample_rate, _lib, _CALLBACK, _chunks
    if _inited:
        return _lib is not None
    with _lock:
        if _inited:
            return _lib is not None
        try:
            lib = ctypes.CDLL("libespeak-ng.so.1")
        except OSError:
            _inited = True
            _lib = None
            return False

        AUDIO_OUTPUT_RETRIEVAL = 1
        lib.espeak_Initialize.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.espeak_Initialize.restype = ctypes.c_int
        lib.espeak_SetVoiceByName.argtypes = [ctypes.c_char_p]
        lib.espeak_SetVoiceByName.restype = ctypes.c_int
        CALLBACK = ctypes.CFUNCTYPE(
            ctypes.c_int, ctypes.POINTER(ctypes.c_short), ctypes.c_int, ctypes.c_void_p
        )
        lib.espeak_SetSynthCallback.argtypes = [CALLBACK]
        lib.espeak_Synth.argtypes = [
            ctypes.c_char_p,
            ctypes.c_size_t,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        lib.espeak_Synth.restype = ctypes.c_int
        lib.espeak_Synchronize.restype = ctypes.c_int

        rate = int(lib.espeak_Initialize(AUDIO_OUTPUT_RETRIEVAL, 0, None, 0))
        if rate > 1000:
            _sample_rate = rate

        for voice in (b"zh", b"cmn", b"zh-cn", b"Chinese"):
            if int(lib.espeak_SetVoiceByName(voice)) == 0:
                break

        chunks: list = []

        @CALLBACK
        def _cb(wav, numsamples, _event):
            if wav and numsamples > 0:
                chunks.append(ctypes.string_at(wav, numsamples * 2))
            return 0

        lib.espeak_SetSynthCallback(_cb)
        _CALLBACK = _cb  # keep ref
        _chunks = chunks
        _lib = lib
        _inited = True
        return True


def synthesize_wav(text: str, out_path: Optional[str] = None) -> Optional[str]:
    """合成 UTF-8 中文文本到 WAV；失败返回 None。"""
    text = (text or "").strip()
    if not text:
        return None
    if not _ensure_espeak() or _lib is None:
        return None
    with _lock:
        _chunks.clear()
        raw = text.encode("utf-8")
        # espeakCHARS_UTF8 = 1
        rc = int(
            _lib.espeak_Synth(
                raw, len(raw) + 1, 0, 1, 0, 1, None, None
            )
        )
        if rc != 0:
            return None
        _lib.espeak_Synchronize()
        pcm = b"".join(_chunks)
        if len(pcm) < 64:
            return None
        if not out_path:
            fd, out_path = tempfile.mkstemp(prefix="bird_ip_", suffix=".wav")
            os.close(fd)
        with wave.open(out_path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(_sample_rate)
            w.writeframes(pcm)
        return out_path


def synthesize_ip_wav(ip: str, out_path: Optional[str] = None) -> Optional[str]:
    spoken = format_lan_connected_ip_zh(ip)
    if not spoken:
        return None
    return synthesize_wav(spoken, out_path=out_path)
