#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用本机 Qwen3-TTS 批量生成 Jarvis 风格系统提示音 WAV。"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from prompts import PROMPTS  # noqa: E402

TTS_URL = os.environ.get("VOICE_TTS_URL", "http://127.0.0.1:8880/v1/audio/speech")
ASSETS_DIR = os.path.join(_DIR, "assets")
VOICE = os.environ.get("VOICE_REMIND_TTS_VOICE", "Ryan")
MODEL = os.environ.get("VOICE_REMIND_TTS_MODEL", "tts-1-zh")
INSTRUCT = (
    "Speak like J.A.R.V.I.S. from Iron Man: calm British male AI butler, "
    "polite, clear, measured pace."
)
TIMEOUT_SEC = int(os.environ.get("VOICE_TTS_TIMEOUT", "900"))


def _synth(text: str) -> bytes:
    payload = {
        "model": MODEL,
        "voice": VOICE,
        "input": text,
        "response_format": "wav",
        "speed": 0.95,
        "language": "Chinese",
        "instruct": INSTRUCT,
    }
    req = urllib.request.Request(
        TTS_URL,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=TIMEOUT_SEC) as resp:
        return resp.read()


def main() -> int:
    os.makedirs(ASSETS_DIR, exist_ok=True)
    print(f"[jarvis] TTS={TTS_URL} voice={VOICE} → {ASSETS_DIR}", flush=True)
    ok = 0
    fail = 0
    for prompt_id, (fname, text) in PROMPTS.items():
        out = os.path.join(ASSETS_DIR, fname)
        print(f"[gen] {prompt_id}: {text}", flush=True)
        t0 = time.time()
        try:
            data = _synth(text)
            if len(data) < 1000:
                raise RuntimeError(f"响应过短: {len(data)} bytes")
            with open(out, "wb") as f:
                f.write(data)
            print(
                f"[ok] {fname} ({len(data)} bytes, {time.time() - t0:.1f}s)",
                flush=True,
            )
            ok += 1
        except (urllib.error.URLError, TimeoutError, OSError, RuntimeError) as e:
            print(f"[fail] {fname}: {e}", flush=True)
            fail += 1
    print(f"[done] ok={ok} fail={fail} / {len(PROMPTS)}", flush=True)
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
