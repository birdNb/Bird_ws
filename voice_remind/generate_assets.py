#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 espeak-ng 生成固定语音提示 WAV 文件。"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from prompts import PROMPTS  # noqa: E402

ASSETS_DIR = os.path.join(_DIR, "assets")
VOICE = os.environ.get("VOICE_REMIND_ESPEAK_VOICE", "zh")


def main() -> int:
    if shutil.which("espeak-ng") is None:
        print("请先安装: sudo apt-get install -y espeak-ng", file=sys.stderr)
        return 1
    os.makedirs(ASSETS_DIR, exist_ok=True)
    ok = 0
    for prompt_id, (fname, text) in PROMPTS.items():
        out = os.path.join(ASSETS_DIR, fname)
        cmd = ["espeak-ng", "-v", VOICE, "-s", "150", "-w", out, text]
        print(f"[gen] {prompt_id}: {text}")
        try:
            subprocess.run(cmd, check=True)
            ok += 1
        except subprocess.CalledProcessError as e:
            print(f"[error] 生成失败 {fname}: {e}", file=sys.stderr)
    print(f"[done] {ok}/{len(PROMPTS)} → {ASSETS_DIR}")
    return 0 if ok == len(PROMPTS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
