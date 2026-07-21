#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将 assets/ 下提示音峰值归一化到约 -1 dBFS（与音乐响度接近）。"""

from typing import Optional

import os
import subprocess
import sys
import tempfile

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from prompts import PROMPTS  # noqa: E402

ASSETS_DIR = os.path.join(_DIR, "assets")
TARGET_PEAK_DB = -1.0


def _peak_db(path: str) -> Optional[float]:
    r = subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-i",
            path,
            "-af",
            "volumedetect",
            "-f",
            "null",
            "-",
        ],
        capture_output=True,
        text=True,
    )
    for line in (r.stderr or "").splitlines():
        if "max_volume:" in line:
            # max_volume: -12.1 dB
            part = line.split("max_volume:")[1].strip().split()[0]
            return float(part)
    return None


def normalize_file(path: str) -> bool:
    peak = _peak_db(path)
    if peak is None:
        print(f"[skip] 无法检测峰值: {path}", file=sys.stderr)
        return False
    if peak >= TARGET_PEAK_DB - 0.3:
        print(f"[ok] 已足够响 {os.path.basename(path)} peak={peak:.1f}dB")
        return True
    gain = TARGET_PEAK_DB - peak
    fd, tmp = tempfile.mkstemp(suffix=".wav", dir=ASSETS_DIR)
    os.close(fd)
    try:
        r = subprocess.run(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-i",
                path,
                "-af",
                f"volume={gain}dB",
                tmp,
            ],
            capture_output=True,
            text=True,
        )
        if r.returncode != 0:
            print(f"[fail] {path}: {r.stderr}", file=sys.stderr)
            return False
        os.replace(tmp, path)
        new_peak = _peak_db(path)
        print(
            f"[ok] {os.path.basename(path)} {peak:.1f}dB → {new_peak:.1f}dB "
            f"(+{gain:.1f}dB)"
        )
        return True
    finally:
        if os.path.isfile(tmp):
            os.remove(tmp)


def main() -> int:
    if not shutil_which("ffmpeg"):
        print("需要 ffmpeg", file=sys.stderr)
        return 1
    os.makedirs(ASSETS_DIR, exist_ok=True)
    ok = 0
    for _pid, (fname, _text) in PROMPTS.items():
        path = os.path.join(ASSETS_DIR, fname)
        if not os.path.isfile(path):
            print(f"[skip] 缺少 {fname}")
            continue
        if normalize_file(path):
            ok += 1
    print(f"[done] {ok}/{len(PROMPTS)}")
    return 0


def shutil_which(cmd: str) -> bool:
    from shutil import which

    return which(cmd) is not None


if __name__ == "__main__":
    raise SystemExit(main())
