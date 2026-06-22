#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 指令 V {0-100} → 系统播放音量（PulseAudio / amixer）。"""

from __future__ import annotations

import os
import subprocess
from typing import Callable, Optional

LogFn = Callable[[str], None]


class VolumeController:
    """V 10 表示将系统主音量设为 10%。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._last_percent: Optional[int] = None

    def handle(self, percent_str: str) -> bool:
        try:
            pct = max(0, min(100, int(percent_str.strip())))
        except ValueError:
            self._log(f"[V] 无效音量: {percent_str!r}")
            return False
        return self._apply(pct)

    @property
    def last_percent(self) -> Optional[int]:
        return self._last_percent

    def _apply(self, pct: int) -> bool:
        env = os.environ.copy()
        env.setdefault("PULSE_SERVER", "unix:/run/user/1000/pulse/native")

        result = subprocess.run(
            ["amixer", "-D", "pulse", "sset", "Master", f"{pct}%"],
            capture_output=True,
            text=True,
            timeout=3.0,
            env=env,
        )
        if result.returncode == 0:
            self._last_percent = pct
            self._log(f"[V] 音量设为 {pct}%")
            return True

        err = (result.stderr or result.stdout or "").strip()
        self._log(f"[V] 设置音量失败 ({pct}%): {err or 'amixer 无输出'}")
        return False
