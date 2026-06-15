#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 指令 HI ON/OFF → 启停 hand_identify_cpp/start.sh（手势控制）。"""

from __future__ import annotations

import os
import subprocess
import threading
from typing import Callable, Optional

HAND_IDENTIFY_START = os.path.expanduser(
    "~/Bird_ws/hand_identify_cpp/start.sh"
)
STOP_TIMEOUT_SEC = 8.0

LogFn = Callable[[str], None]


class HandIdentifyManager:
    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None

    def handle(self, action: str) -> bool:
        cmd = action.strip().upper()
        if cmd == "ON":
            return self._start()
        if cmd == "OFF":
            return self._stop()
        return False

    def stop(self) -> None:
        self._stop()

    def _start(self) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                self._log("[HI] 手势控制已在运行，忽略 ON")
                return True
            self._proc = None

        if not os.path.isfile(HAND_IDENTIFY_START):
            self._log(f"[HI] 未找到脚本: {HAND_IDENTIFY_START}")
            return False

        self._kill_orphans()

        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        env.setdefault("LANG", "C.UTF-8")
        env.setdefault("LC_ALL", "C.UTF-8")
        xauth = os.path.expanduser("~/.Xauthority")
        if os.path.exists(xauth):
            env.setdefault("XAUTHORITY", xauth)

        try:
            proc = subprocess.Popen(
                ["bash", HAND_IDENTIFY_START, "--no-joy"],
                cwd=os.path.dirname(HAND_IDENTIFY_START),
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as e:
            self._log(f"[HI] 启动失败: {e}")
            return False

        with self._lock:
            self._proc = proc
        self._log(f"[HI] 手势控制已启动 ON (pid={proc.pid})")
        return True

    def _stop(self) -> bool:
        with self._lock:
            proc = self._proc
            self._proc = None

        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)

        self._kill_orphans()
        self._log("[HI] 手势控制已停止 OFF")
        return True

    def _kill_orphans(self) -> None:
        for pattern in (
            r"hand_identify_cpp/build/vision_controller",
            r"zed_gesture_recognition\.py",
        ):
            subprocess.run(
                ["pkill", "-f", pattern],
                check=False,
                capture_output=True,
            )
