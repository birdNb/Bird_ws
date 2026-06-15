#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 指令 locate_face ON/OFF → 启停 locate_face.py。"""

from __future__ import annotations

import os
import subprocess
import threading
from typing import Callable, Optional

LOCATE_FACE_SCRIPT = os.path.expanduser("~/Bird_ws/locate_face/locate_face.py")
ROS_ENV_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros_env.sh")
STOP_TIMEOUT_SEC = 5.0

LogFn = Callable[[str], None]


class LocateFaceManager:
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
                self._log("[locate_face] 已在运行，忽略 ON")
                return True
            self._proc = None

        if not os.path.isfile(LOCATE_FACE_SCRIPT):
            self._log(f"[locate_face] 未找到脚本: {LOCATE_FACE_SCRIPT}")
            return False

        self._kill_orphans()

        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        xauth = os.path.expanduser("~/.Xauthority")
        if os.path.exists(xauth):
            env.setdefault("XAUTHORITY", xauth)

        shell_cmd = (
            f'source "{ROS_ENV_SH}" && '
            f'exec python3 "{LOCATE_FACE_SCRIPT}" --no-gui'
        )
        try:
            proc = subprocess.Popen(
                ["bash", "-c", shell_cmd],
                cwd=os.path.dirname(LOCATE_FACE_SCRIPT),
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as e:
            self._log(f"[locate_face] 启动失败: {e}")
            return False

        with self._lock:
            self._proc = proc
        self._log(f"[locate_face] 已启动 ON (pid={proc.pid})")
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
        self._log("[locate_face] 已停止 OFF")
        return True

    def _kill_orphans(self) -> None:
        subprocess.run(
            ["pkill", "-f", r"locate_face\.py"],
            check=False,
            capture_output=True,
        )
