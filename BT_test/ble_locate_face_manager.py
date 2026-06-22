#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 指令 locate_face ON/OFF → 启停 locate_face_cpp（优先）或 locate_face.py。"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from typing import Callable, Optional, Tuple

LOCATE_FACE_CPP_BIN = os.path.expanduser("~/Bird_ws/locate_face_cpp/build/locate_face")
LOCATE_FACE_CPP_START = os.path.expanduser("~/Bird_ws/locate_face_cpp/start.sh")
LOCATE_FACE_SCRIPT = os.path.expanduser("~/Bird_ws/locate_face/locate_face.py")
NECK_HOME_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "neck_smooth_home.py")
NECK_STATE_FILE = "/tmp/locate_face_neck.state"
ROS_ENV_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros_env.sh")
START_VERIFY_SEC = 1.5
PROC_EXIT_WAIT_SEC = 3.0
HOMING_EXIT_SEC = 8.0

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

    def _read_neck_state(self) -> Tuple[float, float]:
        try:
            with open(NECK_STATE_FILE, "r", encoding="ascii") as f:
                parts = f.read().strip().split()
            if len(parts) >= 2:
                return float(parts[0]), float(parts[1])
        except (OSError, ValueError):
            pass
        return 0.0, 0.0

    def _smooth_neck_home(self, yaw_deg: float, pitch_deg: float) -> None:
        if abs(yaw_deg) < 0.1 and abs(pitch_deg) < 0.1:
            return
        if not os.path.isfile(NECK_HOME_SCRIPT):
            self._log("[locate_face] 未找到 neck_smooth_home.py，跳过 BLE 侧回中")
            return
        self._log(
            f"[locate_face] BLE 侧平滑回中 yaw={yaw_deg:+.1f} pitch={pitch_deg:+.1f}"
        )
        cmd = (
            f'source "{ROS_ENV_SH}" && '
            f'exec python3 "{NECK_HOME_SCRIPT}" {yaw_deg:.3f} {pitch_deg:.3f}'
        )
        try:
            subprocess.run(
                ["bash", "-c", cmd],
                timeout=HOMING_EXIT_SEC,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self._log("[locate_face] BLE 侧脖子回中超时")

    def _start(self) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                self._log("[locate_face] 已在运行，忽略 ON")
                return True
            self._proc = None

        self._kill_orphans()

        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        xauth = os.path.expanduser("~/.Xauthority")
        if os.path.exists(xauth):
            env.setdefault("XAUTHORITY", xauth)
        env["LOCATE_FACE_CPP_ROOT"] = os.path.expanduser("~/Bird_ws/locate_face_cpp")

        if os.path.isfile(LOCATE_FACE_CPP_BIN) and os.access(LOCATE_FACE_CPP_BIN, os.X_OK):
            shell_cmd = (
                f'source "{ROS_ENV_SH}" && '
                f'exec "{LOCATE_FACE_CPP_BIN}"'
            )
            cwd = os.path.dirname(LOCATE_FACE_CPP_BIN)
            backend = "C++"
        elif os.path.isfile(LOCATE_FACE_CPP_START):
            shell_cmd = f'exec "{LOCATE_FACE_CPP_START}"'
            cwd = os.path.dirname(LOCATE_FACE_CPP_START)
            backend = "C++(start.sh)"
        elif os.path.isfile(LOCATE_FACE_SCRIPT):
            shell_cmd = (
                f'source "{ROS_ENV_SH}" && '
                f'exec python3 "{LOCATE_FACE_SCRIPT}" --no-gui'
            )
            cwd = os.path.dirname(LOCATE_FACE_SCRIPT)
            backend = "Python"
        else:
            self._log(
                f"[locate_face] 未找到可执行文件: {LOCATE_FACE_CPP_BIN} 或 {LOCATE_FACE_SCRIPT}"
            )
            self._log("[locate_face] 请先执行: cd ~/Bird_ws/locate_face_cpp && ./build.sh")
            return False

        log_path = os.path.expanduser("~/Bird_ws/locate_face_cpp/locate_face_ble.log")
        try:
            log_f = open(log_path, "ab", buffering=0)
        except OSError:
            log_f = subprocess.DEVNULL
            log_path = ""

        try:
            proc = subprocess.Popen(
                ["bash", "-c", shell_cmd],
                cwd=cwd,
                env=env,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as e:
            self._log(f"[locate_face] 启动失败: {e}")
            if log_f not in (None, subprocess.DEVNULL):
                log_f.close()
            return False

        time.sleep(START_VERIFY_SEC)
        if proc.poll() is not None:
            self._log(
                f"[locate_face] {backend} 进程已退出 (code={proc.returncode})"
                + (f", 日志: {log_path}" if log_path else "")
            )
            if log_f not in (None, subprocess.DEVNULL):
                log_f.close()
            return False

        with self._lock:
            self._proc = proc
        if log_path:
            self._log(f"[locate_face] 已启动 ON ({backend}, pid={proc.pid}), 日志: {log_path}")
        else:
            self._log(f"[locate_face] 已启动 ON ({backend}, pid={proc.pid})")
        return True

    def _stop(self) -> bool:
        with self._lock:
            proc = self._proc
            self._proc = None

        yaw_deg, pitch_deg = self._read_neck_state()

        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except OSError:
                proc.terminate()
            try:
                proc.wait(timeout=HOMING_EXIT_SEC)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    proc.kill()
                proc.wait(timeout=2.0)

        yaw_deg, pitch_deg = self._read_neck_state()
        self._smooth_neck_home(yaw_deg, pitch_deg)

        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                proc.kill()
            proc.wait(timeout=PROC_EXIT_WAIT_SEC)

        self._log("[locate_face] 已停止 OFF")
        return True

    def _kill_orphans(self) -> None:
        for pattern in (
            r"locate_face_cpp/build/locate_face",
            r"locate_face_cpp/start\.sh",
            r"locate_face\.py",
        ):
            subprocess.run(
                ["pkill", "-f", pattern],
                check=False,
                capture_output=True,
            )
