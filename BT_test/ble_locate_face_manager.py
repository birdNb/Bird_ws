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

_BT_DIR = os.path.dirname(os.path.abspath(__file__))
_BIRD_WS = os.environ.get("BIRD_WS") or os.path.abspath(os.path.join(_BT_DIR, ".."))
_RUN_USER_HOME = os.environ.get("BIRD_HOME") or os.path.expanduser(
    f"~{os.environ.get('BIRD_USER', 'hightorque')}"
)
NECK_HOME_SCRIPT = os.path.join(_BT_DIR, "neck_smooth_home.py")
NECK_STATE_FILE = "/tmp/locate_face_neck.state"
ROS_ENV_SH = os.path.join(_BT_DIR, "ros_env.sh")
START_VERIFY_SEC = 12.0
PROC_EXIT_WAIT_SEC = 3.0
HOMING_EXIT_SEC = 8.0
ORPHAN_KILL_WAIT_SEC = 2.5

LogFn = Callable[[str], None]

_READY_MARKERS = ("[FSM] OK", "进入视觉伺服", "[gui] 后台模式", "[gui] 全屏预览")


def _candidate_roots() -> Tuple[str, ...]:
    """装机包根、工作区根、显式环境变量，按优先级去重。"""
    roots = []
    env_root = os.environ.get("LOCATE_FACE_ROOT", "").strip()
    if env_root:
        roots.append(env_root)
    # Bt_voice_pull_bag 本身
    roots.append(_BIRD_WS)
    # 包内路径：.../Bt_voice_pull_bag → 上一级 Bird_ws
    parent = os.path.abspath(os.path.join(_BIRD_WS, ".."))
    roots.append(parent)
    roots.append(os.path.join(_RUN_USER_HOME, "Bird_ws"))
    # 去重且保序
    seen = set()
    out = []
    for r in roots:
        if r and r not in seen and os.path.isdir(r):
            seen.add(r)
            out.append(r)
    return tuple(out)


def _resolve_locate_face_paths() -> Tuple[str, str, str, str, str]:
    """返回 (cpp_bin, cpp_start, py_script, cpp_root, log_path)。"""
    for root in _candidate_roots():
        cpp_bin = os.path.join(root, "locate_face_cpp/build/locate_face")
        cpp_start = os.path.join(root, "locate_face_cpp/start.sh")
        py_script = os.path.join(root, "locate_face/locate_face.py")
        cpp_root = os.path.join(root, "locate_face_cpp")
        if os.path.isfile(cpp_bin) and os.access(cpp_bin, os.X_OK):
            log_path = os.path.join(cpp_root, "locate_face_ble.log")
            return cpp_bin, cpp_start, py_script, cpp_root, log_path
        if os.path.isfile(cpp_start) or os.path.isfile(py_script):
            log_path = os.path.join(cpp_root, "locate_face_ble.log")
            return cpp_bin, cpp_start, py_script, cpp_root, log_path
    # 回退到 BIRD_WS（便于错误信息）
    cpp_root = os.path.join(_BIRD_WS, "locate_face_cpp")
    return (
        os.path.join(cpp_root, "build/locate_face"),
        os.path.join(cpp_root, "start.sh"),
        os.path.join(_BIRD_WS, "locate_face/locate_face.py"),
        cpp_root,
        os.path.join(cpp_root, "locate_face_ble.log"),
    )


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

    @staticmethod
    def _read_log_tail(path: str, max_bytes: int = 65536) -> str:
        try:
            with open(path, "rb") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - max_bytes))
                return f.read().decode("utf-8", errors="replace")
        except OSError:
            return ""

    def _wait_ready(self, proc: subprocess.Popen, log_path: str) -> Tuple[bool, str]:
        deadline = time.monotonic() + START_VERIFY_SEC
        while time.monotonic() < deadline:
            code = proc.poll()
            if code is not None:
                return False, f"进程已退出 (code={code})"

            if not log_path:
                time.sleep(0.2)
                continue

            tail = self._read_log_tail(log_path)
            if "无法打开相机" in tail or "[cam] 无法打开" in tail:
                return False, "相机打开失败（/dev/video0 可能被占用）"
            if "人脸后端初始化失败" in tail:
                return False, "人脸检测后端初始化失败"
            if "未进入默认执行态" in tail:
                return False, "机器人未处于默认模式，请先发送 M_default"
            if any(m in tail for m in _READY_MARKERS):
                return True, "ready"
            if "[track]" in tail and "face=" in tail:
                return True, "tracking"

            time.sleep(0.25)

        if proc.poll() is not None:
            return False, f"进程已退出 (code={proc.returncode})"

        tail = self._read_log_tail(log_path) if log_path else ""
        if "[FSM] 等待" in tail and not any(m in tail for m in _READY_MARKERS):
            return (
                False,
                "等待 FSM 超时，请先 M_default 并确认 sim2real_master 在运行",
            )
        return False, f"启动超时 ({START_VERIFY_SEC:.0f}s)，详见日志: {log_path}"

    def _kill_orphans(self) -> None:
        for pattern in (
            r"locate_face_cpp/build/locate_face",
            r"locate_face_cpp/start\.sh",
            r"locate_face\.py",
            r"face_yunet_worker\.py",
            r"face_mediapipe_worker\.py",
        ):
            subprocess.run(
                ["pkill", "-f", pattern],
                check=False,
                capture_output=True,
            )
        deadline = time.monotonic() + ORPHAN_KILL_WAIT_SEC
        while time.monotonic() < deadline:
            alive = False
            for pattern in (
                r"locate_face_cpp/build/locate_face",
                r"face_yunet_worker\.py",
            ):
                r = subprocess.run(
                    ["pgrep", "-f", pattern],
                    check=False,
                    capture_output=True,
                )
                if r.returncode == 0:
                    alive = True
                    break
            if not alive:
                return
            time.sleep(0.15)

    def _start(self) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                self._log("[locate_face] 已在运行，忽略 ON")
                return True
            self._proc = None

        self._kill_orphans()

        env = os.environ.copy()
        # systemd User=root 且 HOME=/root 时，子进程 python3 会落到系统 cv2 4.2（无 FaceDetectorYN）；
        _run_user = os.environ.get("BIRD_USER", os.path.basename(_RUN_USER_HOME))
        env["HOME"] = _RUN_USER_HOME
        env["USER"] = _run_user
        env.setdefault("DISPLAY", ":0")
        xauth = os.path.join(_RUN_USER_HOME, ".Xauthority")
        if os.path.exists(xauth):
            env.setdefault("XAUTHORITY", xauth)
        cpp_bin, cpp_start, py_script, cpp_root, log_path = _resolve_locate_face_paths()
        env["LOCATE_FACE_CPP_ROOT"] = cpp_root
        env["BIRD_WS"] = _BIRD_WS
        env.pop("ROS_HOSTNAME", None)
        _py_site = os.path.join(_RUN_USER_HOME, ".local/lib/python3.8/site-packages")
        if os.path.isdir(_py_site):
            _pp = env.get("PYTHONPATH", "")
            env["PYTHONPATH"] = f"{_py_site}:{_pp}" if _pp else _py_site

        if os.path.isfile(cpp_bin) and os.access(cpp_bin, os.X_OK):
            shell_cmd = (
                f'source "{ROS_ENV_SH}" && '
                f'exec "{cpp_bin}"'
            )
            cwd = os.path.dirname(cpp_bin)
            backend = "C++"
        elif os.path.isfile(cpp_start):
            shell_cmd = f'exec "{cpp_start}"'
            cwd = os.path.dirname(cpp_start)
            backend = "C++(start.sh)"
        elif os.path.isfile(py_script):
            shell_cmd = (
                f'source "{ROS_ENV_SH}" && '
                f'exec python3 "{py_script}" --no-gui'
            )
            cwd = os.path.dirname(py_script)
            backend = "Python"
        else:
            self._log(
                f"[locate_face] 未找到可执行文件: {cpp_bin} 或 {py_script}"
            )
            self._log(
                "[locate_face] 请将 locate_face_cpp 放入装机包，"
                "或: cd ~/Bird_ws/locate_face_cpp && ./build.sh"
            )
            return False

        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
        except OSError:
            pass
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

        ok, reason = self._wait_ready(proc, log_path)
        if not ok:
            self._log(f"[locate_face] 启动失败: {reason}")
            if log_path:
                self._log(f"[locate_face] 日志: {log_path}")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except OSError:
                proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    proc.kill()
            if log_f not in (None, subprocess.DEVNULL):
                log_f.close()
            return False

        with self._lock:
            self._proc = proc
        if log_path:
            self._log(f"[locate_face] 已启动 ON ({backend}, pid={proc.pid}), 日志: {log_path}")
        else:
            self._log(f"[locate_face] 已启动 ON ({backend}, pid={proc.pid})")
        if log_f not in (None, subprocess.DEVNULL):
            pass  # 保持日志文件打开供子进程写入
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
