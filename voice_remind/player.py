#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地 WAV 语音提示播放器。

- assets 系统提示：入队阻塞播放，互不打断
- conversation_bag / play_file：打断当前并立即播放
"""

from __future__ import annotations

import os
import queue
import shutil
import subprocess
import threading
import time
from typing import Callable, Dict, Optional, Tuple, Union

from .prompts import BATTERY_VOICE_THRESHOLDS, PROMPTS

LogFn = Callable[[str], None]

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
_DEFAULT_COOLDOWN_SEC = 3.0

QueueItem = Union[str, Tuple[str, str, str], None]


class VoiceRemindPlayer:
    """播放 voice_remind/assets 下的固定语音提示。"""

    def __init__(
        self,
        enabled: bool = True,
        assets_dir: Optional[str] = None,
        log: LogFn = print,
        cooldown_sec: float = _DEFAULT_COOLDOWN_SEC,
    ) -> None:
        self._enabled = enabled
        self._assets_dir = assets_dir or _ASSETS_DIR
        self._log = log
        self._cooldown_sec = cooldown_sec
        self._queue: queue.Queue[QueueItem] = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_play_ts: Dict[str, float] = {}
        self._last_battery_pct: Optional[int] = None
        self._battery_announced: set[int] = set()
        self._play_cmd = self._detect_play_cmd()
        self._system_prompts_ok = False
        # sound OFF 后关闭系统提示音；conversation / 小程序音频仍走 play_file
        self._prompts_enabled = True
        self._play_lock = threading.Lock()
        self._current_proc: Optional[subprocess.Popen[bytes]] = None

    @staticmethod
    def _detect_play_cmd() -> Optional[list[str]]:
        # 勿用 media.role=phone：Pulse 会单独记音量，常比音乐通道小
        if shutil.which("paplay"):
            return ["paplay", "--volume=65536"]
        if shutil.which("aplay"):
            return ["aplay", "-q"]
        return None

    def start(self) -> None:
        if self._thread is not None:
            return
        if self._play_cmd is None:
            self._log("[voice] 未找到 paplay/aplay，语音提示已禁用")
            self._enabled = False
            return
        missing = [
            name
            for name, (fname, _) in PROMPTS.items()
            if not os.path.isfile(os.path.join(self._assets_dir, fname))
        ]
        present = len(PROMPTS) - len(missing)
        # 允许缺文件：只播已有 WAV，便于分批补录音
        self._system_prompts_ok = present > 0
        if missing:
            self._log(
                f"[voice] 系统提示音已就绪 {present}/{len(PROMPTS)} "
                f"({self._play_cmd[0]})；缺少 {missing}"
            )
        else:
            self._log(f"[voice] 系统提示音已就绪 ({self._play_cmd[0]})")
        self._ensure_worker()

    def _stop_current_proc(self) -> None:
        """仅停止当前播放进程（用于服务退出）；不清空队列。"""
        with self._play_lock:
            proc = self._current_proc
            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=0.4)
                except (OSError, subprocess.SubprocessError):
                    try:
                        proc.kill()
                    except OSError:
                        pass
                finally:
                    self._current_proc = None

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def play_file(
        self,
        path: str,
        label: str = "",
        cooldown_key: Optional[str] = None,
    ) -> bool:
        """播放对话/任意 WAV（conversation_bag）：打断当前播放并清空待播队列。"""
        if self._play_cmd is None:
            return False
        if not os.path.isfile(path):
            return False
        self._ensure_worker()
        key = cooldown_key or path
        self._last_play_ts[key] = time.monotonic()
        # 对话优先：停掉当前（含系统提示），丢掉排队中的提示，立刻播本条
        self._stop_current_proc()
        self._drain_queue()
        self._queue.put(("file", path, label or os.path.basename(path)))
        return True

    def prompts_enabled(self) -> bool:
        return self._prompts_enabled

    def set_prompts_enabled(self, enabled: bool) -> None:
        self._prompts_enabled = bool(enabled)
        self._log(
            f"[voice] 系统提示音已{'开启' if self._prompts_enabled else '关闭'}"
        )

    def _ensure_worker(self) -> None:
        if self._thread is not None:
            return
        if self._play_cmd is None:
            self._play_cmd = self._detect_play_cmd()
        if self._play_cmd is None:
            return
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_current_proc()
        self._drain_queue()
        self._queue.put(None)
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _wav_path(self, prompt_id: str) -> Optional[str]:
        item = PROMPTS.get(prompt_id)
        if item is None:
            return None
        path = os.path.join(self._assets_dir, item[0])
        return path if os.path.isfile(path) else None

    def _should_play(self, key: str) -> bool:
        now = time.monotonic()
        last = self._last_play_ts.get(key, 0.0)
        if now - last < self._cooldown_sec:
            return False
        self._last_play_ts[key] = now
        return True

    def play(
        self,
        prompt_id: str,
        *,
        force: bool = False,
    ) -> None:
        """入队播放系统提示音（阻塞式队列，不打断正在播放的条目）。"""
        if not self._enabled or not self._system_prompts_ok:
            return
        if not force and not self._prompts_enabled:
            return
        if not self._should_play(prompt_id):
            return
        if self._wav_path(prompt_id) is None:
            return
        self._ensure_worker()
        self._queue.put(prompt_id)

    def on_ble_ready(self) -> None:
        self.play("ble_ready")

    def on_ble_connected(self) -> None:
        self.play("ble_connected")

    def on_ble_disconnected(self) -> None:
        self.play("ble_disconnected")

    def on_auto_stand(self) -> None:
        self.play("auto_stand")

    def on_motor_on(self) -> None:
        self.play("motor_on")

    def on_walk_mode(self) -> None:
        """GAIT ON → 行走模式"""
        self.play("walk_mode")

    def on_stand_mode(self) -> None:
        """GAIT OFF → 站立模式"""
        self.play("stand_mode")

    def on_pull_on(self) -> None:
        """PULL ON → 拖拽模式已打开"""
        self.play("pull_on")

    def on_pull_off(self) -> None:
        """PULL OFF → 拖拽模式已关闭"""
        self.play("pull_off")

    def on_sprint_on(self) -> None:
        """LT ON → 启动疾跑"""
        self.play("sprint_on")

    def on_sprint_off(self) -> None:
        """LT OFF → 关闭疾跑"""
        self.play("sprint_off")

    def on_mode(self, mode_key: str) -> None:
        """M_* → 对应模式提示音。"""
        key = (mode_key or "").strip().lower()
        prompt_id = {
            "m_default": "mode_default",
            "m_init": "mode_init",
            "m_protect": "mode_protect",
            "m_resetzero": "mode_resetzero",
            "m_tech": "mode_tech",
        }.get(key)
        if prompt_id:
            self.play(prompt_id)

    def on_squat(self) -> None:
        """LT+RT+RB → 蹲下"""
        self.play("squat")

    def on_locate_face(self) -> None:
        """locate_face ON → 人脸追踪"""
        self.play("locate_face")

    def on_sound_on(self) -> None:
        """sound ON → 打开系统语音提示，并播报确认。"""
        self._prompts_enabled = True
        self._log("[voice] 系统提示音已开启")
        self.play("sound_on", force=True)

    def on_sound_off(self) -> None:
        """sound OFF → 先播「关闭语音提示」，之后不再播系统提示音。"""
        self._prompts_enabled = False
        self._log("[voice] 系统提示音已关闭（conversation 等非提示音频仍可播）")
        # 入队播报，不打断当前；后续系统提示因开关关闭不再入队
        self.play("sound_off", force=True)

    def on_battery_pct(self, pct: int) -> None:
        if not self._enabled or not self._system_prompts_ok:
            return
        if not self._prompts_enabled:
            # 仍跟踪电量，避免重新打开后漏报/连报
            pct = max(0, min(100, int(pct)))
            prev = self._last_battery_pct
            self._last_battery_pct = pct
            if prev is not None:
                for threshold in BATTERY_VOICE_THRESHOLDS:
                    if prev > threshold >= pct or (prev > threshold and pct <= threshold):
                        self._battery_announced.add(threshold)
            return
        pct = max(0, min(100, int(pct)))
        prev = self._last_battery_pct
        self._last_battery_pct = pct
        if prev is None:
            return
        for threshold in BATTERY_VOICE_THRESHOLDS:
            if threshold in self._battery_announced:
                continue
            if prev > threshold >= pct or (prev > threshold and pct <= threshold):
                self._battery_announced.add(threshold)
                self.play(f"battery_{threshold}")

    def _worker(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            if isinstance(item, tuple) and item[0] == "file":
                _, path, label = item
            else:
                prompt_id = str(item)
                path = self._wav_path(prompt_id)
                if path is None:
                    continue
                label = PROMPTS.get(prompt_id, (prompt_id, prompt_id))[1]
            if self._play_cmd is None:
                continue
            proc: Optional[subprocess.Popen[bytes]] = None
            try:
                proc = subprocess.Popen(
                    [*self._play_cmd, path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                with self._play_lock:
                    self._current_proc = proc
                # 正常播完；对话打断时由 _stop_current_proc 终止本进程
                proc.wait(timeout=30.0)
                self._log(f"[voice] 播放: {label}")
            except (OSError, subprocess.SubprocessError) as e:
                self._log(f"[voice] 播放失败 {label}: {e}")
            finally:
                with self._play_lock:
                    if proc is not None and self._current_proc is proc:
                        self._current_proc = None
