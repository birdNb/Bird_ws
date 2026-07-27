#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""conversation_bag：录音文本前5字拼音首字母（大写）→ WAV。"""

from __future__ import annotations

import json
import os
import re
import threading
from typing import Callable, Dict, Optional, Tuple

from .player import VoiceRemindPlayer

LogFn = Callable[[str], None]

# 小程序发送：录音文案前 5 个汉字拼音首字母，大写，如 LYJXD
_CODE_RE = re.compile(r"^[A-Z][A-Z0-9]{1,31}$")
_BAG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "conversation_bag")

_default_bag: Optional["ConversationBag"] = None
_bag_lock = threading.Lock()


def normalize_code(code: str) -> str:
    return code.strip().upper()


class ConversationBag:
    """加载 conversation_bag/ 下 manifest + WAV，按大写 code 播放。"""

    def __init__(
        self,
        bag_dir: Optional[str] = None,
        player: Optional[VoiceRemindPlayer] = None,
        log: LogFn = print,
    ) -> None:
        self._bag_dir = bag_dir or _BAG_DIR
        self._player = player
        self._log = log
        self._index: Dict[str, Tuple[str, str]] = {}
        self._manifest_mtime: float = 0.0
        self.reload()

    def reload(self) -> int:
        os.makedirs(self._bag_dir, exist_ok=True)
        index: Dict[str, Tuple[str, str]] = {}
        manifest_path = os.path.join(self._bag_dir, "manifest.json")
        if os.path.isfile(manifest_path):
            try:
                with open(manifest_path, encoding="utf-8") as f:
                    data = json.load(f)
                for code, meta in (data.get("items") or {}).items():
                    norm = normalize_code(str(code))
                    if not _CODE_RE.match(norm):
                        continue
                    if not isinstance(meta, dict):
                        meta = {}
                    fname = str(meta.get("file") or f"{norm}.wav")
                    path = os.path.join(self._bag_dir, fname)
                    if os.path.isfile(path):
                        label = str(meta.get("text") or norm)
                        index[norm] = (path, label)
                self._manifest_mtime = os.path.getmtime(manifest_path)
            except (OSError, json.JSONDecodeError) as e:
                self._log(f"[conv] manifest 读取失败: {e}")

        try:
            for fname in os.listdir(self._bag_dir):
                if not fname.lower().endswith(".wav"):
                    continue
                code = normalize_code(fname[:-4])
                if not _CODE_RE.match(code) or code in index:
                    continue
                path = os.path.join(self._bag_dir, fname)
                index[code] = (path, code)
        except OSError:
            pass

        self._index = index
        return len(index)

    def _maybe_reload(self) -> None:
        manifest_path = os.path.join(self._bag_dir, "manifest.json")
        if not os.path.isfile(manifest_path):
            return
        try:
            mtime = os.path.getmtime(manifest_path)
        except OSError:
            return
        if mtime != self._manifest_mtime:
            n = self.reload()
            self._log(f"[conv] manifest 已重载，{n} 条语音")

    def has_code(self, code: str) -> bool:
        self._maybe_reload()
        return normalize_code(code) in self._index

    def lookup(self, code: str) -> Optional[Tuple[str, str]]:
        norm = normalize_code(code)
        if not _CODE_RE.match(norm):
            return None
        self._maybe_reload()
        return self._index.get(norm)

    def play(self, code: str) -> bool:
        norm = normalize_code(code)
        item = self.lookup(norm)
        if item is None:
            self._log(f"[conv] 未知 code: {norm!r}")
            return False
        path, label = item
        if self._player is None:
            self._log(f"[conv] 播放器未就绪: {label}")
            return False
        ok = self._player.play_file(path, label, cooldown_key=f"conv:{norm}")
        if not ok:
            self._log(f"[conv] 播放失败: {label} ({norm})")
        return ok

    def codes(self) -> Tuple[str, ...]:
        self._maybe_reload()
        return tuple(sorted(self._index))


def get_conversation_bag(
    player: Optional[VoiceRemindPlayer] = None,
    log: LogFn = print,
) -> ConversationBag:
    global _default_bag
    with _bag_lock:
        if _default_bag is None:
            _default_bag = ConversationBag(player=player, log=log)
        elif player is not None and _default_bag._player is None:
            _default_bag._player = player
        return _default_bag


def conversation_code_exists(code: str) -> bool:
    norm = normalize_code(code)
    if not _CODE_RE.match(norm):
        return False
    return get_conversation_bag().has_code(norm)
