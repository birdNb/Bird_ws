#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 终端日志：时间前缀 HH:MM:SS:；RX 红 / TX 蓝。"""

from __future__ import annotations

import os
import time

# systemd/journal 非 TTY，默认仍输出 ANSI（journalctl -f 可显示颜色）
_USE_COLOR = os.environ.get("BLE_LOG_COLOR", "1") != "0"
_RED = "\033[31m" if _USE_COLOR else ""
_BLUE = "\033[34m" if _USE_COLOR else ""
_YELLOW = "\033[33m" if _USE_COLOR else ""
_RESET = "\033[0m" if _USE_COLOR else ""


def _ts() -> str:
    return time.strftime("%H:%M:%S")


def _prefix() -> str:
    return f"{_ts()}:"


def log_info(msg: str) -> None:
    print(f"{_prefix()} {msg}", flush=True)


def log_warn(msg: str) -> None:
    print(f"{_YELLOW}{_prefix()} {msg}{_RESET}", flush=True)


def log_rx(msg: str) -> None:
    print(f"{_prefix()} {_RED}RX {msg}{_RESET}", flush=True)


def log_tx(msg: str) -> None:
    print(f"{_prefix()} {_BLUE}TX {msg}{_RESET}", flush=True)
