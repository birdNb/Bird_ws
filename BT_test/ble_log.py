#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 终端日志：RX 红 / TX 绿。"""

from __future__ import annotations

import sys
import time

_USE_COLOR = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()
_RED = "\033[31m" if _USE_COLOR else ""
_GREEN = "\033[32m" if _USE_COLOR else ""
_YELLOW = "\033[33m" if _USE_COLOR else ""
_RESET = "\033[0m" if _USE_COLOR else ""


def _ts() -> str:
    return time.strftime("%H:%M:%S")


def log_info(msg: str) -> None:
    print(f"[{_ts()}] {msg}", flush=True)


def log_warn(msg: str) -> None:
    print(f"{_YELLOW}[{_ts()}] {msg}{_RESET}", flush=True)


def log_rx(msg: str) -> None:
    print(f"{_RED}[{_ts()}] RX {msg}{_RESET}", flush=True)


def log_tx(msg: str) -> None:
    print(f"{_GREEN}[{_ts()}] TX {msg}{_RESET}", flush=True)
