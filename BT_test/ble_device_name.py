#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 广播名称：默认 HT_88888888，支持 rename HT_xxxxxxxx 持久化。"""

from __future__ import annotations

import os
import re
from typing import Optional, Tuple

DEFAULT_BLE_NAME = "HT_88888888"
BLE_NAME_PREFIX = "HT_"
_NAME_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ble_device_name.conf")

RENAME_RE = re.compile(r"^rename\s+HT_(\d{8})$", re.IGNORECASE)
BLE_NAME_RE = re.compile(r"^HT_(\d{8})$", re.IGNORECASE)


def format_ble_name(digits: str) -> str:
    d = re.sub(r"\D", "", digits)[-8:].zfill(8)
    return f"{BLE_NAME_PREFIX}{d}"


def parse_ble_name(text: str) -> Optional[str]:
    raw = text.strip()
    m = BLE_NAME_RE.match(raw)
    if m:
        return format_ble_name(m.group(1))
    m2 = re.match(r"^(\d{8})$", raw)
    if m2:
        return format_ble_name(m2.group(1))
    return None


def parse_rename_command(text: str) -> Optional[Tuple[str, str]]:
    """解析 rename HT_12345678 → (新名称, 回显原文)。"""
    raw = text.strip()
    m = RENAME_RE.match(raw)
    if not m:
        return None
    name = format_ble_name(m.group(1))
    return name, f"rename {name}"


def load_ble_name() -> str:
    try:
        with open(_NAME_FILE, encoding="utf-8") as f:
            line = f.readline().strip()
        parsed = parse_ble_name(line)
        if parsed:
            return parsed
    except OSError:
        pass
    return DEFAULT_BLE_NAME


def save_ble_name(name: str) -> None:
    parsed = parse_ble_name(name)
    if not parsed:
        return
    tmp = f"{_NAME_FILE}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(parsed + "\n")
    os.replace(tmp, _NAME_FILE)
