#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 广播名称：默认 HT_88888888，支持 rename HT_xxxxxxxx 持久化。"""

from __future__ import annotations

import os
import re
from typing import Optional, Tuple

DEFAULT_BLE_NAME = "HT_88888888"
BLE_NAME_PREFIX = "HT_"
PERSISTENT_NAME_FILE = "/var/lib/bird-ble/ble_device_name.conf"

RENAME_RE = re.compile(r"^rename\s+HT_(\d{8})$", re.IGNORECASE)
BLE_NAME_RE = re.compile(r"^HT_(\d{8})$", re.IGNORECASE)


def format_ble_name(digits: str) -> str:
    d = re.sub(r"\D", "", digits)[-8:].zfill(8)
    return f"{BLE_NAME_PREFIX}{d}"


def parse_ble_name(text: str) -> Optional[str]:
    raw = text.strip()
    m = re.match(r"^HT_(\d+)$", raw, re.IGNORECASE)
    if m:
        return format_ble_name(m.group(1))
    m2 = re.match(r"^(\d+)$", raw)
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


def _pkg_name_file() -> Optional[str]:
    env_file = os.environ.get("BLE_DEVICE_NAME_FILE", "").strip()
    if env_file and env_file != PERSISTENT_NAME_FILE and os.path.isfile(env_file):
        return env_file
    pkg = os.environ.get("PKG_DIR", "").strip()
    if pkg:
        path = os.path.join(pkg, "ble_device_name.conf")
        if os.path.isfile(path):
            return path
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.join(os.path.dirname(here), "ble_device_name.conf")
    if os.path.isfile(parent):
        return parent
    local = os.path.join(here, "ble_device_name.conf")
    if os.path.isfile(local):
        return local
    return None


def _read_name_file(path: str) -> Optional[str]:
    try:
        with open(path, encoding="utf-8") as f:
            line = f.readline().strip()
        return parse_ble_name(line)
    except OSError:
        return None


def _migrate_to_persistent() -> None:
    """首次运行：把安装包内 ble_device_name.conf 迁入 /var/lib/bird-ble/。"""
    if os.path.isfile(PERSISTENT_NAME_FILE):
        return
    pkg_file = _pkg_name_file()
    if not pkg_file:
        return
    name = _read_name_file(pkg_file)
    if not name or name == DEFAULT_BLE_NAME:
        return
    try:
        os.makedirs(os.path.dirname(PERSISTENT_NAME_FILE), mode=0o755, exist_ok=True)
        _write_name_file(PERSISTENT_NAME_FILE, name)
    except OSError:
        pass


def _write_name_file(path: str, name: str) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, mode=0o755, exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(name + "\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    try:
        os.chmod(path, 0o644)
    except OSError:
        pass


def persistent_name_file() -> str:
    return PERSISTENT_NAME_FILE


def load_ble_name() -> str:
    _migrate_to_persistent()
    if os.path.isfile(PERSISTENT_NAME_FILE):
        name = _read_name_file(PERSISTENT_NAME_FILE)
        if name:
            return name
    pkg_file = _pkg_name_file()
    if pkg_file:
        name = _read_name_file(pkg_file)
        if name:
            return name
    return DEFAULT_BLE_NAME


def save_ble_name(name: str) -> None:
    parsed = parse_ble_name(name)
    if not parsed:
        return
    _write_name_file(PERSISTENT_NAME_FILE, parsed)
    # 同步写回安装包 conf，便于查看；失败不影响持久化
    pkg_file = _pkg_name_file()
    if pkg_file and pkg_file != PERSISTENT_NAME_FILE:
        try:
            _write_name_file(pkg_file, parsed)
        except OSError:
            pass
