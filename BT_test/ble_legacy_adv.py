#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 广播实现：RK 板载走 Legacy HCI，Orin USB 走 btmgmt。"""

from __future__ import annotations

import re
import subprocess
import time
from typing import Callable, List, Optional

from platform_detect import PlatformInfo, detect_platform

LogFn = Callable[[str], None]


def build_adv_payload(name: str) -> bytes:
    name_b = name.encode("utf-8")
    base = bytes([0x02, 0x01, 0x06, 0x03, 0x03, 0xE0, 0xFF])
    budget = 31 - len(base) - 2
    name_b = name_b[: max(0, budget)]
    return (base + bytes([len(name_b) + 1, 0x09]) + name_b)[:31]


def build_scan_rsp(name: str) -> bytes:
    name_b = name.encode("utf-8")[:25]
    return bytes([len(name_b) + 1, 0x09]) + name_b


def _run(argv: List[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        argv, capture_output=True, text=True, timeout=timeout, check=False
    )


def hci_le_cmd(hci_dev: str, *args: str) -> bool:
    try:
        r = _run(["hcitool", "-i", hci_dev, "cmd", "0x08", *args])
        out = (r.stdout or "") + (r.stderr or "")
        if re.search(r"\b00\s*$", out) or " 00 " in out:
            return True
        if " 0c " in out or " 12 " in out:
            return False
        return r.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return False


def btmgmt(hci_dev: str, *args: str) -> None:
    idx = hci_dev.replace("hci", "")
    try:
        _run(["btmgmt", "--index", idx, *args])
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass


def prepare_adapter(hci_dev: str, name: str, platform: Optional[PlatformInfo] = None) -> None:
    plat = platform or detect_platform()
    for cmd in (
        ["rfkill", "unblock", "bluetooth"],
        ["hciconfig", hci_dev, "up"],
        ["bluetoothctl", "power", "on"],
    ):
        _run(cmd)
    btmgmt(hci_dev, "le", "on")
    btmgmt(hci_dev, "connectable", "on")
    btmgmt(hci_dev, "discov", "off")
    btmgmt(hci_dev, "pairable", "off")
    btmgmt(hci_dev, "bondable", "off")
    btmgmt(hci_dev, "advertising", "off")
    btmgmt(hci_dev, "name", name)
    _run(["hciconfig", hci_dev, "name", name])
    _run(["bluetoothctl", "system-alias", name])
    if plat.is_onboard_bt:
        _run(["hciconfig", hci_dev, "noscan"])
    time.sleep(0.15)


def stop_advertising(hci_dev: str) -> None:
    hci_le_cmd(hci_dev, "0x000a", "0x00")
    btmgmt(hci_dev, "advertising", "off")


def start_legacy_hci(hci_dev: str, name: str) -> bool:
    adv = build_adv_payload(name)
    sr = build_scan_rsp(name)
    hci_le_cmd(hci_dev, "0x0039", "0x00", "0x00", "0x00")
    hci_le_cmd(
        hci_dev,
        "0x0006",
        "0xa0",
        "0x00",
        "0xc0",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x00",
        "0x07",
        "0x00",
    )
    ok_data = hci_le_cmd(
        hci_dev, "0x0008", *[f"0x{len(adv):02x}"] + [f"0x{b:02x}" for b in adv]
    )
    hci_le_cmd(
        hci_dev, "0x0009", *[f"0x{len(sr):02x}"] + [f"0x{b:02x}" for b in sr]
    )
    ok_on = hci_le_cmd(hci_dev, "0x000a", "0x01")
    return ok_data and ok_on


def start_btmgmt_adv(hci_dev: str, name: str, force: bool = False) -> bool:
    idx = hci_dev.replace("hci", "")
    adv = build_adv_payload(name)
    sr = build_scan_rsp(name)
    if force:
        btmgmt(hci_dev, "rm-adv", "1")
    r = _run(
        [
            "btmgmt",
            "--index",
            idx,
            "add-adv",
            "-d",
            adv.hex(),
            "-s",
            sr.hex(),
            "-c",
            "1",
        ],
        timeout=6,
    )
    if r.returncode == 0:
        btmgmt(hci_dev, "advertising", "on")
        return True
    return False


def start_advertising(
    hci_dev: str,
    name: str,
    platform: Optional[PlatformInfo] = None,
    force: bool = False,
    log: Optional[LogFn] = None,
) -> bool:
    """按平台选择广播方式。"""
    plat = platform or detect_platform()
    prepare_adapter(hci_dev, name, plat)

    if plat.adv_mode == "legacy_hci":
        ok = start_legacy_hci(hci_dev, name)
        if not ok:
            ok = start_btmgmt_adv(hci_dev, name, force=force)
        if log:
            if ok:
                log(f"[adv] Legacy 广播已开启 ({plat.hw_desc}): {name}")
            else:
                log(f"[adv] 板载蓝牙广播失败: {name}")
        return ok

    ok = start_btmgmt_adv(hci_dev, name, force=force)
    if not ok:
        ok = start_legacy_hci(hci_dev, name)
    if log:
        if ok:
            log(f"[adv] USB 蓝牙广播已开启 ({plat.hw_desc}): {name}")
        else:
            log(f"[adv] USB 蓝牙广播失败: {name}")
    return ok
