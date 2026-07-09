#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE 广播：RK Realtek 仅 Legacy HCI；Orin USB 优先 btmgmt。

Realtek RTL8822 上 btmgmt advertising/add-adv 会走 Extended Advertising，
控制器常返回 Disallowed/Busy，导致空中无广播。板载路径禁止调用 btmgmt advertising。
"""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from typing import Callable, List, Optional

from platform_detect import PlatformInfo, detect_platform

LogFn = Callable[[str], None]

FFE0_UUID128_LE = bytes(
    [
        0xE0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
        0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB,
    ]
)

ADV_INTERVAL_MIN = 0x0030  # ~30ms，iOS 扫描窗口短
ADV_INTERVAL_MAX = 0x0050  # ~50ms

_keeper_stop = threading.Event()
_keeper_thread: Optional[threading.Thread] = None


def build_adv_payload(name: str = "") -> bytes:
    """主包：Flags + 设备名（iOS/nRF 直接显示 HT_）。"""
    name_b = (name or "HT_BLE").encode("utf-8")[:26]
    parts = [bytes([0x02, 0x01, 0x06])]
    parts.append(bytes([len(name_b) + 1, 0x09]) + name_b)
    return b"".join(parts)[:31]


def build_scan_rsp(name: str = "") -> bytes:
    """Scan Response：128/16-bit FFE0，供 services 过滤。"""
    _ = name
    return (
        bytes([0x11, 0x07]) + FFE0_UUID128_LE + bytes([0x03, 0x03, 0xE0, 0xFF])
    )[:31]


def _run(argv: List[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        argv, capture_output=True, text=True, timeout=timeout, check=False
    )


def _hci_ok(out: str) -> bool:
    lines = [ln.strip() for ln in (out or "").splitlines() if ln.strip()]
    for i, ln in enumerate(lines):
        if "HCI Event" in ln and i + 1 < len(lines):
            toks = lines[i + 1].split()
            if len(toks) >= 4:
                return toks[3] == "00"
    return bool(re.search(r"\b00\s*$", out or ""))


def hci_le_cmd(hci_dev: str, *args: str) -> bool:
    try:
        r = _run(["hcitool", "-i", hci_dev, "cmd", "0x08", *args])
        return _hci_ok((r.stdout or "") + (r.stderr or ""))
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
    # Realtek：勿 btmgmt advertising off/on，会触发 Extended 冲突
    if plat.adv_mode != "legacy_hci":
        btmgmt(hci_dev, "advertising", "off")
    hci_le_cmd(hci_dev, "0x000a", "0x00")
    btmgmt(hci_dev, "name", name)
    _run(["hciconfig", hci_dev, "name", name])
    _run(["bluetoothctl", "system-alias", name])
    if plat.is_onboard_bt:
        _run(["hciconfig", hci_dev, "noscan"])
    time.sleep(0.2)


def stop_advertising(hci_dev: str) -> None:
    stop_adv_keeper()
    hci_le_cmd(hci_dev, "0x000a", "0x00")


def _pad31(payload: bytes) -> List[str]:
    data = (payload + bytes(31 - len(payload)))[:31]
    return [f"0x{len(payload):02x}"] + [f"0x{b:02x}" for b in data]


def _set_params(hci_dev: str) -> bool:
    lo_min, hi_min = ADV_INTERVAL_MIN & 0xFF, (ADV_INTERVAL_MIN >> 8) & 0xFF
    lo_max, hi_max = ADV_INTERVAL_MAX & 0xFF, (ADV_INTERVAL_MAX >> 8) & 0xFF
    return hci_le_cmd(
        hci_dev, "0x0006",
        f"0x{lo_min:02x}", f"0x{hi_min:02x}",
        f"0x{lo_max:02x}", f"0x{hi_max:02x}",
        "0x00", "0x00", "0x00", "0x00", "0x00", "0x00",
        "0x00", "0x00", "0x00", "0x07", "0x00",
    )


def refresh_legacy_hci(hci_dev: str, name: str) -> bool:
    hci_le_cmd(hci_dev, "0x000a", "0x00")
    hci_le_cmd(hci_dev, "0x0039", "0x00", "0x00", "0x00")
    time.sleep(0.05)
    if not _set_params(hci_dev):
        return False
    adv, sr = build_adv_payload(name), build_scan_rsp(name)
    if not hci_le_cmd(hci_dev, "0x0008", *_pad31(adv)):
        return False
    if not hci_le_cmd(hci_dev, "0x0009", *_pad31(sr)):
        return False
    return hci_le_cmd(hci_dev, "0x000a", "0x01")


def start_legacy_hci(hci_dev: str, name: str) -> bool:
    return refresh_legacy_hci(hci_dev, name)


def start_btmgmt_adv(hci_dev: str, name: str, force: bool = False) -> bool:
    idx = hci_dev.replace("hci", "")
    adv, sr = build_adv_payload(name), build_scan_rsp(name)
    if force:
        btmgmt(hci_dev, "rm-adv", "1")
        btmgmt(hci_dev, "rm-adv", "2")
    r = _run(
        ["btmgmt", "--index", idx, "add-adv", "-d", adv.hex(), "-s", sr.hex(), "-c", "1"],
        timeout=6,
    )
    if r.returncode != 0:
        return False
    r2 = _run(["btmgmt", "--index", idx, "advertising", "on"])
    out = ((r2.stdout or "") + (r2.stderr or "")).lower()
    return "failed" not in out and "busy" not in out


def stop_adv_keeper() -> None:
    global _keeper_thread
    _keeper_stop.set()
    th = _keeper_thread
    if th is not None and th.is_alive():
        th.join(timeout=2.0)
    _keeper_thread = None


def start_adv_keeper(
    hci_dev: str,
    name: str,
    platform: Optional[PlatformInfo] = None,
    interval_sec: float = 10.0,
    log: Optional[LogFn] = None,
) -> None:
    global _keeper_thread
    stop_adv_keeper()
    _keeper_stop.clear()
    plat = platform or detect_platform()

    def _loop() -> None:
        while not _keeper_stop.wait(interval_sec):
            if plat.adv_mode == "legacy_hci":
                ok = refresh_legacy_hci(hci_dev, name)
            else:
                ok = start_btmgmt_adv(hci_dev, name, force=True) or refresh_legacy_hci(
                    hci_dev, name
                )
            if log and not ok:
                log(f"[adv] 保活重发失败: {name}")

    _keeper_thread = threading.Thread(target=_loop, daemon=True, name="ble-adv-keeper")
    _keeper_thread.start()


def start_advertising(
    hci_dev: str,
    name: str,
    platform: Optional[PlatformInfo] = None,
    force: bool = False,
    log: Optional[LogFn] = None,
    keepalive: bool = True,
) -> bool:
    plat = platform or detect_platform()
    env_name = os.environ.get("BLE_DEVICE_NAME", "").strip()
    if env_name:
        name = env_name
    prepare_adapter(hci_dev, name, plat)

    if plat.adv_mode == "legacy_hci":
        ok = start_legacy_hci(hci_dev, name)
        if log:
            log(
                f"[adv] Legacy HCI 广播已开启 ({plat.hw_desc}): {name}"
                if ok else f"[adv] Legacy HCI 广播失败: {name}"
            )
        if ok and keepalive:
            start_adv_keeper(hci_dev, name, plat, log=log)
        return ok

    ok = start_btmgmt_adv(hci_dev, name, force=force)
    if not ok:
        ok = start_legacy_hci(hci_dev, name)
    if log:
        log(
            f"[adv] USB 蓝牙广播已开启 ({plat.hw_desc}): {name}"
            if ok else f"[adv] USB 蓝牙广播失败: {name}"
        )
    if ok and keepalive:
        start_adv_keeper(hci_dev, name, plat, log=log)
    return ok
