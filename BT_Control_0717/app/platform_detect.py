#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动识别主控平台与蓝牙硬件类型（Orin USB / RK 板载）。"""

from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from functools import lru_cache
from typing import Optional


@dataclass(frozen=True)
class PlatformInfo:
    platform_id: str
    board_name: str
    bt_kind: str
    bt_chip: str
    bt_hci_dev: str
    adv_mode: str
    fw_wait_sec: float
    hw_desc: str

    @property
    def is_orin(self) -> bool:
        return self.platform_id == "orin"

    @property
    def is_rk(self) -> bool:
        return self.platform_id.startswith("rk")

    @property
    def is_onboard_bt(self) -> bool:
        return self.bt_kind == "onboard_combo"

    @property
    def is_usb_bt(self) -> bool:
        return self.bt_kind == "usb_dongle"


def _read_model() -> str:
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return f.read().decode("utf-8", errors="replace").strip("\x00").strip()
    except OSError:
        return ""


def _lsmod_has(mod: str) -> bool:
    try:
        r = subprocess.run(
            ["lsmod"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        return mod in (r.stdout or "")
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return False


def _env_override() -> Optional[PlatformInfo]:
    pid = os.environ.get("BLE_PLATFORM", "").strip().lower()
    if not pid:
        return None
    if pid in ("orin", "jetson"):
        return PlatformInfo(
            platform_id="orin",
            board_name=os.environ.get("BLE_BOARD_NAME", "Jetson Orin"),
            bt_kind="usb_dongle",
            bt_chip=os.environ.get("BLE_CHIP", "USB"),
            bt_hci_dev=os.environ.get("BLE_HCI_DEV", "hci0"),
            adv_mode="btmgmt",
            fw_wait_sec=float(os.environ.get("BLE_FW_WAIT_SEC", "2")),
            hw_desc=os.environ.get("BLE_HW_DESC", "外接 USB 蓝牙模块"),
        )
    if pid.startswith("rk"):
        return PlatformInfo(
            platform_id="rk3588s",
            board_name=os.environ.get("BLE_BOARD_NAME", "RK3588"),
            bt_kind="onboard_combo",
            bt_chip=os.environ.get("BLE_CHIP", "RTL8822CE"),
            bt_hci_dev=os.environ.get("BLE_HCI_DEV", "hci0"),
            adv_mode="legacy_hci",
            fw_wait_sec=float(os.environ.get("BLE_FW_WAIT_SEC", "6")),
            hw_desc=os.environ.get(
                "BLE_HW_DESC", "板载 RTL8822CE WiFi+蓝牙一体网卡"
            ),
        )
    return None


@lru_cache(maxsize=1)
def detect_platform() -> PlatformInfo:
    """检测当前主控：Orin → USB 蓝牙；RK3588 → 板载一体网卡蓝牙。"""
    override = _env_override()
    if override is not None:
        return override

    model = _read_model()
    low = model.lower()

    if re.search(r"orin|jetson|nvidia|tegra", low):
        return PlatformInfo(
            platform_id="orin",
            board_name=model or "Jetson Orin",
            bt_kind="usb_dongle",
            bt_chip="USB",
            bt_hci_dev="hci0",
            adv_mode="btmgmt",
            fw_wait_sec=2.0,
            hw_desc="外接 USB 蓝牙模块",
        )

    if re.search(r"rk3588|lubancat|embedfire|rockchip", low) or _lsmod_has(
        "rtw88_8822ce"
    ):
        return PlatformInfo(
            platform_id="rk3588s",
            board_name=model or "RK3588s",
            bt_kind="onboard_combo",
            bt_chip="RTL8822CE",
            bt_hci_dev="hci0",
            adv_mode="legacy_hci",
            fw_wait_sec=6.0,
            hw_desc="板载 RTL8822CE WiFi+蓝牙一体网卡（非外插 USB 棒）",
        )

    return PlatformInfo(
        platform_id="unknown",
        board_name=model or "unknown",
        bt_kind="usb_dongle",
        bt_chip="unknown",
        bt_hci_dev="hci0",
        adv_mode="btmgmt",
        fw_wait_sec=3.0,
        hw_desc="蓝牙适配器 hci0",
    )
