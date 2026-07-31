#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立 BLE 广播脚本（自动识别 Orin USB / RK 板载）。"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from ble_device_name import load_ble_name, parse_ble_name
from ble_legacy_adv import start_advertising, stop_advertising
from platform_detect import detect_platform


def main() -> int:
    if os.geteuid() != 0:
        print("[error] 需要 root: sudo python3 ble_advertise.py", flush=True)
        return 1

    p = argparse.ArgumentParser(description="BLE 广播外发")
    p.add_argument("--name", default=None)
    p.add_argument("--hci", default=None)
    p.add_argument("--once", action="store_true")
    args = p.parse_args()

    plat = detect_platform()
    hci = args.hci or plat.bt_hci_dev
    name = args.name or load_ble_name()
    parsed = parse_ble_name(name)
    if not parsed:
        print(f"[error] 名称须 HT_ + 8 位数字: {name!r}", flush=True)
        return 1
    name = parsed

    print(f"平台: {plat.platform_id} | {plat.hw_desc}", flush=True)
    print(f"广播: {name} @ {hci} ({plat.adv_mode})", flush=True)

    stopping = False

    def _stop(*_a) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    if not start_advertising(hci, name, plat, log=print, keepalive=not args.once):
        return 1
    if args.once:
        return 0

    print("持续广播（保活重发），Ctrl+C 停止…", flush=True)
    while not stopping:
        time.sleep(1)

    stop_advertising(hci)
    print("已停止", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
