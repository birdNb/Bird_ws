#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""装机包入口：先打遥测补丁，再启动 ble_gatt_server。"""

from __future__ import annotations

import sys


def main() -> int:
    import ble_gatt_server as gatt
    import ble_status_hooks

    ble_status_hooks.apply(gatt)
    if hasattr(gatt, "main"):
        return int(gatt.main() or 0)
    # 兼容：无 main 时回退 argparse 路径不应发生
    raise SystemExit("ble_gatt_server.main 不可用")


if __name__ == "__main__":
    sys.exit(main())
