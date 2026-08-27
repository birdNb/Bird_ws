#!/usr/bin/env python3
"""右手 4 轴寻硬限位（兼容入口）。"""
import sys

from cw_limit import main

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--group" not in argv:
        argv = ["--group", "right_arm", *argv]
    sys.exit(main(argv))
