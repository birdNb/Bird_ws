#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据中文文案生成 conversation_bag 调用 code（前5字拼音首字母大写）。"""

from __future__ import annotations

import re
import sys

_CJK = re.compile(r"[\u4e00-\u9fff]")


def text_to_conv_code(text: str, max_chars: int = 5) -> str:
    chars = _CJK.findall(text)[:max_chars]
    if not chars:
        raise ValueError("文案中无汉字")
    try:
        from pypinyin import Style, lazy_pinyin
    except ImportError as e:
        raise ImportError("请先安装: pip3 install pypinyin") from e
    parts = lazy_pinyin("".join(chars), style=Style.FIRST_LETTER, errors="ignore")
    code = "".join(p for p in parts if p).upper()
    if not code:
        raise ValueError("无法生成拼音首字母")
    return code


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("用法: python3 make_conv_code.py \"录音完整文案\"", file=sys.stderr)
        print("示例: python3 make_conv_code.py \"蓝牙就绪待连接\"  →  LYJXD", file=sys.stderr)
        return 1
    text = " ".join(argv[1:])
    try:
        code = text_to_conv_code(text)
    except (ImportError, ValueError) as e:
        print(f"[error] {e}", file=sys.stderr)
        return 1
    print(code)
    print(f"# 文件: conversation_bag/{code}.wav", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
