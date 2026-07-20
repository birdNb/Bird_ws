#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 voice_bag/ 中的录音导入 conversation_bag/，按文案前5字拼音首字母大写命名。

源文件命名方式（任选其一）：
  1. 文件名即中文文案：蓝牙就绪待连接.wav
  2. voice_bag/manifest.json 指定 text + file
  3. 同名 .txt  sidecar：录音1.mp3 + 录音1.txt（txt 内为完整文案）

用法：
  python3 import_voice_bag.py
  python3 import_voice_bag.py /path/to/voice_bag
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from make_conv_code import text_to_conv_code  # noqa: E402

DEFAULT_SRC = os.path.join(os.path.dirname(_DIR), "voice_bag")
DEST = os.path.join(_DIR, "conversation_bag")
AUDIO_EXT = {".wav", ".mp3", ".m4a", ".flac", ".ogg", ".aac"}
_CJK = re.compile(r"[\u4e00-\u9fff]")
_CODE_STEM = re.compile(r"^[A-Z][A-Z0-9]{1,31}$")


def _normalize_code(code: str) -> str:
    return code.strip().upper()


def _has_cjk(text: str) -> bool:
    return bool(_CJK.search(text))


def _load_src_manifest(src: str) -> dict[str, dict]:
    path = os.path.join(src, "manifest.json")
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    out: dict[str, dict] = {}
    items = data.get("items")
    if isinstance(items, dict):
        for k, v in items.items():
            if isinstance(v, dict) and v.get("file"):
                out[str(v["file"])] = v
            elif isinstance(v, str):
                out[str(k)] = {"text": v, "file": k}
    elif isinstance(items, list):
        for it in items:
            if isinstance(it, dict) and it.get("file"):
                out[str(it["file"])] = it
    return out


def _code_for_file(src: str, fname: str, meta: dict) -> tuple[str, str]:
    """返回 (code, text)。"""
    stem, _ = os.path.splitext(fname)
    if meta.get("code"):
        code = _normalize_code(str(meta["code"]))
        text = str(meta.get("text") or code)
        return code, text
    if meta.get("text"):
        text = str(meta["text"]).strip()
        return text_to_conv_code(text), text
    if _has_cjk(stem):
        text = stem.strip()
        return text_to_conv_code(text), text
    txt_path = os.path.join(src, f"{stem}.txt")
    if os.path.isfile(txt_path):
        with open(txt_path, encoding="utf-8") as f:
            text = f.read().strip()
        return text_to_conv_code(text), text
    # 文件名已是首字母 code（如 BBMWX.mp3）
    code = _normalize_code(stem)
    if _CODE_STEM.match(code):
        return code, code
    raise ValueError(f"无法确定 code: {fname}（中文文件名 / manifest / 同名.txt / 大写code文件名）")


def _to_wav(src_path: str, dst_path: str) -> None:
    ext = os.path.splitext(src_path)[1].lower()
    if ext == ".wav":
        shutil.copy2(src_path, dst_path)
        return
    if shutil.which("ffmpeg") is None:
        raise RuntimeError(f"需要 ffmpeg 转换 {ext} → wav")
    subprocess.run(
        ["ffmpeg", "-y", "-i", src_path, "-ar", "16000", "-ac", "1", dst_path],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def import_bag(src: str = DEFAULT_SRC, dest: str = DEST) -> int:
    if not os.path.isdir(src):
        print(f"[error] 源目录不存在: {src}", file=sys.stderr)
        print("请先将 Windows 上 voice_bag 复制到板子，例如：", file=sys.stderr)
        print(f"  scp -r d:/HT_File/Bird_ws/voice_bag hightorque@<板子IP>:{os.path.dirname(_DIR)}/", file=sys.stderr)
        return 1

    meta_map = _load_src_manifest(src)
    files = sorted(
        f
        for f in os.listdir(src)
        if os.path.splitext(f)[1].lower() in AUDIO_EXT
    )
    if not files:
        print(f"[error] {src} 内无音频文件", file=sys.stderr)
        return 1

    os.makedirs(dest, exist_ok=True)
    manifest_path = os.path.join(dest, "manifest.json")
    if os.path.isfile(manifest_path):
        with open(manifest_path, encoding="utf-8") as f:
            manifest = json.load(f)
    else:
        manifest = {"version": 2, "comment": "前5字拼音首字母大写", "items": {}}
    items: dict = manifest.setdefault("items", {})

    ok = 0
    for fname in files:
        meta = meta_map.get(fname, {})
        try:
            code, text = _code_for_file(src, fname, meta)
        except (ImportError, ValueError, RuntimeError) as e:
            print(f"[skip] {fname}: {e}", file=sys.stderr)
            continue

        dst_wav = os.path.join(dest, f"{code}.wav")
        src_path = os.path.join(src, fname)
        try:
            _to_wav(src_path, dst_wav)
        except (OSError, subprocess.CalledProcessError, RuntimeError) as e:
            print(f"[skip] {fname}: 转换失败 {e}", file=sys.stderr)
            continue

        items[code] = {"text": text, "file": f"{code}.wav"}
        print(f"[ok] {fname} → {code}.wav  ({text})")
        ok += 1

    manifest["items"] = items
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print(f"[done] 导入 {ok}/{len(files)} → {dest}")
    return 0 if ok else 1


def main(argv: list[str]) -> int:
    src = argv[1] if len(argv) > 1 else DEFAULT_SRC
    return import_bag(src)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
