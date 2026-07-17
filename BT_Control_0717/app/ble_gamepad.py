#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄组合键解析：文本 → 标准化组合键 → /joy_msg 模拟（非文本转发）。"""

from __future__ import annotations

import os
import re
from functools import lru_cache
from typing import Dict, Optional, Set, Tuple

# 单键（无 + ）也视为手柄指令
SINGLE_GAMEPAD_KEYS = frozenset({"a", "b", "x", "y", "lb", "rb", "back", "start", "center"})

# 组合键中的合法 token
GAMEPAD_PART_ALIASES = {
    "dpu": "dpu",
    "dpup": "dpu",
    "up": "dpu",
    "↑": "dpu",
    "dpd": "dpd",
    "dpdown": "dpd",
    "down": "dpd",
    "↓": "dpd",
    "dpl": "dpl",
    "dpleft": "dpl",
    "left": "dpl",
    "←": "dpl",
    "dpr": "dpr",
    "dpright": "dpr",
    "right": "dpr",
    "→": "dpr",
}

VALID_GAMEPAD_PARTS = SINGLE_GAMEPAD_KEYS | frozenset(
    {"lt", "rt", "dpu", "dpd", "dpl", "dpr", "↑", "↓", "←", "→"}
)

# 长按 1s（预选/系统级）
HOLD_COMBOS = frozenset({
    "lt+rt+start",
    "lt+rt+rb",
    "lt+rt+b",
    "lt+rt+lb",
})

# 短脉冲（custom_action / multi_waypoint）
PULSE_COMBOS = frozenset({
    "rt+a",
    "rt+x",
    "rt+y",
    "rt+b",
    "a",
    "x",
    "lt+rt+dpu",
    "lt+rt+dpr",
    "lt+dpr",
    "lt+dpd",
    "lt+dpl",
})

# 内置显示名（可被 custom_action.yaml 覆盖）
DEFAULT_LABELS: Dict[str, str] = {
    "lt+rt+start": "起立",
    "lt+rt+rb": "蹲下",
    "lt+rt+b": "卸力",
    "lt+rt+lb": "步态",
    "rt+a": "挥双手",
    "rt+x": "挥单手",
    "rt+y": "握手",
    "rt+b": "摇手防守",
    "a": "小脚踢球",
    "x": "秀肌肉",
    "lt+rt+dpu": "byd_bb",
    "lt+rt+dpr": "猪猪侠",
    "lt+dpr": "踢腿",
    "lt+dpd": "重拳",
    "lt+dpl": "上勾拳",
}

_KEY_LINE = re.compile(r'^\s*key:\s*"(.*?)"\s*$')
_NAME_LINE = re.compile(r'^\s*name:\s*"(.*?)"\s*$')
_REMARK_LINE = re.compile(r'^\s*remark:\s*"(.*?)"\s*$')


def _normalize_part(part: str) -> Optional[str]:
    p = part.strip().lower()
    if not p:
        return None
    if p in GAMEPAD_PART_ALIASES:
        return GAMEPAD_PART_ALIASES[p]
    if p in VALID_GAMEPAD_PARTS:
        return p
    return None


def parse_gamepad_combo(text: str) -> Optional[str]:
    """解析手柄组合键文本，返回小写规范形式如 ``rt+y``；非手柄指令返回 None。"""
    raw = text.strip()
    if not raw:
        return None
    if "+" not in raw:
        part = _normalize_part(raw)
        if part is None:
            return None
        return part
    parts: list[str] = []
    for segment in raw.split("+"):
        part = _normalize_part(segment)
        if part is None:
            return None
        parts.append(part)
    if not parts:
        return None
    return "+".join(parts)


def combo_to_wire(combo: str) -> str:
    """ACK 用的大写 wire，如 ``RT+Y``、``LT+RT+start``。"""
    out: list[str] = []
    for p in combo.split("+"):
        if p in ("start", "back", "center"):
            out.append(p)
        elif p in ("dpu", "dpd", "dpl", "dpr"):
            out.append(p.upper())
        elif len(p) <= 2:
            out.append(p.upper())
        else:
            out.append(p)
    return "+".join(out)


def is_hold_combo(combo: str) -> bool:
    return combo in HOLD_COMBOS


def is_pulse_combo(combo: str) -> bool:
    if combo in PULSE_COMBOS:
        return True
    # rt+单键 默认短脉冲（multi_waypoint）
    parts = combo.split("+")
    if len(parts) == 2 and parts[0] == "rt" and parts[1] in SINGLE_GAMEPAD_KEYS:
        return True
    # lt+rt+dpad / lt+dpad / 单键策略
    if combo in PULSE_COMBOS:
        return True
    if len(parts) == 1 and parts[0] in ("a", "x"):
        return True
    if "dpu" in parts or "dpd" in parts or "dpl" in parts or "dpr" in parts:
        return True
    return False


def _find_custom_action_yaml() -> Optional[str]:
    env = os.environ.get("CUSTOM_ACTION_YAML", "").strip()
    if env and os.path.isfile(env):
        return env
    home = os.environ.get("BIRD_HOME") or os.path.expanduser("~")
    ws = os.environ.get("SIM2REAL_WS") or os.path.join(home, "sim2real")
    roots = [
        os.path.join(ws, "install/share/sim2real/robot_config"),
        os.path.join(ws, "src/sim2real/robot_config"),
    ]
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, _, files in os.walk(root):
            if "custom_action.yaml" in files:
                return os.path.join(dirpath, "custom_action.yaml")
    return None


@lru_cache(maxsize=1)
def load_combo_labels() -> Dict[str, str]:
    """从 custom_action.yaml 加载 key→显示名；合并内置默认。"""
    labels = dict(DEFAULT_LABELS)
    path = _find_custom_action_yaml()
    if not path:
        return labels
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return labels

    current_key: Optional[str] = None
    current_name: Optional[str] = None
    current_remark: Optional[str] = None

    def _flush() -> None:
        nonlocal current_key, current_name, current_remark
        if not current_key or current_key in ("null", "none", ""):
            current_key = current_name = current_remark = None
            return
        combo = parse_gamepad_combo(current_key)
        if combo is None:
            current_key = current_name = current_remark = None
            return
        label = (current_remark or current_name or combo).strip()
        if label:
            labels[combo] = label
        current_key = current_name = current_remark = None

    for line in lines:
        m_key = _KEY_LINE.match(line)
        if m_key:
            _flush()
            current_key = m_key.group(1).strip()
            continue
        m_name = _NAME_LINE.match(line)
        if m_name:
            current_name = m_name.group(1).strip()
            continue
        m_remark = _REMARK_LINE.match(line)
        if m_remark:
            current_remark = m_remark.group(1).strip()
    _flush()
    return labels


def label_for_combo(combo: str) -> str:
    return load_combo_labels().get(combo, combo)


def known_gamepad_combos() -> Set[str]:
    """custom_action.yaml + 内置表中的全部手柄组合键。"""
    combos: Set[str] = set(DEFAULT_LABELS)
    combos |= set(HOLD_COMBOS)
    combos |= set(PULSE_COMBOS)
    path = _find_custom_action_yaml()
    if path:
        try:
            text = open(path, encoding="utf-8").read()
        except OSError:
            return combos
        for m in _KEY_LINE.finditer(text):
            key = m.group(1).strip()
            c = parse_gamepad_combo(key)
            if c:
                combos.add(c)
    return combos


def classify_gamepad_text(text: str) -> Optional[Tuple[str, str]]:
    """若文本为手柄指令，返回 (combo, wire)。"""
    combo = parse_gamepad_combo(text)
    if combo is None:
        return None
    return combo, combo_to_wire(combo)
