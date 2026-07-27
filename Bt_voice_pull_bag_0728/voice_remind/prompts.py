#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固定语音提示定义（assets/ 英文文件名，非首字母）。"""

from __future__ import annotations

from typing import Dict, Tuple

# (文件名, 朗读文本) — 文案与 voice_bag/notice_bag_baby 一致
PROMPTS: Dict[str, Tuple[str, str]] = {
    "ble_ready": ("ble_ready.wav", "蓝牙就绪，待连接"),
    "ble_connected": ("ble_connected.wav", "蓝牙已连接"),
    "ble_disconnected": ("ble_disconnected.wav", "蓝牙已断开"),
    "auto_stand": ("auto_stand.wav", "自动站立"),
    "motor_on": ("motor_on.wav", "电机电源已开启"),
    "motor_off": ("motor_off.wav", "电机电源已关闭"),
    "walk_mode": ("walk_mode.wav", "行走模式"),
    "stand_mode": ("stand_mode.wav", "站立模式"),
    "pull_on": ("pull_on.wav", "拖拽模式已开启"),
    "pull_off": ("pull_off.wav", "拖拽模式已关闭"),
    "sprint_on": ("sprint_on.wav", "疾跑模式已开启"),
    "sprint_off": ("sprint_off.wav", "疾跑模式已关闭"),
    "mode_default": ("mode_default.wav", "默认模式"),
    "mode_init": ("mode_init.wav", "初始化模式"),
    "mode_protect": ("mode_protect.wav", "保护模式"),
    "mode_resetzero": ("mode_resetzero.wav", "调零模式"),
    "mode_tech": ("mode_tech.wav", "示教模式"),
    "squat": ("squat.wav", "蹲下"),
    "face_on": ("face_on.wav", "人脸跟随已开启"),
    "face_off": ("face_off.wav", "人脸跟随已关闭"),
    "sound_on": ("sound_on.wav", "提示音已开启"),
    "sound_off": ("sound_off.wav", "提示音已关闭"),
    "battery_50": ("battery_50.wav", "当前电量百分之五十"),
    "battery_25": ("battery_25.wav", "当前电量百分之二十五"),
    "battery_10": ("battery_10.wav", "当前电量百分之十"),
    "battery_5": ("battery_5.wav", "当前电量百分之五，请立即更换电池"),
}

BATTERY_VOICE_THRESHOLDS = (50, 25, 10, 5)
