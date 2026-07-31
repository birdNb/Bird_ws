#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运行时补丁：在装机包 ble_gatt_server.pyc 上挂接功能状态遥测。

不改二进制主体；由 ble_gatt_boot 在 main() 前 apply。
"""

from __future__ import annotations

from typing import Any, Callable, Optional


def _wrap(cls: type, name: str, maker: Callable[[Any], Any]) -> None:
    orig = getattr(cls, name, None)
    if orig is None:
        return
    setattr(cls, name, maker(orig))


def apply(gatt_module: Any) -> None:
    cls = getattr(gatt_module, "BleGattServer", None)
    if cls is None:
        return

    # ---- locate_face / sound / pull：指令成功后推遥测 ----
    orig_handle = cls._handle_dispatched

    def _handle_dispatched(self, kind, payload: str) -> None:
        orig_handle(self, kind, payload)
        tel = getattr(self, "_telemetry", None)
        if tel is None:
            return
        try:
            kv = getattr(kind, "value", str(kind))
            action = (payload or "").strip().upper()
            if kv == "locate_face" and action in ("ON", "OFF"):
                # 以进程实测为准，避免启动失败仍报 ON
                from ble_status_telemetry import detect_locate_face_on

                tel.push_feature("locate_face", detect_locate_face_on(), force=True)
            elif kv == "sound" and action in ("ON", "OFF"):
                tel.push_feature("sound", action, force=True)
            elif kv == "pull" and action in ("ON", "OFF"):
                from ble_status_telemetry import detect_pull_on

                tel.push_feature("pull", detect_pull_on(), force=True)
            elif kv == "sprint" and action in ("ON", "OFF"):
                tel.push_feature("sprint", action, force=True)
        except Exception:
            pass

    cls._handle_dispatched = _handle_dispatched

    # ---- 步态 / 疾跑：在原有语音提示上叠加遥测 ----
    orig_gait = getattr(cls, "_on_gait_state", None)
    if orig_gait is not None:

        def _on_gait_state(self, state: str) -> None:
            orig_gait(self, state)
            tel = getattr(self, "_telemetry", None)
            if tel is not None and state in ("ON", "OFF"):
                try:
                    tel.push_feature("gait", state, force=True)
                except Exception:
                    pass

        cls._on_gait_state = _on_gait_state

    orig_sprint = getattr(cls, "_on_ros_sprint", None)
    if orig_sprint is not None:

        def _on_ros_sprint(self, enabled: bool) -> None:
            orig_sprint(self, enabled)
            tel = getattr(self, "_telemetry", None)
            if tel is not None:
                try:
                    tel.push_feature("sprint", "ON" if enabled else "OFF", force=True)
                except Exception:
                    pass

        cls._on_ros_sprint = _on_ros_sprint

    # ---- 订阅时：注册读取器（sound/sprint 从板端缓存）----
    # sound / sprint 状态缓存挂在 server 实例上
    _orig_init = cls.__init__

    def __init__(self, *args, **kwargs):
        _orig_init(self, *args, **kwargs)
        # 默认：语音开，人脸/拖拽关（后两者由进程/服务实测）
        self._feat_sound_on = True
        self._feat_sprint_on = False

    cls.__init__ = __init__

    # 在 sound 分支更新缓存（通过再包一层 handle；上面已包，这里用独立 wrap sound 成功路径）
    # 简化：再包一次 handle 更新缓存
    _handle2 = cls._handle_dispatched

    def _handle_with_cache(self, kind, payload: str) -> None:
        try:
            kv = getattr(kind, "value", str(kind))
            action = (payload or "").strip().upper()
            if kv == "sound" and action in ("ON", "OFF"):
                self._feat_sound_on = action == "ON"
            elif kv == "sprint" and action in ("ON", "OFF"):
                self._feat_sprint_on = action == "ON"
            elif kv == "pull" and action in ("ON", "OFF"):
                pass  # 由 detect_pull_on 实测
        except Exception:
            pass
        _handle2(self, kind, payload)

    cls._handle_dispatched = _handle_with_cache

    _sprint2 = getattr(cls, "_on_ros_sprint", None)
    if _sprint2 is not None:

        def _on_ros_sprint_cache(self, enabled: bool) -> None:
            self._feat_sprint_on = bool(enabled)
            _sprint2(self, enabled)

        cls._on_ros_sprint = _on_ros_sprint_cache

    orig_wire = getattr(cls, "_wire_mp_telemetry", None)
    if orig_wire is not None:

        def _wire_mp_telemetry(self) -> None:
            orig_wire(self)
            tel = getattr(self, "_telemetry", None)
            if tel is None:
                return
            try:
                def _read_sound() -> str:
                    vr = getattr(self, "_voice_remind", None)
                    if vr is not None:
                        try:
                            return "ON" if bool(vr.prompts_enabled) else "OFF"
                        except Exception:
                            pass
                    return "ON" if getattr(self, "_feat_sound_on", True) else "OFF"

                tel.set_feature_reader("sound", _read_sound)
                tel.set_feature_reader(
                    "sprint",
                    lambda: "ON" if getattr(self, "_feat_sprint_on", False) else "OFF",
                )
                rb = getattr(self, "_ros_bridge", None)
                if rb is not None and hasattr(rb, "_sprint_enabled"):
                    tel.set_feature_reader(
                        "sprint",
                        lambda b=rb: "ON"
                        if getattr(b, "_sprint_enabled", False)
                        else "OFF",
                    )
                from ble_status_telemetry import detect_locate_face_on, detect_pull_on

                tel.set_feature_reader("locate_face", detect_locate_face_on)
                tel.set_feature_reader("pull", detect_pull_on)
            except Exception:
                pass

        cls._wire_mp_telemetry = _wire_mp_telemetry

    # pull 成功后也推（_set_pull 在 pyc 内）
    orig_set_pull = getattr(cls, "_set_pull", None)
    if orig_set_pull is not None:

        def _set_pull(self, enable: bool, voice: bool = True) -> None:
            orig_set_pull(self, enable, voice=voice)
            tel = getattr(self, "_telemetry", None)
            if tel is not None:
                try:
                    from ble_status_telemetry import detect_pull_on

                    tel.push_feature("pull", detect_pull_on(), force=True)
                except Exception:
                    pass

        cls._set_pull = _set_pull
