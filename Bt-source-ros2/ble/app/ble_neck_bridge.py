#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脖子控制（对齐 BLE_PROTOCOL.md §1.4）：

  P{n}Y{m}  pitch/yaw 步进（整数，可带 +/-），每步 10°
  neck0     平滑回中（先同步 /joint_states 再回 0）
  P0Y0      同 neck0

方向（协议）：
  P+ 往上 / P- 往下
  Y+ 往右 / Y- 往左

ROS2：request_control(head) → /control_command；兼发 /pi_plus_absolute。
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from typing import Callable, List, Optional, Tuple

NECK_TOPIC = "/pi_plus_absolute"
CONTROL_TOPIC = "/control_command"
REQUEST_CONTROL_SRV = "/request_control"
RELEASE_CONTROL_SRV = "/release_control"
HEAD_YAW_JOINT = "head_yaw_joint"
HEAD_PITCH_JOINT = "head_pitch_joint"
FALLBACK_MOTOR_IDS = (20, 21)
NECK_STEP_DEG = 10.0
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0
PITCH_DOWN_DEG = 60.0
HOME_RATE_DEG_PER_SEC = 60.0
TICK_HZ = 50.0
NECK_STATE_FILE = "/tmp/locate_face_neck.state"
HEAD_KP = 6.0
HEAD_KD = 0.6
SESSION_TIMEOUT_MS = 1500
# midware 超时后仍带着旧 uuid 发指令会被静默丢弃；超过该空闲须重新 request_control
SESSION_STALE_SEC = 0.7
HOLD_PUBLISH_HZ = 50.0
HOLD_AFTER_CMD_SEC = 1.5
HOLD_AFTER_HOME_SEC = 2.0
# hold 结束后释放头电机，避免挡住量产控制器全身控制（起立/行走）
RELEASE_AFTER_HOLD = True

NECK_OFFSET_RE = re.compile(r"^[Pp]([+-]?\d+)[Yy]([+-]?\d+)$")
NECK_CENTER_RE = re.compile(r"^neck0$", re.IGNORECASE)

LogFn = Callable[[str], None]


def parse_neck_command(text: str) -> Optional[Tuple[str, int, int]]:
    raw = text.strip()
    if NECK_CENTER_RE.match(raw):
        return ("neck0", 0, 0)
    m = NECK_OFFSET_RE.match(raw)
    if not m:
        return None
    p_steps = int(m.group(1))
    y_steps = int(m.group(2))
    return (f"P{p_steps}Y{y_steps}", p_steps, y_steps)


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _clamp_pitch_deg(deg: float) -> float:
    return max(PITCH_UP_DEG, min(PITCH_DOWN_DEG, deg))


def _clamp_yaw_deg(deg: float) -> float:
    return max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, deg))


def _step_toward_zero(val: float, step: float) -> float:
    if abs(val) <= 1e-4:
        return 0.0
    return val - math.copysign(min(step, abs(val)), val)


class NeckController:
    """协议步进 → /control_command + /pi_plus_absolute。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._fb_yaw_deg: Optional[float] = None
        self._fb_pitch_deg: Optional[float] = None
        self._abs_pub = None
        self._cmd_pub = None
        self._clock = None
        self._node = None
        self._req_cli = None
        self._rel_cli = None
        self._MotorControlCommand = None
        self._RequestControl = None
        self._ReleaseControl = None
        self._pending: Optional[str] = None
        self._homing = False
        self._uuid: Optional[str] = None
        self._motor_ids: List[int] = list(FALLBACK_MOTOR_IDS)
        self._last_hold_pub = 0.0
        self._last_req_attempt = 0.0
        self._last_midware_ok = 0.0
        self._session_ok = False
        self._hold_until = 0.0
        self._was_holding = False
        self._load_state()

    def _load_state(self) -> None:
        try:
            with open(NECK_STATE_FILE, "r", encoding="ascii") as f:
                parts = f.read().strip().split()
            if len(parts) >= 2:
                self._yaw_deg = _clamp_yaw_deg(float(parts[0]))
                self._pitch_deg = _clamp_pitch_deg(float(parts[1]))
        except (OSError, ValueError):
            pass

    def _save_state(self, yaw_deg: float, pitch_deg: float) -> None:
        try:
            tmp = f"{NECK_STATE_FILE}.tmp"
            with open(tmp, "w", encoding="ascii") as f:
                f.write(f"{yaw_deg:.4f} {pitch_deg:.4f}\n")
            os.replace(tmp, NECK_STATE_FILE)
        except OSError:
            pass

    def attach(self, node, abs_pub=None, clock=None) -> None:
        self._node = node
        self._clock = clock or (node.get_clock() if node is not None else None)
        self._abs_pub = abs_pub
        try:
            from hightorque_msgs.msg import MotorControlCommand
            from hightorque_msgs.srv import ReleaseControl, RequestControl
            from sensor_msgs.msg import JointState

            self._MotorControlCommand = MotorControlCommand
            self._RequestControl = RequestControl
            self._ReleaseControl = ReleaseControl
            self._cmd_pub = node.create_publisher(MotorControlCommand, CONTROL_TOPIC, 10)
            self._req_cli = node.create_client(RequestControl, REQUEST_CONTROL_SRV)
            self._rel_cli = node.create_client(ReleaseControl, RELEASE_CONTROL_SRV)
            node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
            self._log(
                f"[neck] ROS2 路径: {REQUEST_CONTROL_SRV} + {CONTROL_TOPIC} "
                f"(兼容 {NECK_TOPIC})；步进 {NECK_STEP_DEG:.0f}°；neck0 先同步实机再回中"
            )
        except Exception as e:
            self._log(f"[neck][warn] midware 接口不可用，仅发 {NECK_TOPIC}: {e}")

    def attach_publisher(self, pub, clock=None) -> None:
        self._abs_pub = pub
        if clock is not None:
            self._clock = clock

    def _on_joint_states(self, msg) -> None:
        names = list(msg.name or [])
        pos = list(msg.position or [])
        if not names or len(pos) < len(names):
            return
        try:
            iy = names.index(HEAD_YAW_JOINT)
            ip = names.index(HEAD_PITCH_JOINT)
        except ValueError:
            return
        self._motor_ids = [iy, ip]
        with self._lock:
            self._fb_yaw_deg = _clamp_yaw_deg(_rad2deg(float(pos[iy])))
            self._fb_pitch_deg = _clamp_pitch_deg(_rad2deg(float(pos[ip])))

    def _sync_from_feedback(self) -> Tuple[float, float]:
        """用 /joint_states 校正内部目标，避免软件角与实机脱节导致 neck0 空转。"""
        with self._lock:
            if self._fb_yaw_deg is not None and self._fb_pitch_deg is not None:
                self._yaw_deg = self._fb_yaw_deg
                self._pitch_deg = self._fb_pitch_deg
            return self._yaw_deg, self._pitch_deg

    def enqueue(self, text: str) -> bool:
        if parse_neck_command(text) is None:
            return False
        with self._lock:
            self._pending = text.strip()
        return True

    def tick(self) -> None:
        with self._lock:
            text = self._pending
            self._pending = None
        if text is not None:
            self.handle(text)
        self._tick_homing()
        now = time.monotonic()
        with self._lock:
            active = self._homing or now < self._hold_until
            yaw, pitch = self._yaw_deg, self._pitch_deg
        if active and self._cmd_pub is not None:
            if now - self._last_hold_pub >= (1.0 / HOLD_PUBLISH_HZ):
                self._publish_midware(yaw, pitch)
                self._last_hold_pub = now
            self._was_holding = True
        elif self._was_holding and RELEASE_AFTER_HOLD:
            self._was_holding = False
            self.release_control(reason="hold 结束")

    def release_control(self, reason: str = "") -> None:
        """释放头电机控制权，供起立/行走等全身动作使用。"""
        with self._lock:
            self._homing = False
            self._hold_until = 0.0
            uuid = self._uuid
        if uuid and self._rel_cli is not None and self._ReleaseControl is not None:
            try:
                if self._rel_cli.service_is_ready():
                    req = self._ReleaseControl.Request()
                    req.node_name = "ble_neck"
                    req.uuid = str(uuid)
                    req.release_mode = self._ReleaseControl.Request.KEEP_MODE
                    fut = self._rel_cli.call_async(req)
                    t0 = time.monotonic()
                    while not fut.done() and time.monotonic() - t0 < 0.4:
                        time.sleep(0.02)
            except Exception as e:
                self._log(f"[neck][warn] release_control 异常: {e}")
        self._invalidate_session()
        self._was_holding = False
        if reason:
            self._log(f"[neck] 已释放头电机控制权（{reason}）")

    def handle(self, text: str) -> bool:
        parsed = parse_neck_command(text)
        if parsed is None:
            return False
        if self._abs_pub is None and self._cmd_pub is None:
            self._log("[neck] 未就绪，忽略指令")
            return False
        wire, p_steps, y_steps = parsed

        if wire == "neck0" or (p_steps == 0 and y_steps == 0):
            yaw0, pitch0 = self._sync_from_feedback()
            travel = max(abs(yaw0), abs(pitch0))
            need = travel / max(HOME_RATE_DEG_PER_SEC, 1.0) + 2.0
            with self._lock:
                self._hold_until = time.monotonic() + max(HOLD_AFTER_HOME_SEC, need)
                self._homing = True
            # 空闲后 uuid 易过期；回中前强制重新要权，否则平滑指令被 midware 丢弃
            self._invalidate_session()
            if not self._ensure_session():
                self._log("[neck][warn] 回中未能取得头电机控制权，仍尝试下发")
            self._publish(yaw0, pitch0)
            self._log(
                f"[neck] 回中 {wire} 从实机 yaw={yaw0:+.1f}° pitch={pitch0:+.1f}° "
                f"→ 0（平滑 {HOME_RATE_DEG_PER_SEC:.0f}°/s） "
                f"session={'ok' if self._session_ok else 'pending'}"
            )
            return True

        with self._lock:
            self._homing = False
            self._hold_until = time.monotonic() + HOLD_AFTER_CMD_SEC
            self._pitch_deg = _clamp_pitch_deg(
                self._pitch_deg - p_steps * NECK_STEP_DEG
            )
            self._yaw_deg = _clamp_yaw_deg(self._yaw_deg - y_steps * NECK_STEP_DEG)
            yaw, pitch = self._yaw_deg, self._pitch_deg
        self._publish(yaw, pitch)
        self._log(
            f"[neck] {wire} → pitch={pitch:+.1f}° yaw={yaw:+.1f}° "
            f"(P{p_steps:+d} Y{y_steps:+d} ×{NECK_STEP_DEG:.0f}°) "
            f"motors={self._motor_ids} session={'ok' if self._session_ok else 'pending'}"
        )
        return True

    def _tick_homing(self) -> None:
        with self._lock:
            if not self._homing:
                return
            step = HOME_RATE_DEG_PER_SEC / TICK_HZ
            self._yaw_deg = _step_toward_zero(self._yaw_deg, step)
            self._pitch_deg = _step_toward_zero(self._pitch_deg, step)
            yaw, pitch = self._yaw_deg, self._pitch_deg
            done = abs(yaw) < 1e-4 and abs(pitch) < 1e-4
            if done:
                self._yaw_deg = 0.0
                self._pitch_deg = 0.0
                self._homing = False
                yaw, pitch = 0.0, 0.0
                self._hold_until = max(self._hold_until, time.monotonic() + 1.0)
        self._publish(yaw, pitch)
        if done:
            self._log("[neck] 回中完成 yaw=0 pitch=0")

    def _invalidate_session(self) -> None:
        self._session_ok = False
        self._uuid = None
        self._last_req_attempt = 0.0

    def _ensure_session(self) -> bool:
        now = time.monotonic()
        if (
            self._uuid
            and self._session_ok
            and (now - self._last_midware_ok) < SESSION_STALE_SEC
        ):
            return True
        # 空闲过久：旧 uuid 对 midware 已失效，必须重新申请
        if self._uuid and (now - self._last_midware_ok) >= SESSION_STALE_SEC:
            self._session_ok = False
        if self._req_cli is None or self._RequestControl is None or self._node is None:
            return False
        if now - self._last_req_attempt < 0.35 and self._uuid and self._session_ok:
            return True
        if now - self._last_req_attempt < 0.35:
            return bool(self._uuid and self._session_ok)
        self._last_req_attempt = now
        if not self._req_cli.service_is_ready():
            self._log("[neck][warn] /request_control 不可用（midware 未起？）")
            self._session_ok = False
            return False
        req = self._RequestControl.Request()
        req.node_name = "ble_neck"
        req.motor_ids = [int(x) for x in self._motor_ids]
        req.control_mode = self._RequestControl.Request.POSITION_MODE
        req.default_behavior = 0
        req.timeout_ms = SESSION_TIMEOUT_MS
        req.default_kp = [HEAD_KP, HEAD_KP]
        req.default_kd = [HEAD_KD, HEAD_KD]
        try:
            future = self._req_cli.call_async(req)
            t0 = time.monotonic()
            while not future.done() and time.monotonic() - t0 < 0.8:
                time.sleep(0.02)
            if not future.done():
                self._log("[neck][warn] request_control 超时")
                return False
            resp = future.result()
            if resp is None or not resp.success:
                msg = getattr(resp, "message", "no response")
                self._log(f"[neck][warn] request_control 失败: {msg}")
                self._session_ok = False
                self._uuid = None
                return False
            self._uuid = str(resp.uuid)
            self._session_ok = True
            self._last_midware_ok = time.monotonic()
            self._log(
                f"[neck] 已获得头电机控制权 uuid={self._uuid[:8]}… "
                f"ids={self._motor_ids}"
            )
            return True
        except Exception as e:
            self._log(f"[neck][warn] request_control 异常: {e}")
            self._session_ok = False
            return False

    def _publish(self, yaw_deg: float, pitch_deg: float) -> None:
        self._publish_absolute(yaw_deg, pitch_deg)
        self._publish_midware(yaw_deg, pitch_deg)
        self._save_state(yaw_deg, pitch_deg)

    def _publish_absolute(self, yaw_deg: float, pitch_deg: float) -> None:
        if self._abs_pub is None:
            return
        from sensor_msgs.msg import JointState
        from builtin_interfaces.msg import Time

        msg = JointState()
        if self._clock is not None:
            msg.header.stamp = self._clock.now().to_msg()
        else:
            msg.header.stamp = Time()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [_deg2rad(yaw_deg), _deg2rad(pitch_deg)]
        try:
            self._abs_pub.publish(msg)
        except Exception:
            pass

    def _publish_midware(self, yaw_deg: float, pitch_deg: float) -> None:
        if self._cmd_pub is None or self._MotorControlCommand is None:
            return
        if not self._ensure_session() or not self._uuid:
            return
        msg = self._MotorControlCommand()
        msg.uuid = self._uuid
        msg.motor_ids = [int(x) for x in self._motor_ids]
        msg.positions = [float(_deg2rad(yaw_deg)), float(_deg2rad(pitch_deg))]
        msg.velocities = [0.0, 0.0]
        msg.kp = [HEAD_KP, HEAD_KP]
        msg.kd = [HEAD_KD, HEAD_KD]
        msg.torques = [0.0, 0.0]
        try:
            self._cmd_pub.publish(msg)
            self._session_ok = True
            self._last_midware_ok = time.monotonic()
        except Exception as e:
            self._log(f"[neck][warn] 发布 control_command 失败: {e}")
            self._session_ok = False
