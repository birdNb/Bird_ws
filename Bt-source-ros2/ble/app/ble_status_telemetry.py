#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人 → 小程序 状态遥测（FFE2 notify）。

  IP:192.168.19.11   局域网 IPv4 四段完整地址
  pwr:83             电量 0~100；握手后立即发送，下降时再推
  mp:ON/OFF          电机电源；订阅后推送 1 次，变化时再推 1 次
  fsm:5              底层 FSM 数值；订阅/变化时连发 2 次
  mode:M_default     模式文本；订阅/变化时推送
  locate_face ON/OFF 人脸追踪
  GAIT ON/OFF        行走/站立（OFF=站立；可由 default_bt current_state 推断）
  PULL ON/OFF        拖拽模式
  sound ON/OFF       语音开关
  LT ON/OFF          疾跑开关
"""

from __future__ import annotations

import glob
import os
import socket
import subprocess
import sys
import threading
import time
from typing import Callable, Dict, Optional

MotorPowerFn = Callable[[], Optional[str]]
BatteryFn = Callable[[], Optional[int]]
FeatureFn = Callable[[], Optional[str]]
BatteryListener = Callable[[int], None]
LanIpListener = Callable[[str], None]

from ble_log import log_info, log_tx, log_warn

IP_POLL_SEC = 15.0
PWR_POLL_SEC = 5.0
FSM_REPEAT = 2
FSM_REPEAT_GAP_SEC = 0.05
FEATURE_REPEAT = 1
CONTROLLER_STATE_TOPICS = (
    "/hightorque_controller/state",
    "/hightorque_controller/controller_state",
)
PWR_RETRY_INTERVAL_SEC = 0.2
PWR_RETRY_MAX_SEC = 8.0
# 机器人实际发布电量的话题（优先 /battery_level UInt8）
BATTERY_FLOAT_TOPICS = ("/battery_percent", "/battery")
BATTERY_INT_TOPICS = ()

# 功能开关：FFE2 明文（与上行指令同形，便于小程序复用解析）
FEATURE_WIRES = {
    "locate_face": "locate_face",
    "gait": "GAIT",
    "pull": "PULL",
    "sound": "sound",
    "sprint": "LT",
}

FSM_TO_MODE_WIRE = {
    0: "M_init",
    1: "M_init",
    2: "M_init",
    5: "M_default",
    8: "M_protect",
    9: "M_resetzero",
    10: "M_resetzero",
    11: "M_resetzero",
    12: "M_resetzero",
}
DEFAULT_FSM = 5
DEFAULT_MODE_WIRE = "M_default"

NotifyFn = Callable[[bytes], None]


def _bootstrap_ros_python_path() -> None:
    home = os.environ.get("BIRD_HOME") or os.path.expanduser("~")
    ws = os.environ.get("COLCON_WS") or os.path.join(home, "colcon_ws")
    extra = [
        "/opt/ros/foxy/lib/python3.8/site-packages",
        "/opt/ros/foxy/local/lib/python3.8/dist-packages",
    ]
    if os.path.isdir(os.path.join(ws, "install")):
        for name in os.listdir(os.path.join(ws, "install")):
            site = os.path.join(ws, "install", name, "lib", "python3.8", "site-packages")
            if os.path.isdir(site):
                extra.append(site)
    for p in extra:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


def read_lan_ip() -> Optional[str]:
    """返回局域网 IPv4 四段，如 192.168.19.11。"""
    candidates = []
    try:
        out = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        for item in out:
            ip = item[4][0]
            if not ip.startswith("127.") and ip.count(".") == 3:
                candidates.append(ip)
    except OSError:
        pass

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            candidates.insert(0, ip)
    except OSError:
        pass

    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4 and (parts[0] == "192" or parts[0] == "10"):
            return ip
    for ip in candidates:
        parts = ip.split(".")
        if len(parts) == 4:
            return ip
    return None


# 兼容旧调用名
read_lan_ip_suffix = read_lan_ip


def read_battery_percent() -> Optional[int]:
    for path in sorted(glob.glob("/sys/class/power_supply/*/capacity")):
        try:
            with open(path, "r", encoding="ascii") as f:
                val = int(f.read().strip())
            if 0 <= val <= 100:
                return val
        except (OSError, ValueError):
            continue
    return None


def _normalize_pwr(pct: int) -> int:
    return max(0, min(100, int(pct)))


def _format_pwr(pct: int) -> str:
    """实际电量，如 pwr:83"""
    return f"pwr:{_normalize_pwr(pct)}"


def _normalize_on_off(wire: Optional[str]) -> Optional[str]:
    if wire is None:
        return None
    w = str(wire).strip().upper()
    if w in ("ON", "OFF"):
        return w
    if w in ("1", "TRUE", "YES"):
        return "ON"
    if w in ("0", "FALSE", "NO"):
        return "OFF"
    # 允许传入完整报文 "GAIT ON"
    parts = w.split()
    if len(parts) >= 2 and parts[-1] in ("ON", "OFF"):
        return parts[-1]
    return None


def detect_locate_face_on() -> str:
    try:
        r = subprocess.run(
            ["pgrep", "-f", r"locate_face_cpp/build/locate_face|locate_face\.py"],
            check=False,
            capture_output=True,
        )
        return "ON" if r.returncode == 0 else "OFF"
    except OSError:
        return "OFF"


def detect_pull_on(service: str = "torque-cmd-vel.service") -> str:
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "--quiet", service],
            check=False,
        )
        return "ON" if r.returncode == 0 else "OFF"
    except OSError:
        return "OFF"


class BleStatusTelemetry:
    def __init__(
        self,
        notify: NotifyFn,
        motor_power_fn: Optional[MotorPowerFn] = None,
        battery_fn: Optional[BatteryFn] = None,
        on_battery_pct: Optional[BatteryListener] = None,
        on_lan_ip: Optional[LanIpListener] = None,
    ) -> None:
        self._notify = notify
        self._motor_power_fn = motor_power_fn
        self._battery_fn = battery_fn
        self._on_battery_listener = on_battery_pct
        self._on_lan_ip = on_lan_ip
        self._stop = threading.Event()
        self._subscribed = threading.Event()
        self._lock = threading.Lock()
        self._last_ip: Optional[str] = None
        self._voice_ip_announced: Optional[str] = None
        self._last_pwr_sent: Optional[int] = None
        self._last_mp_sent: Optional[str] = None
        self._last_fsm: Optional[int] = DEFAULT_FSM
        self._last_fsm_sent: Optional[int] = None
        self._last_mode_sent: Optional[str] = None
        self._last_features: Dict[str, str] = {}
        self._feature_readers: Dict[str, FeatureFn] = {
            "locate_face": detect_locate_face_on,
            "pull": detect_pull_on,
        }
        self._ros_thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._pwr_retry_gen = 0
        self._ros_battery_pct: Optional[int] = None
        self._ros_gait: Optional[str] = None

    def start(self) -> None:
        if self._poll_thread is not None:
            return
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        self._ros_thread = threading.Thread(target=self._ros_loop, daemon=True)
        self._ros_thread.start()
        log_info(
            "状态遥测已启动（IP / pwr / mp / fsm / mode / locate_face / GAIT / PULL / sound / LT → FFE2）"
        )

    def stop(self) -> None:
        self._stop.set()

    def set_feature_reader(self, name: str, fn: FeatureFn) -> None:
        """注册功能开关读取器；name 见 FEATURE_WIRES。"""
        if name not in FEATURE_WIRES:
            return
        self._feature_readers[name] = fn

    def on_subscribed(self, emit_snapshot: bool = True) -> None:
        """FFE2 订阅成功；可选择是否立即推送快照。"""
        self._subscribed.set()
        with self._lock:
            self._last_pwr_sent = None
            self._last_mode_sent = None
            self._last_fsm_sent = None
            self._pwr_retry_gen += 1
            pwr_gen = self._pwr_retry_gen
        if emit_snapshot:
            self._push_snapshot()
            self._send_mp_state(force=True)
            self._push_all_features(force=True)
            threading.Thread(
                target=self._pwr_retry_loop, args=(pwr_gen,), daemon=True
            ).start()

    def on_unsubscribed(self) -> None:
        self._subscribed.clear()
        with self._lock:
            self._pwr_retry_gen += 1

    def _pwr_retry_loop(self, gen: int) -> None:
        """ROS /pwr 可能晚于 FFE2 订阅到达，短时重试直到发出 pwr。"""
        deadline = time.monotonic() + PWR_RETRY_MAX_SEC
        while time.monotonic() < deadline:
            if self._stop.is_set() or not self._subscribed.is_set():
                return
            with self._lock:
                if gen != self._pwr_retry_gen:
                    return
                if self._last_pwr_sent is not None:
                    return
            pct = self._read_pwr_source()
            if pct is None:
                pct = self._poll_ros_battery_once()
            if pct is not None:
                self._maybe_send_pwr(pct, force=True)
                return
            time.sleep(PWR_RETRY_INTERVAL_SEC)
        log_warn("握手后仍未读到电量（/battery_level 无数据）")

    def _poll_ros_battery_once(self) -> Optional[int]:
        """Foxy 无 wait_for_message，依赖订阅回调填充电量。"""
        with self._lock:
            return self._ros_battery_pct

    def _push_snapshot(self) -> None:
        ip = read_lan_ip()
        if ip:
            self._send_ip(ip, force=True)
        pct = self._read_pwr_source()
        if pct is None:
            pct = self._poll_ros_battery_once()
        if pct is not None:
            self._maybe_send_pwr(pct, force=True)
        with self._lock:
            fsm = self._last_fsm if self._last_fsm is not None else DEFAULT_FSM
        self._send_fsm(fsm, force=True)
        self._send_mode_by_fsm(fsm, force=True)

    def _push_all_features(self, force: bool = False) -> None:
        for name in FEATURE_WIRES:
            wire = self._read_feature(name)
            if wire is None:
                # 未注册且无法探测时的默认：语音开；人脸/拖拽/疾跑关
                if name == "gait":
                    with self._lock:
                        wire = self._ros_gait
                elif name == "sound":
                    wire = "ON"
                elif name in ("sprint", "locate_face", "pull"):
                    wire = "OFF"
            if wire is not None:
                self.push_feature(name, wire, force=force)

    def _read_feature(self, name: str) -> Optional[str]:
        fn = self._feature_readers.get(name)
        if fn is None:
            with self._lock:
                return self._last_features.get(name)
        try:
            return _normalize_on_off(fn())
        except Exception as e:
            log_warn(f"读取功能状态 {name} 失败: {e}")
            return None

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            ip = read_lan_ip()
            if ip:
                self._send_ip(ip)
            pct = self._read_pwr_source()
            if pct is not None:
                self._maybe_send_pwr(pct)
            # 周期校正 locate_face / pull（进程/服务可能被外部启停）
            if self._subscribed.is_set():
                for name in ("locate_face", "pull"):
                    wire = self._read_feature(name)
                    if wire is not None:
                        self.push_feature(name, wire, force=False)
            time.sleep(PWR_POLL_SEC)

    def _read_pwr_source(self) -> Optional[int]:
        if self._battery_fn is not None:
            try:
                pct = self._battery_fn()
            except Exception:
                pct = None
            if pct is not None:
                return pct
        with self._lock:
            ros_pct = self._ros_battery_pct
        if ros_pct is not None:
            return ros_pct
        return read_battery_percent()

    def _on_battery_pct(self, pct: int) -> None:
        pct = _normalize_pwr(pct)
        with self._lock:
            self._ros_battery_pct = pct
            need_send = self._subscribed.is_set() and self._last_pwr_sent is None
        if self._on_battery_listener is not None:
            try:
                self._on_battery_listener(pct)
            except Exception:
                pass
        if need_send:
            self._maybe_send_pwr(pct, force=True)
        elif self._subscribed.is_set():
            self._maybe_send_pwr(pct)

    def _wait_dds_ready(self, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        attempt = 0
        while not self._stop.is_set() and time.monotonic() < deadline:
            attempt += 1
            try:
                import rclpy
                from rclpy.node import Node

                if not rclpy.ok():
                    rclpy.init(args=None)
                n = Node(f"ble_telem_dds_probe_{os.getpid()}_{attempt}")
                n.destroy_node()
                return True
            except Exception as e:
                if attempt == 1 or attempt % 5 == 0:
                    log_warn(f"[telemetry] 等待 DDS… ({e})")
                if self._stop.wait(timeout=1.5):
                    return False
        return False

    def _ros_loop(self) -> None:
        _bootstrap_ros_python_path()
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import Float32, Int32, UInt8
        except ImportError:
            return

        delay = 2.0
        while not self._stop.is_set():
            try:
                # 先等 DDS，避免 create_node 失败刷屏占满 CPU（拖垮 BLE 广播）
                if not self._wait_dds_ready():
                    raise RuntimeError("CycloneDDS 未就绪")
                if not rclpy.ok():
                    rclpy.init(args=None)
                node = Node("ble_status_telemetry")
                break
            except Exception as e:
                log_warn(f"[telemetry] ROS2 节点创建失败: {e}，{delay:.0f}s 后重试")
                if self._stop.wait(timeout=delay):
                    return
                delay = min(delay * 1.5, 30.0)
        else:
            return

        def on_float_battery(msg: Float32) -> None:
            self._on_battery_pct(int(msg.data))

        def on_int_battery(msg: Int32) -> None:
            self._on_battery_pct(int(msg.data))

        def on_uint8_battery(msg: UInt8) -> None:
            self._on_battery_pct(int(msg.data))

        def on_fsm(msg: Int32) -> None:
            state = int(msg.data)
            with self._lock:
                prev = self._last_fsm
                self._last_fsm = state
            if prev != state:
                self._send_fsm(state)
                self._send_mode_by_fsm(state)

        node.create_subscription(Int32, "/fsm_state", on_fsm, 1)
        node.create_subscription(UInt8, "/battery_level", on_uint8_battery, 1)
        try:
            from hightorque_msgs.msg import ControllerState

            def on_ctrl(msg) -> None:
                state = str(getattr(msg, "current_state", "") or "").strip().lower()
                gait = "ON" if state == "running" else "OFF"
                with self._lock:
                    self._ros_gait = gait
                if self._subscribed.is_set():
                    self.push_feature("gait", gait, force=False)

            for topic in CONTROLLER_STATE_TOPICS:
                node.create_subscription(ControllerState, topic, on_ctrl, 1)
        except Exception:
            pass
        for topic in BATTERY_FLOAT_TOPICS:
            try:
                node.create_subscription(Float32, topic, on_float_battery, 1)
            except Exception:
                pass
        for topic in BATTERY_INT_TOPICS:
            try:
                node.create_subscription(Int32, topic, on_int_battery, 1)
            except Exception:
                pass

        try:
            from sensor_msgs.msg import BatteryState

            def on_battery_state(msg: BatteryState) -> None:
                pct = int(msg.percentage * 100) if msg.percentage <= 1.0 else int(msg.percentage)
                self._on_battery_pct(pct)

            node.create_subscription(BatteryState, "/battery_state", on_battery_state, 1)
        except ImportError:
            pass

        try:
            from hightorque_power.msg import PowerSwitch

            def on_power_switch(msg) -> None:
                wire = "ON" if bool(msg.power_switch) else "OFF"
                with self._lock:
                    if wire == self._last_mp_sent:
                        return
                self._send_mp_wire(wire)

            node.create_subscription(
                PowerSwitch, "/power_switch_state", on_power_switch, 1
            )
        except ImportError:
            pass

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while not self._stop.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.2)
        finally:
            try:
                executor.shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass

    def _tx(self, text: str, repeat: int = 1) -> bool:
        if not self._subscribed.is_set():
            return False
        payload = text.encode("utf-8", errors="replace")[:180]
        for i in range(repeat):
            try:
                self._notify(payload)
                log_tx(text)
            except Exception as e:
                log_warn(f"notify 失败: {e}")
                return False
            if i + 1 < repeat:
                time.sleep(FSM_REPEAT_GAP_SEC)
        return True

    def _send_ip(self, ip: str, force: bool = False) -> None:
        announce = False
        with self._lock:
            if self._voice_ip_announced != ip:
                announce = True
                self._voice_ip_announced = ip
            elif not force and ip == self._last_ip:
                return
            self._last_ip = ip
        if announce:
            self._emit_lan_ip(ip)
        self._tx(f"IP:{ip}")

    def _emit_lan_ip(self, ip: str) -> None:
        if self._on_lan_ip is None or not ip:
            return
        try:
            self._on_lan_ip(ip)
        except Exception as e:
            log_warn(f"[voice] 局域网 IP 回调失败: {e}")

    def _maybe_send_pwr(self, raw_pct: int, force: bool = False) -> None:
        if not self._subscribed.is_set():
            return
        pct = _normalize_pwr(raw_pct)
        with self._lock:
            prev = self._last_pwr_sent
            should_send = force or prev is None or pct < prev
        if not should_send:
            return
        if self._tx(_format_pwr(pct)):
            with self._lock:
                self._last_pwr_sent = pct

    def _send_mp_state(self, force: bool = False) -> None:
        if self._motor_power_fn is None:
            return
        wire = self._motor_power_fn()
        if wire not in ("ON", "OFF"):
            return
        self._send_mp_wire(wire, force=force)

    def _send_mp_wire(self, wire: str, force: bool = False) -> None:
        if wire not in ("ON", "OFF"):
            return
        with self._lock:
            if not force and wire == self._last_mp_sent:
                return
            self._last_mp_sent = wire
        self._tx(f"mp:{wire}", repeat=FSM_REPEAT)

    def push_ip(self, force: bool = True) -> Optional[str]:
        """主动推送当前局域网 IP（配网成功后调用）。"""
        ip = read_lan_ip()
        if ip:
            self._send_ip(ip, force=force)
        return ip

    def push_mp_state(self, wire: str, force: bool = True) -> None:
        """MP 指令后立即推送 mp:ON/OFF，供小程序自动站立判断。"""
        self._send_mp_wire(wire, force=force)

    def _send_mode_wire(self, mode_wire: str, force: bool = False) -> None:
        if not mode_wire:
            return
        with self._lock:
            if not force and mode_wire == self._last_mode_sent:
                return
            self._last_mode_sent = mode_wire
        self._tx(f"mode:{mode_wire}", repeat=FSM_REPEAT)

    def push_mode_command(
        self,
        mode_key: str,
        fsm: Optional[int] = None,
        force: bool = True,
    ) -> None:
        """由控制桥推送 mode:M_*（小程序主显示）和可选 fsm:。"""
        key = (mode_key or "").strip()
        if not key:
            return
        low = key.lower()
        pretty = {
            "m_default": "M_default",
            "m_init": "M_init",
            "m_protect": "M_protect",
            "m_resetzero": "M_resetzero",
            "m_tech": "M_tech",
        }.get(low)
        if pretty is None and low.startswith("m_"):
            pretty = "M_" + key.split("_", 1)[-1]
        if pretty is None:
            pretty = key if key.startswith("M_") else f"M_{key}"
        if fsm is not None:
            with self._lock:
                self._last_fsm = int(fsm)
            self._send_fsm(int(fsm), force=force)
        self._send_mode_wire(pretty, force=force)

    def _send_fsm(self, state: int, force: bool = False) -> None:
        with self._lock:
            if not force and state == self._last_fsm_sent:
                return
            self._last_fsm_sent = state
        self._tx(f"fsm:{state}", repeat=FSM_REPEAT)

    def _send_mode_by_fsm(self, state: int, force: bool = False) -> None:
        mode_wire = FSM_TO_MODE_WIRE.get(state, DEFAULT_MODE_WIRE)
        self._send_mode_wire(mode_wire, force=force)

    def push_feature(self, name: str, wire: str, force: bool = True) -> None:
        """推送功能开关；name=locate_face|gait|pull|sound|sprint。"""
        prefix = FEATURE_WIRES.get(name)
        onoff = _normalize_on_off(wire)
        if not prefix or onoff is None:
            return
        with self._lock:
            if not force and self._last_features.get(name) == onoff:
                return
            self._last_features[name] = onoff
        text = f"{prefix} {onoff}"
        self._tx(text, repeat=FEATURE_REPEAT)

    # 便捷别名，供 GATT 钩子调用
    def push_locate_face(self, on: bool, force: bool = True) -> None:
        self.push_feature("locate_face", "ON" if on else "OFF", force=force)

    def push_gait(self, on: bool, force: bool = True) -> None:
        self.push_feature("gait", "ON" if on else "OFF", force=force)

    def push_pull(self, on: bool, force: bool = True) -> None:
        self.push_feature("pull", "ON" if on else "OFF", force=force)

    def push_sound(self, on: bool, force: bool = True) -> None:
        self.push_feature("sound", "ON" if on else "OFF", force=force)

    def push_sprint(self, on: bool, force: bool = True) -> None:
        self.push_feature("sprint", "ON" if on else "OFF", force=force)
