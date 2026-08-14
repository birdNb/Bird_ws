#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE WiFi 配网：WIFI <SSID> <PASSWORD> → nmcli 保存并连接，回调最终结果。"""

from __future__ import annotations

import re
import shlex
import subprocess
import threading
import time
from typing import Callable, Optional, Tuple

LogFn = Callable[[str], None]
ResultFn = Callable[[bool, str], None]
VoidFn = Callable[[], None]

# 与 ble_command_dispatcher 载荷分隔一致
WIFI_PAYLOAD_SEP = "\x1e"
WIFI_IFACE_CANDIDATES = ("wlan0", "wlan1", "wlp1s0")
CONNECT_TIMEOUT_SEC = 45.0
LINK_POLL_SEC = 3.0
RESULT_OK = "WIFI OK"
RESULT_FAIL_PREFIX = "WIFI FAIL"


def parse_wifi_command(text: str) -> Optional[Tuple[str, str]]:
    """
    解析 `WIFI <SSID> <PASSWORD>`。
    - 支持引号：WIFI "My SSID" "p@ss w"
    - 无引号时：第一个 token 为 SSID，其余为密码（密码可含空格）
    """
    raw = (text or "").strip()
    if not raw:
        return None
    m = re.match(r"^WIFI\s+(.+)$", raw, re.IGNORECASE | re.DOTALL)
    if not m:
        return None
    rest = m.group(1).strip()
    if not rest:
        return None
    try:
        parts = shlex.split(rest, posix=True)
    except ValueError:
        parts = rest.split(None, 1)
        if len(parts) < 2:
            return None
        return parts[0], parts[1]
    if len(parts) < 2:
        return None
    ssid = parts[0].strip()
    password = " ".join(parts[1:]).strip()
    if not ssid or not password:
        return None
    if len(ssid) > 32 or len(password) > 63:
        return None
    return ssid, password


def _run(
    cmd: list, *, timeout: float = 20.0
) -> Tuple[int, str, str]:
    try:
        r = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return r.returncode, (r.stdout or "").strip(), (r.stderr or "").strip()
    except FileNotFoundError:
        return 127, "", "nmcli_missing"
    except subprocess.TimeoutExpired:
        return 124, "", "timeout"
    except OSError as e:
        return 1, "", str(e)


def _pick_wifi_iface() -> Optional[str]:
    rc, out, _ = _run(
        ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device", "status"],
        timeout=5.0,
    )
    if rc != 0:
        return None
    wifi_devs = []
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) < 2:
            continue
        dev, typ = parts[0], parts[1]
        if typ != "wifi":
            continue
        state = parts[2] if len(parts) > 2 else ""
        wifi_devs.append((dev, state))
    for prefer in WIFI_IFACE_CANDIDATES:
        for dev, _state in wifi_devs:
            if dev == prefer:
                return dev
    return wifi_devs[0][0] if wifi_devs else None


def _classify_nmcli_error(stderr: str, stdout: str) -> str:
    msg = f"{stderr}\n{stdout}".lower()
    if "secret was required" in msg or "secrets were required" in msg:
        return "auth"
    if "no network with ssid" in msg or "not found" in msg:
        return "not_found"
    if "timeout" in msg:
        return "timeout"
    if "password" in msg and ("wrong" in msg or "fail" in msg):
        return "auth"
    # 取首行简短原因，去掉换行
    brief = (stderr or stdout or "error").splitlines()[0].strip()
    brief = re.sub(r"\s+", "_", brief)[:40]
    return brief or "error"


def _wifi_link_connected(iface: Optional[str] = None) -> bool:
    """当前 WiFi 网卡是否处于 connected。"""
    rc, out, _ = _run(
        ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device", "status"],
        timeout=5.0,
    )
    if rc != 0:
        return False
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) < 3:
            continue
        dev, typ, state = parts[0], parts[1], parts[2]
        if typ != "wifi":
            continue
        if iface and dev != iface:
            continue
        if state == "connected":
            return True
    return False


class WifiManager:
    """异步配网：保存配置并尝试连接，完成后回调结果文案。"""

    def __init__(self, log: LogFn = print) -> None:
        self._log = log
        self._lock = threading.Lock()
        self._busy = False
        self._stop = threading.Event()
        self._monitor_thread: Optional[threading.Thread] = None
        self._on_link_down: Optional[VoidFn] = None
        self._was_connected = False
        self._suppress_link_events = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def start_link_monitor(self, on_disconnected: Optional[VoidFn] = None) -> None:
        """轮询 WiFi 链路：曾连接后断开时回调（配网过程中抑制）。"""
        self._on_link_down = on_disconnected
        self._was_connected = _wifi_link_connected()
        if self._monitor_thread is not None:
            return
        self._stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._link_monitor_loop, name="ble-wifi-link", daemon=True
        )
        self._monitor_thread.start()

    def stop_link_monitor(self) -> None:
        self._stop.set()
        t = self._monitor_thread
        if t is not None:
            t.join(timeout=2.0)
        self._monitor_thread = None

    def _link_monitor_loop(self) -> None:
        while not self._stop.wait(LINK_POLL_SEC):
            if self._suppress_link_events:
                continue
            connected = _wifi_link_connected()
            if self._was_connected and not connected:
                self._log("[wifi] 链路断开")
                cb = self._on_link_down
                if cb is not None:
                    try:
                        cb()
                    except Exception as e:
                        self._log(f"[wifi] 断连回调失败: {e}")
            self._was_connected = connected

    def connect_async(
        self,
        ssid: str,
        password: str,
        on_result: Optional[ResultFn] = None,
    ) -> bool:
        """启动后台配网。若已有任务进行中则返回 False。"""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            self._suppress_link_events = True

        def _job() -> None:
            ok = False
            detail = "error"
            try:
                ok, detail = self._connect_blocking(ssid, password)
            except Exception as e:
                ok, detail = False, "exception"
                self._log(f"[wifi] 异常: {e}")
            finally:
                with self._lock:
                    self._busy = False
                    self._suppress_link_events = False
                # 同步链路状态，避免配网刚结束误报断连
                self._was_connected = _wifi_link_connected()
            if on_result is not None:
                try:
                    on_result(ok, detail)
                except Exception as e:
                    self._log(f"[wifi] 结果回调失败: {e}")

        threading.Thread(target=_job, name="ble-wifi-connect", daemon=True).start()
        return True

    def _connect_blocking(self, ssid: str, password: str) -> Tuple[bool, str]:
        self._log(f"[wifi] 开始配网 SSID={ssid!r}")
        iface = _pick_wifi_iface()
        if not iface:
            self._log("[wifi] 未找到 WiFi 网卡")
            return False, "no_device"

        # 触发扫描（忽略失败）
        _run(["nmcli", "device", "wifi", "rescan", "ifname", iface], timeout=8.0)
        time.sleep(1.0)

        cmd = [
            "nmcli",
            "device",
            "wifi",
            "connect",
            ssid,
            "password",
            password,
            "ifname",
            iface,
        ]
        rc, out, err = _run(cmd, timeout=CONNECT_TIMEOUT_SEC)
        if rc == 0:
            self._log(f"[wifi] 连接成功 ifname={iface}")
            return True, "ok"

        reason = _classify_nmcli_error(err, out)
        self._log(f"[wifi] 连接失败 rc={rc} reason={reason}")
        if err:
            self._log(f"[wifi] stderr: {err[:200]}")
        return False, reason


def format_wifi_result(ok: bool, detail: str = "") -> str:
    if ok:
        return RESULT_OK
    reason = (detail or "error").strip() or "error"
    if reason.lower() == "ok":
        reason = "error"
    return f"{RESULT_FAIL_PREFIX} {reason}"
