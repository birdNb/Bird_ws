#!/usr/bin/env python3
"""上层 default_bt 站立。控制器不在时自动拉起（例如 --takeover 之后）。"""
from __future__ import annotations

import os
import subprocess
import sys
import time

import rclpy
from hightorque_msgs.msg import ControllerState
from hightorque_msgs.srv import ChangeState
from rclpy.node import Node

LOG_DIR = "/tmp/ros2_stand"
CONTROLLER_LAUNCH = [
    "ros2",
    "launch",
    "hightorque_controller",
    "hightorque_controller.launch.py",
    "action_library_path:=/home/nvidia/action_library",
]


class StandNode(Node):
    def __init__(self) -> None:
        super().__init__("stand_demo")
        self.cli = self.create_client(ChangeState, "/hightorque_controller/change_state")
        self.state = None
        self.create_subscription(ControllerState, "/hightorque_controller/state", self._on_state, 10)

    def _on_state(self, msg: ControllerState) -> None:
        self.state = msg

    def spin_for(self, dt: float) -> None:
        deadline = time.monotonic() + dt
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))

    def wait_service(self, timeout: float) -> bool:
        return self.cli.wait_for_service(timeout_sec=timeout)

    def start_controller(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, "controller.log")
        print(f"控制器未运行，正在启动… 日志 {log_path}", flush=True)
        with open(log_path, "ab", buffering=0) as log:
            log.write(b"\n===== stand.sh auto-start =====\n")
            subprocess.Popen(
                CONTROLLER_LAUNCH,
                stdout=log,
                stderr=log,
                start_new_session=True,
                env=os.environ.copy(),
            )

    def ensure_controller(self) -> None:
        if self.wait_service(1.5):
            return
        self.start_controller()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25.0:
            if self.wait_service(1.0):
                print("控制器已就绪", flush=True)
                return
        raise RuntimeError("等待 /hightorque_controller/change_state 超时，请检查控制器是否启动")

    def stand(self) -> int:
        self.ensure_controller()
        req = ChangeState.Request()
        req.states = ["standing"]
        print("下发站立: /hightorque_controller/change_state ['standing']", flush=True)
        future = self.cli.call_async(req)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > 8.0:
                raise RuntimeError("change_state 调用超时")
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result()
        print(f"  success={resp.success}  {resp.message}", flush=True)
        if not resp.success:
            return 1

        print("等待进入 standby…", flush=True)
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < 20.0:
            self.spin_for(0.1)
            m = self.state
            if m is None:
                continue
            key = (m.current_mode, m.current_state, m.current_policy)
            if key != last:
                print(
                    f"  mode={m.current_mode} state={m.current_state} policy={m.current_policy!r}",
                    flush=True,
                )
                last = key
            if m.current_mode == "default_bt" and m.current_state.lower() == "standby":
                print("站立完成，当前 standby", flush=True)
                return 0
        print("超时仍未进入 standby", file=sys.stderr)
        return 1


def main() -> int:
    rclpy.init(args=None)
    node = StandNode()
    try:
        return node.stand()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
