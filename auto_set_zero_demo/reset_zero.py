#!/usr/bin/env python3
"""官方调零：底层 FSM init → prev → confirm，进入 EXEC_RESET_ZERO。

与手柄 / BLE M_resetzero 同一条路径。控制器会调用 /reset_zero，
把**当前所有电机角度**写成 0 并写入 Flash。

机必须已经摆在出厂写零姿态。不要在限位处或任意姿态跑本脚本。
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

import rclpy
from hightorque_msgs.msg import ControllerState
from hightorque_msgs.srv import ChangeState
from rclpy.node import Node
from std_msgs.msg import Int32

LOG_DIR = "/tmp/ros2_stand"
CONTROLLER_LAUNCH = [
    "ros2",
    "launch",
    "hightorque_controller",
    "hightorque_controller.launch.py",
    "action_library_path:=/home/nvidia/action_library",
]

FSM_INIT = 0
FSM_CANDIDATE_RESET_ZERO = 9
FSM_EXEC_RESET_ZERO = 10
FSM_RESET_OK = 11
FSM_RESET_FAIL = 12

FSM_NAME = {
    0: "INIT",
    8: "PROTECTION_SHUTDOWN",
    9: "CANDIDATE_RESET_ZERO",
    10: "EXEC_RESET_ZERO",
    11: "EXEC_RESET_ZERO_SUCCESSFULLY",
    12: "EXEC_RESET_ZERO_FAILED",
}


class ResetZeroNode(Node):
    def __init__(self) -> None:
        super().__init__("reset_zero_demo")
        self.cli = self.create_client(ChangeState, "/hightorque_controller/change_fsm_state")
        self.ctrl = None
        self.fsm = None
        self.create_subscription(ControllerState, "/hightorque_controller/state", self._on_ctrl, 10)
        self.create_subscription(Int32, "/fsm_state", self._on_fsm, 10)

    def _on_ctrl(self, msg: ControllerState) -> None:
        self.ctrl = msg

    def _on_fsm(self, msg: Int32) -> None:
        self.fsm = int(msg.data)

    def spin_for(self, dt: float) -> None:
        deadline = time.monotonic() + dt
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))

    def start_controller(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, "controller.log")
        print(f"控制器未运行，正在启动… 日志 {log_path}", flush=True)
        with open(log_path, "ab", buffering=0) as log:
            log.write(b"\n===== reset_zero.sh auto-start =====\n")
            subprocess.Popen(
                CONTROLLER_LAUNCH,
                stdout=log,
                stderr=log,
                start_new_session=True,
                env=os.environ.copy(),
            )

    def ensure_controller(self) -> None:
        if self.cli.wait_for_service(timeout_sec=1.5):
            return
        self.start_controller()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25.0:
            if self.cli.wait_for_service(timeout_sec=1.0):
                print("控制器已就绪", flush=True)
                return
        raise RuntimeError("等待 /hightorque_controller/change_fsm_state 超时")

    def call_fsm(self, command: str) -> None:
        req = ChangeState.Request()
        req.states = [command]
        print(f"change_fsm_state ['{command}']", flush=True)
        future = self.cli.call_async(req)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > 8.0:
                raise RuntimeError(f"change_fsm_state {command} 超时")
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result()
        print(f"  success={resp.success}  {resp.message}", flush=True)
        if not resp.success:
            raise RuntimeError(f"FSM 命令失败: {command}: {resp.message}")

    def wait_fsm(self, *values: int, timeout: float) -> int:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin_for(0.1)
            if self.fsm in values:
                name = FSM_NAME.get(self.fsm, str(self.fsm))
                print(f"  FSM={self.fsm} ({name})", flush=True)
                return self.fsm
        raise RuntimeError(
            f"等待 FSM {values} 超时，当前={self.fsm} "
            f"({FSM_NAME.get(self.fsm, '?') if self.fsm is not None else '无'})"
        )

    def run(self) -> int:
        self.ensure_controller()
        self.spin_for(0.5)
        print("官方调零：init → prev → confirm（全机当前角写零）", flush=True)

        self.call_fsm("init")
        self.wait_fsm(FSM_INIT, timeout=5.0)
        time.sleep(0.25)

        self.call_fsm("prev")
        self.spin_for(0.4)

        self.call_fsm("confirm")
        self.wait_fsm(FSM_EXEC_RESET_ZERO, FSM_RESET_OK, FSM_RESET_FAIL, timeout=8.0)

        print("等待写零完成…", flush=True)
        done = self.wait_fsm(FSM_RESET_OK, FSM_RESET_FAIL, timeout=20.0)
        if self.ctrl is not None:
            print(
                f"  mode={self.ctrl.current_mode} state={self.ctrl.current_state}",
                flush=True,
            )
        if done == FSM_RESET_OK:
            print("调零成功（随后会自动回 INIT）", flush=True)
            return 0
        print("调零失败", file=sys.stderr)
        return 1


def main() -> int:
    rclpy.init(args=None)
    node = ResetZeroNode()
    try:
        return node.run()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
