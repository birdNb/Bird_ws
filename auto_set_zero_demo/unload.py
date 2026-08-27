#!/usr/bin/env python3
"""卸力：关节 kp=0, kd=0, τ=0，不进阻尼。

控制器在跑时走 FSM init（InitController 零力矩）。
控制器不在时（例如 --takeover 之后）经 midware 申请全部空闲电机，发零力矩后
以 ZERO_TORQUE_MODE 释放，避免 protect 的 kd=1 阻尼。
"""
from __future__ import annotations

import sys
import time

import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ChangeState, GetAvailableMotors, ReleaseControl, RequestControl
from rclpy.node import Node


class UnloadZeroTorque(Node):
    def __init__(self) -> None:
        super().__init__("unload_zero_torque")
        self.cli_get = self.create_client(GetAvailableMotors, "get_available_motors")
        self.cli_req = self.create_client(RequestControl, "request_control")
        self.cli_rel = self.create_client(ReleaseControl, "release_control")
        self.cli_fsm = self.create_client(ChangeState, "/hightorque_controller/change_fsm_state")
        self.cmd_pub = self.create_publisher(MotorControlCommand, "control_command", 20)

    def call_srv(self, client, request, timeout: float = 5.0):
        future = client.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f"调用超时: {client.srv_name}")
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def via_controller_init(self) -> bool:
        if not self.cli_fsm.wait_for_service(timeout_sec=1.5):
            return False
        req = ChangeState.Request()
        req.states = ["init"]
        print("控制器在线，下发 FSM init（零力矩，非阻尼）", flush=True)
        resp = self.call_srv(self.cli_fsm, req)
        print(f"  success={resp.success}  {resp.message}", flush=True)
        return bool(resp.success)

    def via_midware(self) -> int:
        if not self.cli_req.wait_for_service(timeout_sec=3.0):
            print("midware 的 /request_control 不可用，请先启动 pi_plus_orin bringup", file=sys.stderr)
            return 1
        if not self.cli_get.wait_for_service(timeout_sec=3.0):
            print("get_available_motors 不可用", file=sys.stderr)
            return 1

        get_req = GetAvailableMotors.Request()
        get_req.node_name = ""
        idle = self.call_srv(self.cli_get, get_req)
        ids = list(idle.motor_ids)
        names = list(idle.motor_names)
        if not ids:
            print("没有空闲电机：仍被其它节点占用，无法卸力", file=sys.stderr)
            return 1

        print(f"控制器未运行，经 midware 对 {len(ids)} 轴发零力矩: {', '.join(names)}", flush=True)

        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = ids
        req.control_mode = RequestControl.Request.TORQUE_MODE
        req.default_behavior = RequestControl.Request.ZERO_TORQUE_MODE
        req.timeout_ms = 2000
        req.default_kp = [0.0] * len(ids)
        req.default_kd = [0.0] * len(ids)
        resp = self.call_srv(self.cli_req, req)
        if not resp.success:
            print("申请控制权失败: " + resp.message, file=sys.stderr)
            return 1
        uuid = resp.uuid

        zeros = [0.0] * len(ids)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.4:
            cmd = MotorControlCommand()
            cmd.uuid = uuid
            cmd.motor_ids = ids
            cmd.torques = zeros
            cmd.kp = zeros
            cmd.kd = zeros
            cmd.positions = zeros
            cmd.velocities = zeros
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

        rel = ReleaseControl.Request()
        rel.node_name = self.get_name()
        rel.uuid = uuid
        rel.release_mode = ReleaseControl.Request.ZERO_TORQUE_MODE
        rel_resp = self.call_srv(self.cli_rel, rel, timeout=3.0)
        print("释放为零力矩: " + (rel_resp.message if rel_resp else "no resp"), flush=True)
        print("卸力完成（kp=0 kd=0 τ=0，不是阻尼）", flush=True)
        return 0


def main() -> int:
    rclpy.init(args=None)
    node = UnloadZeroTorque()
    try:
        if node.via_controller_init():
            return 0
        print("未找到 /hightorque_controller/change_fsm_state（控制器可能被 --takeover 停掉）", flush=True)
        return node.via_midware()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
