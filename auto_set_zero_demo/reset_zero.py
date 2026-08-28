#!/usr/bin/env python3
"""全机写零：把当前电机角写成 0 并写入 Flash。

默认：midware 直连 /reset_zero（不经过 FSM confirm，避免控制器 segfault）。
若 hightorque_controller 在跑，会先 init→prev 进 reset 候选再写零。

pi_plus_orin bringup 只有 midware，无控制器时自动跳过 FSM。

机必须已经摆在目标写零姿态再跑。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from typing import List, Optional, Sequence

import rclpy
from hightorque_msgs.msg import ControllerState
from hightorque_msgs.srv import ChangeState, ResetZero
from rclpy.node import Node
from std_msgs.msg import Int32

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = "/tmp/ros2_stand"
CONTROLLER_LOG = os.path.join(LOG_DIR, "controller.log")

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
        self.cli_fsm = self.create_client(
            ChangeState, "/hightorque_controller/change_fsm_state"
        )
        self.cli_reset = self.create_client(ResetZero, "/reset_zero")
        self.ctrl: Optional[ControllerState] = None
        self.fsm: Optional[int] = None
        self.create_subscription(
            ControllerState, "/hightorque_controller/state", self._on_ctrl, 10
        )
        self.create_subscription(Int32, "/fsm_state", self._on_fsm, 10)

    def _on_ctrl(self, msg: ControllerState) -> None:
        self.ctrl = msg

    def _on_fsm(self, msg: Int32) -> None:
        self.fsm = int(msg.data)

    def spin_for(self, dt: float) -> None:
        deadline = time.monotonic() + dt
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )

    def call_srv(self, client, request, timeout: float = 12.0):
        future = client.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f"调用超时: {client.srv_name}")
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def fsm_available(self) -> bool:
        return self.cli_fsm.wait_for_service(timeout_sec=1.0)

    def ensure_midware(self) -> None:
        if self.cli_reset.wait_for_service(timeout_sec=1.5):
            return
        script = os.path.join(ROOT, "ensure_midware.sh")
        if not os.path.isfile(script):
            raise RuntimeError("未找到 ensure_midware.sh，请先启动 pi_plus_orin bringup")
        print("midware 未就绪，正在执行 ensure_midware.sh …", flush=True)
        subprocess.run([script], check=True, cwd=ROOT)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 35.0:
            if self.cli_reset.wait_for_service(timeout_sec=1.0):
                print("midware /reset_zero 已就绪", flush=True)
                return
        raise RuntimeError("等待 /reset_zero 超时，请检查 pi_plus_orin bringup 日志")

    def start_controller(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        print(f"启动 hightorque_controller… 日志 {CONTROLLER_LOG}", flush=True)
        with open(CONTROLLER_LOG, "ab", buffering=0) as log:
            log.write(b"\n===== reset_zero.py start controller =====\n")
            subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "hightorque_controller",
                    "hightorque_controller.launch.py",
                    "action_library_path:=/home/nvidia/action_library",
                ],
                stdout=log,
                stderr=log,
                start_new_session=True,
                env=os.environ.copy(),
            )
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0:
            if self.fsm_available():
                print("控制器已就绪", flush=True)
                return
            time.sleep(0.5)
        raise RuntimeError("启动 hightorque_controller 超时")

    def release_motor_sessions(self) -> None:
        unload = os.path.join(ROOT, "unload.sh")
        if not os.path.isfile(unload):
            return
        print("释放电机占用（unload.sh）…", flush=True)
        subprocess.run([unload], cwd=ROOT, check=False)
        time.sleep(0.8)

    def call_fsm(self, command: str, *, timeout: float = 12.0) -> None:
        req = ChangeState.Request()
        req.states = [command]
        print(f"change_fsm_state ['{command}']", flush=True)
        try:
            resp = self.call_srv(self.cli_fsm, req, timeout=timeout)
        except RuntimeError as exc:
            if command == "confirm" and self._controller_crashed():
                raise RuntimeError(
                    "confirm 时控制器进程崩溃（segfault）。"
                    "请用默认直连模式（不要 --fsm）"
                ) from exc
            raise
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
        cur = FSM_NAME.get(self.fsm, str(self.fsm) if self.fsm is not None else "无")
        raise RuntimeError(f"等待 FSM {values} 超时，当前={self.fsm} ({cur})")

    def _controller_crashed(self) -> bool:
        try:
            with open(CONTROLLER_LOG, "rb") as f:
                tail = f.read()[-4096].decode("utf-8", errors="ignore")
            return "exit code -11" in tail or "process has died" in tail
        except OSError:
            return False

    def enter_reset_candidate(self) -> None:
        """init → prev，停在 CANDIDATE_RESET_ZERO（不 confirm）。"""
        if not self.fsm_available():
            return
        self.spin_for(0.4)
        if self.fsm == FSM_CANDIDATE_RESET_ZERO:
            print("已在 CANDIDATE_RESET_ZERO，跳过 init/prev", flush=True)
            time.sleep(1.0)
            return
        self.call_fsm("init")
        self.wait_fsm(FSM_INIT, timeout=6.0)
        time.sleep(0.3)
        self.call_fsm("prev")
        self.wait_fsm(FSM_CANDIDATE_RESET_ZERO, timeout=6.0)
        print("等待控制权释放…", flush=True)
        time.sleep(1.2)

    def prepare_for_reset(self, *, use_fsm: bool) -> None:
        if use_fsm and self.fsm_available():
            self.enter_reset_candidate()
            return
        if use_fsm:
            print(
                "控制器未运行（pi_plus_orin 仅 midware），跳过 FSM，直连 /reset_zero",
                flush=True,
            )
        self.release_motor_sessions()

    def recover_candidate(self) -> None:
        if not self.fsm_available():
            return
        print("恢复：FSM init（退出 reset 候选）", flush=True)
        try:
            self.call_fsm("init", timeout=8.0)
            self.wait_fsm(FSM_INIT, timeout=5.0)
        except Exception as exc:
            print(f"  恢复失败: {exc}", flush=True)

    def call_reset_zero_direct(
        self, motor_ids: Sequence[int], timeout_ms: int
    ) -> bool:
        if not self.cli_reset.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/reset_zero 不可用")
        req = ResetZero.Request()
        req.timeout_ms = int(timeout_ms)
        req.motor_ids = [int(x) for x in motor_ids]
        scope = f"{len(motor_ids)} 个电机" if motor_ids else "全机"
        print(f"调用 /reset_zero（{scope}，timeout={timeout_ms}ms）", flush=True)
        resp = self.call_srv(
            self.cli_reset, req, timeout=max(70.0, timeout_ms / 1000.0 + 10.0)
        )
        status = int(getattr(resp, "status", 0))
        print(
            f"  success={resp.success}  status={status}  {resp.message}",
            flush=True,
        )
        if not resp.success and "active session" in (resp.message or "").lower():
            raise RuntimeError(resp.message)
        return bool(resp.success)

    def run_direct(
        self, motor_ids: Sequence[int], timeout_ms: int, *, use_fsm: bool
    ) -> int:
        self.ensure_midware()
        self.prepare_for_reset(use_fsm=use_fsm)
        ok = False
        for attempt in range(3):
            try:
                ok = self.call_reset_zero_direct(motor_ids, timeout_ms)
                break
            except RuntimeError:
                if attempt >= 2:
                    raise
                wait = 1.0 + attempt
                print(f"写零失败，{wait:.0f}s 后重试 ({attempt + 1}/3)…", flush=True)
                self.release_motor_sessions()
                time.sleep(wait)
        if use_fsm:
            self.recover_candidate()
        if ok:
            print("写零成功", flush=True)
            return 0
        print("写零失败", file=sys.stderr)
        return 1

    def run_fsm_official(self) -> int:
        if not self.fsm_available():
            self.ensure_midware()
            self.start_controller()
        self.spin_for(0.5)
        print("警告: FSM confirm 在本机已知会 segfault", flush=True)
        self.call_fsm("init")
        self.wait_fsm(FSM_INIT, timeout=5.0)
        time.sleep(0.25)
        self.call_fsm("prev")
        self.wait_fsm(FSM_CANDIDATE_RESET_ZERO, timeout=5.0)
        time.sleep(0.25)
        self.call_fsm("confirm", timeout=15.0)
        self.wait_fsm(FSM_EXEC_RESET_ZERO, FSM_RESET_OK, FSM_RESET_FAIL, timeout=10.0)
        done = self.wait_fsm(FSM_RESET_OK, FSM_RESET_FAIL, timeout=25.0)
        return 0 if done == FSM_RESET_OK else 1

    def run_recover_only(self, *, start_controller: bool) -> int:
        self.ensure_midware()
        if not self.fsm_available():
            if start_controller:
                self.start_controller()
            else:
                print(
                    "控制器未运行，无法 FSM init。"
                    "OLED 若仍卡 reset，可 ./reset_zero.sh --recover --start-controller",
                    flush=True,
                )
                return 1
        self.recover_candidate()
        return 0


def parse_motor_ids(text: str) -> List[int]:
    ids: List[int] = []
    for part in text.replace(",", " ").split():
        part = part.strip()
        if part:
            ids.append(int(part))
    return ids


def main() -> int:
    parser = argparse.ArgumentParser(description="全机/指定电机写零（midware /reset_zero）")
    parser.add_argument(
        "--fsm",
        action="store_true",
        help="走 FSM init→prev→confirm（本机 confirm 易崩溃，不推荐）",
    )
    parser.add_argument(
        "--no-fsm",
        action="store_true",
        help="不尝试 FSM，直接 /reset_zero（仅 midware 时默认行为）",
    )
    parser.add_argument(
        "--recover",
        action="store_true",
        help="仅 FSM init，退出 OLED reset 候选界面",
    )
    parser.add_argument(
        "--start-controller",
        action="store_true",
        help="配合 --recover：先启动 hightorque_controller",
    )
    parser.add_argument("--motor-ids", default="", help="只写指定电机索引")
    parser.add_argument("--timeout-ms", type=int, default=60000)
    args = parser.parse_args()
    motor_ids = parse_motor_ids(args.motor_ids) if args.motor_ids else []
    use_fsm = not args.no_fsm

    rclpy.init(args=None)
    node = ResetZeroNode()
    try:
        if args.recover:
            return node.run_recover_only(start_controller=args.start_controller)
        if args.fsm:
            return node.run_fsm_official()
        print("写零：midware /reset_zero（控制器在跑时先 FSM 候选）", flush=True)
        return node.run_direct(motor_ids, args.timeout_ms, use_fsm=use_fsm)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        if os.path.isfile(CONTROLLER_LOG):
            print(f"详见日志: {CONTROLLER_LOG}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
