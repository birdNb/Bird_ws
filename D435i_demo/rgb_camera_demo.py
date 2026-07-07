#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D435i RGB 摄像头预览 Demo

打开 Intel RealSense D435i 的彩色流，用 OpenCV 窗口实时显示。
按 ESC / q 退出；Ctrl+C 也可结束。

LubanCat 等未打 RealSense 内核补丁的板子，pyrealsense2 可能报
"No device connected"，此时会自动回退到 V4L2（通常 /dev/video2）。

运行:
    python3 rgb_camera_demo.py
    python3 rgb_camera_demo.py --backend v4l2 --device /dev/video2
    ./start.sh
"""

import os
import glob
import time
import argparse
import subprocess

# SSH 无 DISPLAY 时兜底（与 locate_face.py 一致）
if not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":0"
if not os.environ.get("XAUTHORITY"):
    _xauth = os.path.expanduser("~/.Xauthority")
    if os.path.exists(_xauth):
        os.environ["XAUTHORITY"] = _xauth

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    rs = None
    HAS_REALSENSE = False

WINDOW_NAME = "D435i RGB"


def parse_args():
    p = argparse.ArgumentParser(description="D435i RGB 摄像头预览")
    p.add_argument("--width", type=int, default=640, help="彩色流宽度 (默认 640)")
    p.add_argument("--height", type=int, default=480, help="彩色流高度 (默认 480)")
    p.add_argument("--fps", type=int, default=15, help="帧率 (默认 15；本板 640x480@30 不稳定)")
    p.add_argument(
        "--backend",
        choices=("auto", "realsense", "v4l2"),
        default="auto",
        help="采集后端: auto=先 RealSense 再 V4L2 (默认 auto)",
    )
    p.add_argument(
        "--device",
        type=str,
        default="",
        help="V4L2 设备路径，如 /dev/video2（backend=v4l2 或 auto 回退时用）",
    )
    p.add_argument(
        "--serial",
        type=str,
        default="",
        help="指定 RealSense 序列号（多设备时）",
    )
    p.add_argument(
        "--no-gui",
        action="store_true",
        help="不弹窗，仅打印帧率（无头调试）",
    )
    return p.parse_args()


def usb_has_d435i():
    try:
        out = subprocess.check_output(["lsusb"], text=True, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False
    return "8086:0b3a" in out or "RealSense" in out


def realsense_device_count():
    if not HAS_REALSENSE:
        return 0
    return rs.context().query_devices().size()


def print_realsense_hint():
    print("诊断:")
    if usb_has_d435i():
        print("  - USB 已识别 D435i，但 pyrealsense2 枚举不到设备")
        print("  - 常见原因: 内核未打 RealSense 补丁（LubanCat / 通用 ARM 板常见）")
        print("  - 本 demo 将尝试 V4L2 打开 RGB（/dev/video4，YUYV 彩色）")
        print("  - /dev/video2 是红外流，会显示点阵，请勿使用")
        print("  - 若需完整 SDK 功能，请编译 librealsense2 并启用 RSUSB 后端")
        print("    或执行: ./scripts/install_udev.sh")
    else:
        print("  - USB 未检测到 D435i，请检查线缆与供电")
    print("  - 不要用 sudo 运行本 demo（会用到错误的 Python 环境）")


class RealsenseCapture:
    def __init__(self, args):
        if not HAS_REALSENSE:
            raise RuntimeError("未安装 pyrealsense2")

        pipeline = rs.pipeline()
        config = rs.config()
        if args.serial:
            config.enable_device(args.serial)
        config.enable_stream(
            rs.stream.color,
            args.width,
            args.height,
            rs.format.bgr8,
            args.fps,
        )
        profile = pipeline.start(config)
        self.pipeline = pipeline
        self._print_info(profile)

    def _print_info(self, profile):
        device = profile.get_device()
        color = profile.get_stream(rs.stream.color).as_video_stream_profile()
        print(f"后端: RealSense SDK")
        print(f"设备: {device.get_info(rs.camera_info.name)}")
        print(f"序列号: {device.get_info(rs.camera_info.serial_number)}")
        print(f"固件: {device.get_info(rs.camera_info.firmware_version)}")
        print(f"彩色流: {color.width()}x{color.height()} @ {color.fps()} fps")

    def read(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        return True, np.asanyarray(color_frame.get_data())

    def release(self):
        self.pipeline.stop()


class V4L2Capture:
    # D435i V4L2 节点: video0=深度, video2=红外(点阵), video4=RGB(YUYV)
    DEFAULT_INDEX = 4
    COLOR_FOURCC = "YUYV"

    def __init__(self, args):
        index = self._resolve_index(args.device)
        if index is None:
            index = self._auto_pick_index(args.width, args.height, args.fps)

        self.device = f"/dev/video{index}"
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开 V4L2 设备 {self.device}")

        fourcc = cv2.VideoWriter_fourcc(*self.COLOR_FOURCC)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        self.cap.set(cv2.CAP_PROP_FPS, args.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        for _ in range(3):
            self.cap.grab()

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fc_str = "".join(chr((fc >> 8 * i) & 0xFF) for i in range(4))
        print("后端: V4L2")
        print(f"设备: {self.device} (index={index}, RGB 彩色)")
        print(f"格式: {fc_str}")
        print(f"彩色流: {w}x{h} @ {fps:.0f} fps")
        if index == 2:
            print("警告: /dev/video2 是红外流，画面会带点阵；请改用 /dev/video4")

    @staticmethod
    def _resolve_index(device):
        if not device:
            return None
        if device.startswith("/dev/video"):
            suffix = device.rsplit("video", 1)[-1]
            if suffix.isdigit():
                return int(suffix)
        if device.isdigit():
            return int(device)
        raise RuntimeError(f"无法解析 V4L2 设备: {device}")

    @staticmethod
    def _is_color_frame(frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return False
        # 红外 GREY 被 OpenCV 复制成三通道时 R≈G≈B；真彩色有明显色差
        b, g, r = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
        return float(np.std(r - b)) > 3.0

    @classmethod
    def _probe_index(cls, index, width, height, fps):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cls.COLOR_FOURCC))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        for _ in range(3):
            cap.grab()
        ok, frame = cap.read()
        cap.release()
        return ok and cls._is_color_frame(frame)

    def _auto_pick_index(self, width, height, fps):
        preferred = [self.DEFAULT_INDEX, 4, 6]
        seen = set()
        candidates = []
        for idx in preferred:
            if idx not in seen:
                seen.add(idx)
                candidates.append(idx)

        for path in sorted(glob.glob("/dev/video[0-9]*")):
            suffix = path.rsplit("video", 1)[-1]
            if suffix.isdigit():
                idx = int(suffix)
                if idx not in seen:
                    seen.add(idx)
                    candidates.append(idx)

        for idx in candidates:
            if self._probe_index(idx, width, height, fps):
                return idx

        raise RuntimeError(
            "未找到 RGB 彩色节点，请指定 --device /dev/video4；"
            "勿用 /dev/video2（红外点阵）"
        )

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def open_capture(args):
    if args.backend == "v4l2":
        return V4L2Capture(args)

    if args.backend == "realsense":
        if not HAS_REALSENSE:
            print("未安装 pyrealsense2: pip3 install -r requirements.txt")
            raise SystemExit(1)
        try:
            return RealsenseCapture(args)
        except RuntimeError as exc:
            print(f"无法启动 RealSense 彩色流: {exc}")
            print_realsense_hint()
            raise SystemExit(1)

    # auto
    if HAS_REALSENSE and realsense_device_count() > 0:
        try:
            return RealsenseCapture(args)
        except RuntimeError as exc:
            print(f"RealSense 启动失败: {exc}，尝试 V4L2 ...")

    if HAS_REALSENSE and realsense_device_count() == 0:
        print("pyrealsense2 未枚举到设备，尝试 V4L2 ...")
        print_realsense_hint()

    try:
        return V4L2Capture(args)
    except RuntimeError as exc:
        print(f"V4L2 也失败: {exc}")
        raise SystemExit(1)


def main():
    args = parse_args()
    capture = open_capture(args)

    if not args.no_gui:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    frame_count = 0
    t0 = time.time()
    last_report = t0

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                continue

            frame_count += 1

            if args.no_gui:
                now = time.time()
                if now - last_report >= 2.0:
                    fps = frame_count / (now - t0)
                    print(f"已采集 {frame_count} 帧, 平均 {fps:.1f} fps")
                    last_report = now
                continue

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)")
    finally:
        capture.release()
        if not args.no_gui:
            cv2.destroyAllWindows()

        elapsed = time.time() - t0
        if elapsed > 0:
            print(f"结束: 共 {frame_count} 帧, 平均 {frame_count / elapsed:.1f} fps")
        else:
            print(f"结束: 共 {frame_count} 帧")


if __name__ == "__main__":
    main()
