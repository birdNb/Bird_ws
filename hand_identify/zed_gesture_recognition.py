#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================
# zed_gesture_recognition.py  ——  ZED Mini 手势数字(0-5) + 手掌3D位置 + 移动方向
#
# 架构:
#   ZED SDK 取左眼 RGB + 深度/点云 → MediaPipe Hands → 手势 / 3D位置 / 方向
#   画面叠加 + 终端彩色日志 (colorama)
#
# 运行:
#   python3 zed_gesture_recognition.py
#   python3 zed_gesture_recognition.py --no-gui
# ==============================================================

import argparse
import os
import time

import cv2
import mediapipe as mp
import numpy as np

try:
    import pyzed.sl as sl
except ImportError as exc:
    raise SystemExit(
        "缺少 pyzed, 请先安装 ZED SDK 并: pip install pyzed"
    ) from exc

from colorama import Fore, Style, init

init(autoreset=True)

# ----- MediaPipe -----
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# ----- 手势画面颜色 BGR -----
GESTURE_COLORS_BGR = [
    (0, 0, 255),      # 0 红
    (0, 165, 255),    # 1 橙
    (0, 255, 255),    # 2 黄
    (0, 255, 0),      # 3 绿
    (255, 0, 0),      # 4 蓝
    (255, 0, 255),    # 5 紫
]

# ----- 终端手势颜色 (colorama 无 ORANGE, 用 LIGHTYELLOW 代替) -----
GESTURE_TERM_COLORS = [
    Fore.RED,
    Fore.LIGHTRED_EX,
    Fore.LIGHTYELLOW_EX,
    Fore.GREEN,
    Fore.BLUE,
    Fore.MAGENTA,
]

MOVEMENT_THRESHOLD_M = 0.02   # 2cm 移动阈值
WINDOW_NAME = "ZED Mini Gesture"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def recognize_gesture(hand_landmarks):
    """根据 21 关键点识别手势数字 0-5 (伸直手指数)。"""
    finger_tips = [4, 8, 12, 16, 20]
    finger_pips = [3, 7, 11, 15, 19]

    wrist = hand_landmarks.landmark[0]
    thumb_tip = hand_landmarks.landmark[finger_tips[0]]
    thumb_ip = hand_landmarks.landmark[3]

    # 拇指: 根据手腕相对位置判断左右手
    if thumb_tip.x < wrist.x:
        thumb_up = thumb_tip.x < thumb_ip.x
    else:
        thumb_up = thumb_tip.x > thumb_ip.x

    fingers_up = [thumb_up]
    for i in range(1, 5):
        tip = hand_landmarks.landmark[finger_tips[i]]
        pip = hand_landmarks.landmark[finger_pips[i]]
        fingers_up.append(tip.y < pip.y)

    return sum(fingers_up), fingers_up


def calculate_palm_position(hand_landmarks, img_w, img_h, point_cloud):
    """手掌中心 3D 坐标 (米), 像素中心点。优先用 XYZ 点云。"""
    wrist = hand_landmarks.landmark[0]
    mid_base = hand_landmarks.landmark[9]
    cx_n = (wrist.x + mid_base.x) / 2.0
    cy_n = (wrist.y + mid_base.y) / 2.0
    px = int(clamp(cx_n * img_w, 0, img_w - 1))
    py = int(clamp(cy_n * img_h, 0, img_h - 1))

    err, pt = point_cloud.get_value(px, py)
    if err != sl.ERROR_CODE.SUCCESS:
        return None, (px, py)

    x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
    if np.isnan(x) or np.isnan(y) or np.isnan(z):
        return None, (px, py)
    return (x, y, z), (px, py)


class MovementTracker:
    """帧间手掌位移 → 方向文字。"""

    def __init__(self, threshold_m=MOVEMENT_THRESHOLD_M):
        self._prev = None
        self._threshold = threshold_m

    def reset(self):
        self._prev = None

    def update(self, pos_3d):
        if pos_3d is None:
            self._prev = None
            return "无深度"

        if self._prev is None:
            self._prev = pos_3d
            return "静止"

        dx = pos_3d[0] - self._prev[0]
        dy = pos_3d[1] - self._prev[1]
        dz = pos_3d[2] - self._prev[2]
        self._prev = pos_3d

        parts = []
        if abs(dx) > self._threshold:
            parts.append("右" if dx > 0 else "左")
        if abs(dy) > self._threshold:
            parts.append("下" if dy > 0 else "上")
        if abs(dz) > self._threshold:
            parts.append("后" if dz > 0 else "前")

        return "、".join(parts) if parts else "静止"


def draw_overlay_log(frame, text, position, color=(0, 255, 0),
                      font_scale=0.6, thickness=2):
    cv2.putText(
        frame, text, position, cv2.FONT_HERSHEY_SIMPLEX,
        font_scale, color, thickness, cv2.LINE_AA,
    )


def print_terminal_log(gesture, distance, direction):
    ts = time.strftime("%H:%M:%S")
    if gesture < 0:
        print(
            f"\r{Fore.CYAN}[{ts}] {Fore.WHITE}未检测到手",
            end="", flush=True,
        )
        return

    gcol = GESTURE_TERM_COLORS[min(gesture, 5)]
    dcol = Fore.GREEN if direction == "静止" else Fore.YELLOW
    print(
        f"\r{Fore.CYAN}[{ts}] "
        f"{gcol}手势: {gesture}{Style.RESET_ALL} "
        f"{Fore.WHITE}距离: {distance:.2f}m "
        f"{dcol}方向: {direction}",
        end="", flush=True,
    )


def open_zed_camera():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.QUALITY
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.3
    init_params.depth_maximum_distance = 3.0

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED 相机打开失败: {err}")
    return zed


def main():
    parser = argparse.ArgumentParser(description="ZED Mini 手势数字 + 3D 跟踪")
    parser.add_argument("--no-gui", action="store_true", help="不显示窗口")
    parser.add_argument(
        "--move-threshold", type=float, default=MOVEMENT_THRESHOLD_M,
        help="移动判定阈值(米), 默认 0.02",
    )
    args = parser.parse_args()

    if not args.no_gui and not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = ":0"

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )

    zed = open_zed_camera()
    image = sl.Mat()
    depth_map = sl.Mat()
    point_cloud = sl.Mat()

    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 50

    tracker = MovementTracker(threshold_m=args.move_threshold)

    if not args.no_gui:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print(Fore.GREEN + "系统启动成功！请将手放在相机前 0.3~3.0 米范围内。")
    print(Fore.YELLOW + "按 'q' 键退出。")

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame = image.get_data()
            if frame is None:
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            img_w = image.get_width()
            img_h = image.get_height()

            results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            gesture = -1
            distance = 0.0
            direction = "无手"
            palm_center_px = None
            palm_pos = None

            if results.multi_hand_landmarks:
                for hand_lm in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                        mp_drawing.DrawingSpec(
                            color=(121, 22, 76), thickness=2, circle_radius=4,
                        ),
                        mp_drawing.DrawingSpec(
                            color=(250, 44, 250), thickness=2, circle_radius=2,
                        ),
                    )

                    gesture, _ = recognize_gesture(hand_lm)
                    palm_pos, palm_center_px = calculate_palm_position(
                        hand_lm, img_w, img_h, point_cloud,
                    )

                    if palm_pos is not None:
                        distance = palm_pos[2]
                        direction = tracker.update(palm_pos)
                        if palm_center_px is not None:
                            cv2.circle(
                                frame, palm_center_px, 8, (0, 255, 0), -1,
                            )
                        draw_overlay_log(
                            frame, f"X: {palm_pos[0]:+.2f}m",
                            (10, 30), (255, 0, 0),
                        )
                        draw_overlay_log(
                            frame, f"Y: {palm_pos[1]:+.2f}m",
                            (10, 60), (0, 255, 0),
                        )
                        draw_overlay_log(
                            frame, f"Z: {palm_pos[2]:+.2f}m",
                            (10, 90), (0, 0, 255),
                        )
                    else:
                        direction = tracker.update(None)
            else:
                tracker.reset()
                direction = "无手"

            if gesture >= 0:
                col = GESTURE_COLORS_BGR[min(gesture, 5)]
                draw_overlay_log(
                    frame, f"Gesture: {gesture}", (10, 130),
                    col, font_scale=1.0, thickness=3,
                )
                dcol = (0, 255, 0) if direction == "静止" else (0, 255, 255)
                draw_overlay_log(
                    frame, f"Dir: {direction}", (10, 170), dcol,
                )
            else:
                draw_overlay_log(
                    frame, "No hand", (10, 130), (128, 128, 128),
                )

            print_terminal_log(gesture, distance, direction)

            if not args.no_gui:
                cv2.imshow(WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print(Fore.YELLOW + "\n用户中断")

    finally:
        zed.close()
        hands.close()
        if not args.no_gui:
            cv2.destroyAllWindows()
        print(Fore.GREEN + "\n程序已退出")


if __name__ == "__main__":
    main()
