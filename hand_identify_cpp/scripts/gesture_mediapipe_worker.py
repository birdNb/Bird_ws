#!/usr/bin/env python3
"""MediaPipe 手势 0~5 子进程（算法对齐 zed_gesture_recognition.py）。"""
import struct
import sys
from collections import Counter

import cv2
import mediapipe as mp
import numpy as np

PROC_MAX_W = 560
GESTURE_SMOOTH_FRAMES = 3
THUMB_EXTEND_MIN = 0.04
FINGER_WRIST_RATIO = 1.02
PINKY_WRIST_RATIO = 1.01
THUMB_WRIST_RATIO = 1.02
HAND_MIN_AREA_RATIO = 0.0008
HAND_MAX_AREA_RATIO = 0.75
MP_DETECT_CONF = 0.4
MP_TRACK_CONF = 0.35


def _dist2(a, b):
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2


def _is_right_hand(lm, handedness_label):
    if handedness_label in ("Left", "Right"):
        return handedness_label == "Right"
    return lm[5].x < lm[17].x


def _finger_extended(lm, tip_id, pip_id, wrist_ratio):
    wrist = lm[0]
    tip, pip = lm[tip_id], lm[pip_id]
    tip_d = _dist2(tip, wrist)
    pip_d = _dist2(pip, wrist)
    if tip_d > pip_d * wrist_ratio:
        return True
    mcp_id = pip_id - 1
    mcp = lm[mcp_id]
    return tip.y < pip.y and pip.y < mcp.y and tip_d > pip_d * 0.98


def _thumb_extended(lm, is_right):
    wrist = lm[0]
    tip, ip, mcp = lm[4], lm[3], lm[2]
    if _dist2(tip, wrist) > _dist2(ip, wrist) * THUMB_WRIST_RATIO:
        return True
    spread = (tip.x - ip.x) if is_right else (ip.x - tip.x)
    if spread < THUMB_EXTEND_MIN:
        return False
    if is_right:
        return tip.x > ip.x and tip.x > mcp.x
    return tip.x < ip.x and tip.x < mcp.x


def recognize_gesture(hand_landmarks, handedness_label=None):
    lm = hand_landmarks.landmark
    is_right = _is_right_hand(lm, handedness_label)
    chains = [
        (8, 6, FINGER_WRIST_RATIO),
        (12, 10, FINGER_WRIST_RATIO),
        (16, 14, FINGER_WRIST_RATIO),
        (20, 18, PINKY_WRIST_RATIO),
    ]
    fingers_up = [_thumb_extended(lm, is_right)]
    for tip_id, pip_id, ratio in chains:
        fingers_up.append(_finger_extended(lm, tip_id, pip_id, ratio))
    return sum(fingers_up), fingers_up


class GestureSmoother:
    def __init__(self, window=GESTURE_SMOOTH_FRAMES):
        self._hist = []
        self._window = max(1, window)

    def reset(self):
        self._hist.clear()

    def update(self, raw_gesture):
        if raw_gesture < 0:
            self.reset()
            return -1
        self._hist.append(raw_gesture)
        if len(self._hist) > self._window:
            self._hist.pop(0)
        return Counter(self._hist).most_common(1)[0][0]


def _landmarks_frame_xy(hand_lm, proc_w, proc_h, frame_w, frame_h):
    sx = frame_w / float(proc_w)
    sy = frame_h / float(proc_h)
    pts = []
    for p in hand_lm.landmark:
        pts.append(float(p.x * proc_w * sx))
        pts.append(float(p.y * proc_h * sy))
    return pts


def _landmark_bbox(lm, proc_w, proc_h, frame_w, frame_h, pad=8):
    xs = [p.x for p in lm.landmark]
    ys = [p.y for p in lm.landmark]
    x1 = max(0, int(min(xs) * proc_w) - pad)
    y1 = max(0, int(min(ys) * proc_h) - pad)
    x2 = min(proc_w - 1, int(max(xs) * proc_w) + pad)
    y2 = min(proc_h - 1, int(max(ys) * proc_h) + pad)
    sx = frame_w / float(proc_w)
    sy = frame_h / float(proc_h)
    return (
        int(x1 * sx),
        int(y1 * sy),
        int((x2 - x1) * sx),
        int((y2 - y1) * sy),
    )


def _estimate_distance_m(bbox_h, frame_h):
    if bbox_h <= 0 or frame_h <= 0:
        return 0.5
    ratio = bbox_h / float(frame_h)
    dist = 0.12 / max(ratio, 0.05)
    return float(max(0.2, min(2.0, dist)))


def _in_range_from_bbox(bw, bh, frame_w, frame_h):
    area = bw * bh
    frame_area = frame_w * frame_h
    if frame_area <= 0:
        return False
    ratio = area / frame_area
    return HAND_MIN_AREA_RATIO <= ratio <= HAND_MAX_AREA_RATIO


class _Detector:
    def __init__(self):
        mp_hands = mp.solutions.hands
        self._hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=MP_DETECT_CONF,
            min_tracking_confidence=MP_TRACK_CONF,
        )
        self._smoother = GestureSmoother()

    def process(self, bgr):
        h, w = bgr.shape[:2]
        if w > PROC_MAX_W:
            proc_w = PROC_MAX_W
            proc_h = max(1, int(h * PROC_MAX_W / w))
            proc_bgr = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
        else:
            proc_w, proc_h = w, h
            proc_bgr = bgr

        rgb = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._hands.process(rgb)

        if not results.multi_hand_landmarks:
            self._smoother.reset()
            return None

        handedness_list = results.multi_handedness or []
        best_idx = 0
        best_area = -1
        for idx, hand_lm in enumerate(results.multi_hand_landmarks):
            bx, by, bw, bh = _landmark_bbox(hand_lm, proc_w, proc_h, w, h)
            area = bw * bh
            if area > best_area:
                best_area = area
                best_idx = idx

        hand_lm = results.multi_hand_landmarks[best_idx]

        h_label = None
        if best_idx < len(handedness_list):
            h_label = handedness_list[best_idx].classification[0].label

        raw_gesture, _ = recognize_gesture(hand_lm, handedness_label=h_label)
        gesture = self._smoother.update(raw_gesture)

        x, y, bw, bh = _landmark_bbox(hand_lm, proc_w, proc_h, w, h)
        cx = x + bw / 2.0
        cy = y + bh / 2.0
        cx_img = w / 2.0
        cy_img = h / 2.0
        dx_n = (cx - cx_img) / (w / 2.0)
        dy_n = (cy - cy_img) / (h / 2.0)
        dist_m = _estimate_distance_m(bh, h)
        # 仅五指(G5)做 bbox 距离门控；0~4 不判距（对齐 zed 深度距仅跟手场景）
        if gesture == 5:
            in_range = _in_range_from_bbox(bw, bh, w, h)
        else:
            in_range = True

        return {
            "gesture": gesture,
            "raw": raw_gesture,
            "in_range": in_range,
            "dx_n": dx_n,
            "dy_n": dy_n,
            "distance_m": dist_m,
            "bbox": (x, y, bw, bh),
            "landmarks_xy": _landmarks_frame_xy(hand_lm, proc_w, proc_h, w, h),
        }


def main():
    det = _Detector()
    out = sys.stdout.buffer
    out.write(b"READY\n")
    out.flush()
    while True:
        hdr = sys.stdin.buffer.read(8)
        if len(hdr) < 8:
            break
        w, h = struct.unpack("<II", hdr)
        if w <= 0 or h <= 0 or w > 4096 or h > 4096:
            break
        nbytes = w * h * 3
        raw = sys.stdin.buffer.read(nbytes)
        if len(raw) < nbytes:
            break
        bgr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
        result = det.process(bgr)
        if result is None:
            out.write(struct.pack("<B", 0))
        else:
            g = result["gesture"]
            rg = result["raw"]
            x, y, bw, bh = result["bbox"]
            # BbbB + 3×float + 4×int32 + 21×(x,y) float
            lm = result["landmarks_xy"]
            out.write(
                struct.pack(
                    "<BbbBfffiiii" + "f" * len(lm),
                    1,
                    g if g >= 0 else -1,
                    rg if rg >= 0 else -1,
                    1 if result["in_range"] else 0,
                    result["dx_n"],
                    result["dy_n"],
                    result["distance_m"],
                    x,
                    y,
                    bw,
                    bh,
                    *lm,
                )
            )
        out.flush()


if __name__ == "__main__":
    main()
