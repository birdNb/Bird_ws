#!/usr/bin/env python3
"""YuNet 人脸检测子进程（OpenCV Python，规避 Jetson C++ DNN 崩溃）。"""
import os
import struct
import sys

import cv2

PROC_MAX_W = 960
DETECT_SCORE = 0.4
NMS_THRESH = 0.3
ROI_PAD = 0.30

ROOT = os.environ.get(
    "LOCATE_FACE_CPP_ROOT",
    os.path.expanduser("~/Bird_ws/locate_face_cpp"),
)
MODEL = os.path.join(ROOT, "model", "face_detection_yunet_2023mar.onnx")


def _proc_size(w, h, max_w):
    if w <= max_w:
        return w, h
    pw = max_w
    ph = max(1, int(h * max_w / w))
    return pw, ph


def _expand_roi(box, pad_ratio, fw, fh):
    x, y, bw, bh = box
    px = bw * pad_ratio
    py = bh * pad_ratio
    x1 = max(0, int(x - px))
    y1 = max(0, int(y - py))
    x2 = min(fw, int(x + bw + px))
    y2 = min(fh, int(y + bh + py))
    if x2 - x1 < 20 or y2 - y1 < 20:
        return None
    return x1, y1, x2, y2


class Detector:
    def __init__(self):
        if not os.path.isfile(MODEL):
            raise FileNotFoundError(f"model missing: {MODEL}")
        self._det = cv2.FaceDetectorYN.create(
            MODEL, "", (320, 320), DETECT_SCORE, NMS_THRESH, 5000
        )
        self._last_box = None

    def _detect(self, bgr, roi=None):
        h, w = bgr.shape[:2]
        inp = bgr
        off_x = off_y = 0
        if roi is not None:
            x1, y1, x2, y2 = roi
            inp = bgr[y1:y2, x1:x2]
            off_x, off_y = x1, y1
        self._det.setInputSize((inp.shape[1], inp.shape[0]))
        _, faces = self._det.detect(inp)
        if faces is None or len(faces) == 0:
            return None
        best = max(faces, key=lambda f: float(f[14]))
        if float(best[14]) < DETECT_SCORE:
            return None
        x, y, bw, bh = int(best[0]), int(best[1]), int(best[2]), int(best[3])
        return (x + off_x, y + off_y, bw, bh, float(best[14]))

    def process(self, bgr):
        h, w = bgr.shape[:2]
        pw, ph = _proc_size(w, h, PROC_MAX_W)
        if (pw, ph) != (w, h):
            proc = cv2.resize(bgr, (pw, ph), interpolation=cv2.INTER_AREA)
            sx = w / float(pw)
            sy = h / float(ph)
        else:
            proc = bgr
            sx = sy = 1.0

        box = self._detect(proc)
        if box is None and self._last_box is not None:
            lx, ly, lbw, lbh, _ = self._last_box
            roi = _expand_roi(
                (lx / sx, ly / sy, lbw / sx, lbh / sy), ROI_PAD, pw, ph
            )
            if roi is not None:
                box = self._detect(proc, roi)

        if box is None:
            return None

        x, y, bw, bh, score = box
        self._last_box = (x, y, bw, bh, score)
        fx = (x + bw / 2.0) * sx
        fy = (y + bh / 2.0) * sy
        cx_img = w / 2.0
        cy_img = h / 2.0
        dx_n = (fx - cx_img) / (w / 2.0)
        dy_n = (fy - cy_img) / (h / 2.0)
        return dx_n, dy_n, fx, fy


def main():
    det = Detector()
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
        import numpy as np

        bgr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
        result = det.process(bgr)
        if result is None:
            out.write(struct.pack("<B", 0))
        else:
            dx_n, dy_n, face_cx, face_cy = result
            out.write(struct.pack("<Bffff", 1, dx_n, dy_n, face_cx, face_cy))
        out.flush()


if __name__ == "__main__":
    main()
