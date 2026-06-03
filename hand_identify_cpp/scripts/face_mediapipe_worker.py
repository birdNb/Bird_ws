#!/usr/bin/env python3
"""MediaPipe 人脸检测子进程（与 locate_face.py 同算法），供 C++ FaceTracker 调用。"""
import struct
import sys

import cv2
import mediapipe as mp

DETECT_CONFIDENCE = 0.4
TRACK_CONFIDENCE = 0.5
ROI_PAD_RATIO = 0.30
PROC_MAX_W = 640


def _compute_proc_size(src_w, src_h, max_w):
    if src_w <= max_w:
        return src_w, src_h
    proc_w = max_w
    proc_h = max(1, int(src_h * max_w / src_w))
    return proc_w, proc_h


def _face_bbox_from_landmarks(landmarks, w, h, pad=12):
    xs = [p.x for p in landmarks.landmark]
    ys = [p.y for p in landmarks.landmark]
    x1 = max(0, int(min(xs) * w) - pad)
    y1 = max(0, int(min(ys) * h) - pad)
    x2 = min(w - 1, int(max(xs) * w) + pad)
    y2 = min(h - 1, int(max(ys) * h) + pad)
    return x1, y1, x2, y2


def _detect_face_roi_bbox(face_detector, rgb, w, h, pad_ratio=ROI_PAD_RATIO):
    det = face_detector.process(rgb)
    if not det.detections:
        return None
    best = max(det.detections, key=lambda d: d.score[0])
    rel = best.location_data.relative_bounding_box
    bx = rel.xmin * w
    by = rel.ymin * h
    bw = rel.width * w
    bh = rel.height * h
    pad_x = bw * pad_ratio
    pad_y = bh * pad_ratio
    x1 = max(0, int(bx - pad_x))
    y1 = max(0, int(by - pad_y))
    x2 = min(w, int(bx + bw + pad_x))
    y2 = min(h, int(by + bh + pad_y))
    if x2 - x1 < 20 or y2 - y1 < 20:
        return None
    return x1, y1, x2, y2


class _Detector:
    def __init__(self):
        mp_face_mesh = mp.solutions.face_mesh
        self._mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=DETECT_CONFIDENCE,
            min_tracking_confidence=TRACK_CONFIDENCE,
        )
        self._mesh_roi = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=DETECT_CONFIDENCE,
            min_tracking_confidence=TRACK_CONFIDENCE,
        )
        mp_face_detection = mp.solutions.face_detection
        self._face_det = mp_face_detection.FaceDetection(
            model_selection=1,
            min_detection_confidence=DETECT_CONFIDENCE,
        )

    def process(self, bgr):
        h, w = bgr.shape[:2]
        proc_w, proc_h = _compute_proc_size(w, h, PROC_MAX_W)
        if (proc_w, proc_h) != (w, h):
            proc_bgr = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
        else:
            proc_bgr = bgr

        rgb = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        res = self._mesh.process(rgb)

        if not res.multi_face_landmarks:
            bbox = _detect_face_roi_bbox(self._face_det, rgb, proc_w, proc_h)
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                roi = rgb[y1:y2, x1:x2].copy()
                roi.flags.writeable = False
                res2 = self._mesh_roi.process(roi)
                if res2.multi_face_landmarks:
                    rw_roi = x2 - x1
                    rh_roi = y2 - y1
                    for lms in res2.multi_face_landmarks:
                        for lm in lms.landmark:
                            lm.x = (lm.x * rw_roi + x1) / proc_w
                            lm.y = (lm.y * rh_roi + y1) / proc_h
                    res = res2

        if not res.multi_face_landmarks:
            return None

        lm = res.multi_face_landmarks[0]
        fx1, fy1, fx2, fy2 = _face_bbox_from_landmarks(lm, proc_w, proc_h, pad=12)
        cx_proc = (fx1 + fx2) / 2.0
        cy_proc = (fy1 + fy2) / 2.0
        sx = w / float(proc_w)
        sy = h / float(proc_h)
        face_cx = cx_proc * sx
        face_cy = cy_proc * sy
        cx_img = w / 2.0
        cy_img = h / 2.0
        dx_n = (face_cx - cx_img) / (w / 2.0)
        dy_n = (face_cy - cy_img) / (h / 2.0)
        return dx_n, dy_n, face_cx, face_cy


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
