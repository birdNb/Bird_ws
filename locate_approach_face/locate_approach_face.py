#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================
# locate_approach_face.py  ——  人物接近 + 人脸脖子跟踪 (两阶段)
#
# 阶段 1 APPROACH: Pose 估距/跟随; Pose 丢失时懒加载 Face Detection 用人脸估距继续跟随
# 阶段 2 FACE_TRACK: 停车,脖子 yaw/pitch 跟踪人脸
#
# GPU 优化: 同一时刻只运行 Pose 或 Face 之一,切换时 close() 释放旧模型
#
# 运行:
#   python3 locate_approach_face.py                  # dry-run
#   python3 locate_approach_face.py --enable-motion  # 实际驱动
# ==============================================================

import argparse
import enum
import math
import os
import subprocess
import threading
import time
from typing import Optional

if not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":0"
if not os.environ.get("XAUTHORITY"):
    _xauth = os.path.expanduser("~/.Xauthority")
    if os.path.exists(_xauth):
        os.environ["XAUTHORITY"] = _xauth

import cv2
import numpy as np
import mediapipe as mp

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# =====================================================
#                    可调参数
# =====================================================

CMD_VEL_TOPIC = "/cmd_vel"
ABSOLUTE_TOPIC = "/pi_plus_absolute"
HEAD_YAW_JOINT = "head_yaw_joint"
HEAD_PITCH_JOINT = "head_pitch_joint"

WIDTH, HEIGHT = 2560, 720
TARGET_FPS = 30
CAM_ID = 0
USE_MJPG = False
ZED_STEREO = True
PROC_MAX_W = 960

FULLSCREEN = True
WINDOW_NAME = "Approach + Face Track (Orin Nano)"

# ----- Pose (APPROACH) -----
POSE_MODEL_COMPLEXITY = 0
POSE_DET_CONFIDENCE = 0.5
POSE_TRK_CONFIDENCE = 0.5
KP_VIS_TH = 0.45
FOCAL_PX_BASE = 640.0
FOCAL_PX_BASE_AT_W = 1280
SHOULDER_REAL_M = 0.40
TORSO_REAL_M = 0.50
FACE_REAL_M = 0.16          # 成年人平均脸宽(用于 Pose 丢失时估距)
TARGET_DISTANCE_M = 1.0
DIST_DEAD_M = 0.20
ANG_DEAD = 0.05
# 距离误差(m) -> cmd_vel.linear.x 归一化指令,限幅 [-1, 1]
K_LIN = 0.50
MAX_LIN_CMD = 1.0
LIN_EMA_ALPHA = 0.4
DIST_EMA_ALPHA = 0.35
BASE_PUBLISH_RATE_HZ = 20
# 连续 N 帧在距离/方向死区内才切换到人脸跟踪(防抖)
APPROACH_STABLE_FRAMES = 8

# ----- Face (FACE_TRACK) -----
DETECT_CONFIDENCE = 0.4
TRACK_CONFIDENCE = 0.5
ROI_PAD_RATIO = 0.30
DEAD_BAND_X = 0.04
DEAD_BAND_Y = 0.05
K_YAW_DEG = 20.0
K_PITCH_DEG = 15.0
MAX_STEP_YAW_DEG = 6.0
MAX_STEP_PITCH_DEG = 5.0
TARGET_EMA_ALPHA = 0.6
YAW_LIMIT_DEG = 80.0
PITCH_UP_DEG = -40.0
PITCH_DOWN_DEG = 60.0
NECK_PUBLISH_RATE_HZ = 50
ENABLE_VEL_FEEDFORWARD = False
NO_FACE_RETURN_HOME_SEC = 2.0
RETURN_HOME_RATE_DEG_PER_SEC = 30.0
# 人物/人脸丢失后底盘保持静止,不自动重新接近

# ----- FSM -----
FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5
FSM_WAIT_TIMEOUT = 30.0

# Pose keypoints
KP_NOSE = 0
KP_L_SHOULDER = 11
KP_R_SHOULDER = 12
KP_L_HIP = 23
KP_R_HIP = 24
BODY_KP_IDX = [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]


class Phase(enum.Enum):
    APPROACH = "APPROACH"
    FACE_TRACK = "FACE_TRACK"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    if USE_MJPG:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def detect_screen_size(default=(1920, 1080)):
    try:
        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        out = subprocess.check_output(
            ["xrandr"], env=env, stderr=subprocess.DEVNULL, timeout=2,
        ).decode()
        for line in out.splitlines():
            if "*" in line:
                token = line.strip().split()[0]
                w, h = token.split("x")
                return int(w), int(h)
    except Exception:
        pass
    return default


def fit_letterbox(img, target_w, target_h):
    src_h, src_w = img.shape[:2]
    scale = min(target_w / src_w, target_h / src_h)
    new_w, new_h = int(src_w * scale), int(src_h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    x = (target_w - new_w) // 2
    y = (target_h - new_h) // 2
    canvas[y:y + new_h, x:x + new_w] = resized
    return canvas


def compute_proc_size(src_w, src_h, max_w):
    if src_w <= max_w:
        return src_w, src_h
    scale = max_w / src_w
    return int(src_w * scale), int(src_h * scale)


class FsmStateMonitor:
    _NAME_MAP = {
        0: "INIT", 1: "ERROR",
        2: "CANDIDATE_DEFAULT", 3: "CANDIDATE_CUSTOM",
        4: "CANDIDATE_REMOTE",
        5: "EXEC_DEFAULT", 6: "EXEC_CUSTOM", 7: "EXEC_REMOTE",
        8: "PROTECTION_SHUTDOWN",
        9: "CANDIDATE_CALIBRATION", 10: "EXEC_CALIBRATING",
        11: "EXEC_CALIB_OK", 12: "EXEC_CALIB_FAILED",
        13: "CANDIDATE_TEACHING", 14: "EXEC_TEACHING",
        15: "CANDIDATE_DEVELOP", 16: "EXEC_DEVELOP",
    }

    def __init__(self, topic: str = FSM_STATE_TOPIC):
        self._lock = threading.Lock()
        self._state = None
        self._sub = rospy.Subscriber(topic, Int32, self._cb, queue_size=10)

    def _cb(self, msg):
        with self._lock:
            self._state = int(msg.data)

    @property
    def state(self):
        with self._lock:
            return self._state

    @classmethod
    def state_name(cls, v) -> str:
        return cls._NAME_MAP.get(v, f"UNKNOWN({v})")

    def is_exec_default(self) -> bool:
        return self.state == FSM_EXEC_DEFAULT

    def wait_for_exec_default(self, timeout: float = FSM_WAIT_TIMEOUT) -> bool:
        deadline = time.time() + timeout
        last_log_t = 0.0
        warned_timeout = False
        while not rospy.is_shutdown():
            s = self.state
            if s == FSM_EXEC_DEFAULT:
                return True
            now = time.time()
            if now - last_log_t >= 1.0:
                if s is None:
                    rospy.logwarn(
                        "[FSM] 还没收到 %s, 请确认 sim2real_master 已启动",
                        FSM_STATE_TOPIC,
                    )
                else:
                    rospy.logwarn(
                        "[FSM] 当前状态 %s(%d) != EXEC_DEFAULT(5)",
                        self.state_name(s), s,
                    )
                last_log_t = now
            if not warned_timeout and now > deadline:
                rospy.logerr(
                    "[FSM] 等待 %.0fs 仍未进入 EXEC_DEFAULT,继续等...",
                    timeout,
                )
                warned_timeout = True
            time.sleep(0.1)
        return False


class VelCommand:
    def __init__(self):
        self._lock = threading.Lock()
        self._lin = 0.0
        self._ang = 0.0
        self._stale_after = 0.0
        self._t = time.time()

    def set(self, lin: float, ang: float, valid_for_sec: float = 0.5):
        with self._lock:
            self._lin = lin
            self._ang = ang
            self._t = time.time()
            self._stale_after = valid_for_sec

    def get(self):
        with self._lock:
            if self._stale_after > 0 and time.time() - self._t > self._stale_after:
                return 0.0, 0.0, True
            return self._lin, self._ang, False

    def stop(self):
        self.set(0.0, 0.0, valid_for_sec=0.0)


class NeckTarget:
    def __init__(self):
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._pitch = 0.0
        self._updated_t = time.time()

    def set(self, yaw_rad: float, pitch_rad: float):
        with self._lock:
            self._yaw = yaw_rad
            self._pitch = pitch_rad
            self._updated_t = time.time()

    def get(self):
        with self._lock:
            return self._yaw, self._pitch


class BaseController(threading.Thread):
    def __init__(self, vel: VelCommand, fsm: Optional[FsmStateMonitor], dry_run: bool):
        super().__init__(daemon=True)
        self._vel = vel
        self._fsm = fsm
        self._dry_run = dry_run
        self._pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self._rate = rospy.Rate(BASE_PUBLISH_RATE_HZ)
        self._stop_evt = threading.Event()
        self._enabled = True

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if not enabled:
            self._vel.stop()

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def num_subscribers(self) -> int:
        return self._pub.get_num_connections()

    def stop(self):
        self._stop_evt.set()

    def publish_stop_blocking(self, duration: float = 0.5):
        msg = Twist()
        end_t = time.time() + duration
        while time.time() < end_t:
            try:
                if not self._dry_run:
                    self._pub.publish(msg)
            except Exception:
                break
            time.sleep(1.0 / max(BASE_PUBLISH_RATE_HZ, 1))

    def run(self):
        msg = Twist()
        warned_no_fsm = False
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            if not self._enabled:
                lin, ang = 0.0, 0.0
            else:
                lin, ang, _ = self._vel.get()
            fsm_ok = (self._fsm is None) or self._fsm.is_exec_default()
            if not fsm_ok:
                lin, ang = 0.0, 0.0
                if not warned_no_fsm:
                    rospy.logwarn_throttle(
                        2.0, "[base] FSM=%s,暂停 cmd_vel",
                        FsmStateMonitor.state_name(
                            self._fsm.state if self._fsm else -1,
                        ),
                    )
                    warned_no_fsm = True
            else:
                warned_no_fsm = False
            msg.linear.x = clamp(lin, -MAX_LIN_CMD, MAX_LIN_CMD)
            msg.angular.z = 0.0   # 底盘不转弯,左右靠脖子
            try:
                if not self._dry_run:
                    self._pub.publish(msg)
            except Exception as e:
                rospy.logerr_throttle(2.0, "[base] publish 异常: %s", e)
            self._rate.sleep()


class NeckController(threading.Thread):
    def __init__(self, target: NeckTarget, fsm: Optional[FsmStateMonitor]):
        super().__init__(daemon=True)
        self._target = target
        self._fsm = fsm
        self._stop_evt = threading.Event()
        self._pub = rospy.Publisher(ABSOLUTE_TOPIC, JointState, queue_size=10)
        self._rate = rospy.Rate(NECK_PUBLISH_RATE_HZ)
        self._enabled = False

    def set_enabled(self, enabled: bool):
        self._enabled = enabled

    @property
    def num_subscribers(self) -> int:
        return self._pub.get_num_connections()

    def stop(self):
        self._stop_evt.set()

    def publish_center_blocking(self, duration: float = 0.5):
        msg = JointState()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.position = [0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        end_t = time.time() + duration
        while time.time() < end_t:
            msg.header.stamp = rospy.Time.now()
            try:
                self._pub.publish(msg)
            except Exception:
                break
            time.sleep(1.0 / max(NECK_PUBLISH_RATE_HZ, 1))

    def run(self):
        msg = JointState()
        msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
        msg.velocity = []
        msg.effort = []
        warned_no_fsm = False
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            if not self._enabled:
                self._rate.sleep()
                continue
            if self._fsm is not None and not self._fsm.is_exec_default():
                if not warned_no_fsm:
                    rospy.logwarn_throttle(
                        2.0, "[neck] FSM=%s,暂停脖子下发",
                        FsmStateMonitor.state_name(self._fsm.state),
                    )
                    warned_no_fsm = True
                self._rate.sleep()
                continue
            warned_no_fsm = False
            yaw, pitch = self._target.get()
            msg.position = [yaw, pitch]
            if ENABLE_VEL_FEEDFORWARD:
                msg.velocity = [0.0, 0.0]
            msg.header.stamp = rospy.Time.now()
            try:
                self._pub.publish(msg)
            except Exception as e:
                rospy.logerr_throttle(2.0, "[neck] publish 异常: %s", e)
            self._rate.sleep()


class PosePipeline:
    """APPROACH 阶段专用,close() 后释放 GPU 资源。"""

    def __init__(self):
        self._pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=POSE_MODEL_COMPLEXITY,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=POSE_DET_CONFIDENCE,
            min_tracking_confidence=POSE_TRK_CONFIDENCE,
        )

    def close(self):
        if self._pose is not None:
            self._pose.close()
            self._pose = None

    def process(self, rgb):
        if self._pose is None:
            return None
        return self._pose.process(rgb)


class FaceDetectPipeline:
    """APPROACH 阶段 Pose 丢失时的轻量回退,仅 Face Detection(省 GPU)。"""

    def __init__(self):
        mp_face_detection = mp.solutions.face_detection
        self._detector = mp_face_detection.FaceDetection(
            model_selection=1,
            min_detection_confidence=DETECT_CONFIDENCE,
        )

    def close(self):
        if self._detector is not None:
            self._detector.close()
            self._detector = None

    def detect_best(self, rgb, proc_w, proc_h, disp_w, disp_h, focal_px):
        """返回 (cx, cy, bbox, dist_m) 或 None。"""
        if self._detector is None:
            return None
        det = self._detector.process(rgb)
        if not det.detections:
            return None
        best = max(det.detections, key=lambda d: d.score[0])
        rel = best.location_data.relative_bounding_box
        bx = rel.xmin * proc_w
        by = rel.ymin * proc_h
        bw = rel.width * proc_w
        bh = rel.height * proc_h
        if bw < 8 or bh < 8:
            return None
        sx = disp_w / proc_w
        sy = disp_h / proc_h
        x1 = max(0, int(bx * sx))
        y1 = max(0, int(by * sy))
        x2 = min(disp_w - 1, int((bx + bw) * sx))
        y2 = min(disp_h - 1, int((by + bh) * sy))
        face_w = bw * sx
        dist_m = focal_px * FACE_REAL_M / face_w
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        return cx, cy, (x1, y1, x2, y2), dist_m


class FacePipeline:
    """FACE_TRACK 阶段专用,懒加载。"""

    def __init__(self):
        mp_face_mesh = mp.solutions.face_mesh
        self._face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=DETECT_CONFIDENCE,
            min_tracking_confidence=TRACK_CONFIDENCE,
        )
        self._face_mesh_roi = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=DETECT_CONFIDENCE,
            min_tracking_confidence=TRACK_CONFIDENCE,
        )
        mp_face_detection = mp.solutions.face_detection
        self._face_detector = mp_face_detection.FaceDetection(
            model_selection=1,
            min_detection_confidence=DETECT_CONFIDENCE,
        )

    def close(self):
        for attr in ("_face_mesh", "_face_mesh_roi", "_face_detector"):
            obj = getattr(self, attr, None)
            if obj is not None:
                obj.close()
                setattr(self, attr, None)

    def process(self, rgb, proc_w, proc_h):
        if self._face_mesh is None:
            return None, False
        rgb.flags.writeable = False
        res = self._face_mesh.process(rgb)
        used_roi = False
        if not res.multi_face_landmarks and self._face_detector is not None:
            det = self._face_detector.process(rgb)
            if det.detections:
                best = max(det.detections, key=lambda d: d.score[0])
                rel = best.location_data.relative_bounding_box
                bx, by = rel.xmin * proc_w, rel.ymin * proc_h
                bw, bh = rel.width * proc_w, rel.height * proc_h
                pad_x, pad_y = bw * ROI_PAD_RATIO, bh * ROI_PAD_RATIO
                x1 = max(0, int(bx - pad_x))
                y1 = max(0, int(by - pad_y))
                x2 = min(proc_w, int(bx + bw + pad_x))
                y2 = min(proc_h, int(by + bh + pad_y))
                if x2 - x1 >= 20 and y2 - y1 >= 20:
                    roi = rgb[y1:y2, x1:x2].copy()
                    roi.flags.writeable = False
                    res2 = self._face_mesh_roi.process(roi)
                    if res2.multi_face_landmarks:
                        rw_roi, rh_roi = x2 - x1, y2 - y1
                        for lms in res2.multi_face_landmarks:
                            for lm in lms.landmark:
                                lm.x = (lm.x * rw_roi + x1) / proc_w
                                lm.y = (lm.y * rh_roi + y1) / proc_h
                        res = res2
                        used_roi = True
        return res, used_roi


def _kp_visible(lm, idx) -> bool:
    if idx >= len(lm.landmark):
        return False
    return lm.landmark[idx].visibility >= KP_VIS_TH


def _kp_pix(lm, idx, w, h):
    p = lm.landmark[idx]
    return p.x * w, p.y * h


def estimate_distance(lm, w, h, focal_px):
    have_sh = _kp_visible(lm, KP_L_SHOULDER) and _kp_visible(lm, KP_R_SHOULDER)
    have_hip = _kp_visible(lm, KP_L_HIP) and _kp_visible(lm, KP_R_HIP)
    d_sh = d_torso = None
    if have_sh:
        lsx, _ = _kp_pix(lm, KP_L_SHOULDER, w, h)
        rsx, _ = _kp_pix(lm, KP_R_SHOULDER, w, h)
        sh_pix = abs(lsx - rsx)
        if sh_pix >= 8:
            d_sh = focal_px * SHOULDER_REAL_M / sh_pix
    if have_sh and have_hip:
        _, lsy = _kp_pix(lm, KP_L_SHOULDER, w, h)
        _, rsy = _kp_pix(lm, KP_R_SHOULDER, w, h)
        _, lhy = _kp_pix(lm, KP_L_HIP, w, h)
        _, rhy = _kp_pix(lm, KP_R_HIP, w, h)
        torso_pix = abs((lhy + rhy) / 2.0 - (lsy + rsy) / 2.0)
        if torso_pix >= 12:
            d_torso = focal_px * TORSO_REAL_M / torso_pix
    if d_sh is None and d_torso is None:
        return None, "?"
    if d_sh is None:
        return d_torso, "torso"
    if d_torso is None:
        return d_sh, "shoulder"
    return 0.5 * (d_sh + d_torso), "blend"


def body_center_and_bbox(lm, w, h):
    have_sh = _kp_visible(lm, KP_L_SHOULDER) and _kp_visible(lm, KP_R_SHOULDER)
    if have_sh:
        lsx, lsy = _kp_pix(lm, KP_L_SHOULDER, w, h)
        rsx, rsy = _kp_pix(lm, KP_R_SHOULDER, w, h)
        cx, cy = (lsx + rsx) / 2.0, (lsy + rsy) / 2.0
    elif _kp_visible(lm, KP_NOSE):
        cx, cy = _kp_pix(lm, KP_NOSE, w, h)
    else:
        return None, None
    xs, ys = [], []
    for i in BODY_KP_IDX:
        if i < len(lm.landmark) and _kp_visible(lm, i):
            p = lm.landmark[i]
            xs.append(p.x * w)
            ys.append(p.y * h)
    if len(xs) < 2:
        return (cx, cy), None
    pad = 18
    bbox = (
        max(0, int(min(xs)) - pad), max(0, int(min(ys)) - pad),
        min(w - 1, int(max(xs)) + pad), min(h - 1, int(max(ys)) + pad),
    )
    return (cx, cy), bbox


def face_bbox_from_landmarks(landmarks, w, h, pad=10):
    xs = [p.x for p in landmarks.landmark]
    ys = [p.y for p in landmarks.landmark]
    x1 = max(0, int(min(xs) * w) - pad)
    y1 = max(0, int(min(ys) * h) - pad)
    x2 = min(w - 1, int(max(xs) * w) + pad)
    y2 = min(h - 1, int(max(ys) * h) + pad)
    return x1, y1, x2, y2


def update_neck_from_error(yaw_cur_rad, pitch_cur_rad, dx_n, dy_n, state):
    if abs(dx_n) < DEAD_BAND_X:
        dx_n = 0.0
    if abs(dy_n) < DEAD_BAND_Y:
        dy_n = 0.0
    delta_yaw_deg = clamp(-K_YAW_DEG * dx_n, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG)
    delta_pitch_deg = clamp(
        K_PITCH_DEG * dy_n, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG,
    )
    base_yaw = state.get("yaw_rad", yaw_cur_rad)
    base_pitch = state.get("pitch_rad", pitch_cur_rad)
    raw_yaw = base_yaw + math.radians(delta_yaw_deg)
    raw_pitch = base_pitch + math.radians(delta_pitch_deg)
    a = TARGET_EMA_ALPHA
    yaw_new = base_yaw * (1 - a) + raw_yaw * a
    pitch_new = base_pitch * (1 - a) + raw_pitch * a
    yaw_new = clamp(yaw_new, -math.radians(YAW_LIMIT_DEG), math.radians(YAW_LIMIT_DEG))
    pitch_new = clamp(
        pitch_new,
        math.radians(PITCH_UP_DEG),
        math.radians(PITCH_DOWN_DEG),
    )
    state["yaw_rad"] = yaw_new
    state["pitch_rad"] = pitch_new
    return yaw_new, pitch_new


def switch_to_face_track(pose_pipe, face_detect_pipe, face_pipe_holder, vel_cmd,
                         base_ctrl, neck_ctrl, stable_dist):
    rospy.loginfo(
        "[phase] APPROACH -> FACE_TRACK  (dist=%.2fm 稳定 %d 帧)",
        stable_dist, APPROACH_STABLE_FRAMES,
    )
    vel_cmd.stop()
    base_ctrl.set_enabled(False)
    if pose_pipe is not None:
        pose_pipe.close()
    if face_detect_pipe is not None:
        face_detect_pipe.close()
    if face_pipe_holder[0] is None:
        rospy.loginfo("[gpu] 懒加载 Face Mesh / Detection...")
        face_pipe_holder[0] = FacePipeline()
    neck_ctrl.set_enabled(True)
    return Phase.FACE_TRACK, None, None


def main():
    parser = argparse.ArgumentParser(
        description="人物接近 + 到达距离后脖子跟踪人脸",
    )
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--no-fsm", action="store_true")
    parser.add_argument("--focal-px", type=float, default=None)
    parser.add_argument(
        "--target-dist", type=float, default=TARGET_DISTANCE_M,
        help=f"目标距离米(默认 {TARGET_DISTANCE_M})",
    )
    args = parser.parse_args()

    dry_run = not args.enable_motion
    target_dist_m = args.target_dist
    focal_base = args.focal_px if args.focal_px is not None else FOCAL_PX_BASE

    rospy.init_node("locate_approach_face", anonymous=False)

    cap = open_camera()
    if not cap.isOpened():
        rospy.logerr("[cam] 无法打开相机 /dev/video%d", CAM_ID)
        return
    raw_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    raw_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_w = raw_w // 2 if ZED_STEREO else raw_w
    real_h = raw_h
    proc_w, proc_h = compute_proc_size(real_w, real_h, PROC_MAX_W)
    focal_px_display = focal_base * (real_w / FOCAL_PX_BASE_AT_W)
    rospy.loginfo(
        "[cam] 原始 %dx%d  左眼 %dx%d  proc %dx%d  fx=%.1f",
        raw_w, raw_h, real_w, real_h, proc_w, proc_h, focal_px_display,
    )

    screen_w = screen_h = 0
    is_fullscreen = FULLSCREEN
    if not args.no_gui:
        screen_w, screen_h = detect_screen_size()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        if FULLSCREEN:
            cv2.setWindowProperty(
                WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN,
            )

    fsm = None if args.no_fsm else FsmStateMonitor()
    vel_cmd = VelCommand()
    neck_target = NeckTarget()
    base_ctrl = BaseController(vel_cmd, fsm, dry_run=dry_run)
    neck_ctrl = NeckController(neck_target, fsm)

    rospy.logwarn(
        "[mode] %s  target=%.2fm  (GPU: 单阶段单模型)",
        "DRY-RUN" if dry_run else "MOTION", target_dist_m,
    )

    def on_shutdown():
        rospy.logwarn("[shutdown] 停车 + 脖子回中")
        vel_cmd.stop()
        base_ctrl.set_enabled(False)
        base_ctrl.stop()
        neck_ctrl.set_enabled(False)
        neck_ctrl.stop()
        try:
            base_ctrl.publish_stop_blocking(0.5)
            neck_ctrl.publish_center_blocking(0.5)
        except Exception:
            pass

    rospy.on_shutdown(on_shutdown)
    base_ctrl.set_enabled(True)
    base_ctrl.start()
    neck_ctrl.set_enabled(True)   # APPROACH 阶段即用脖子对左右
    neck_ctrl.start()

    if fsm is not None:
        rospy.loginfo("[FSM] 等待 EXEC_DEFAULT(5)...")
        fsm.wait_for_exec_default(FSM_WAIT_TIMEOUT)

    phase = Phase.APPROACH
    pose_pipe = PosePipeline()
    face_detect_pipe = None
    face_pipe_holder = [None]
    face_fallback_logged = False

    last_person_t = time.time()
    last_face_t = time.time()
    last_loop_t = time.time()
    dist_filt = None
    lin_filt = 0.0
    lin_cmd_out = 0.0
    lin_cmd_target = 0.0
    approach_stable = 0
    neck_state = {"yaw_rad": 0.0, "pitch_rad": 0.0}
    homing_logged = False
    fps_t0 = time.time()
    fps_frames = 0
    fps_show = 0.0
    last_log_t = 0.0

    # 可视化缓存
    person_center = person_bbox = None
    dist_m = dist_src = None
    dx_n = 0.0
    face_bbox_disp = face_cx = face_cy = None
    used_roi = False
    has_face = False

    while not rospy.is_shutdown() and cap.isOpened():
        loop_now = time.time()
        dt_frame = max(1e-3, min(0.2, loop_now - last_loop_t))
        last_loop_t = loop_now

        ret, frame = cap.read()
        if not ret:
            rospy.logwarn_throttle(2.0, "[cam] 抓帧失败")
            continue
        if ZED_STEREO:
            frame = frame[:, : frame.shape[1] // 2]

        fps_frames += 1
        if fps_frames >= 10:
            now = time.time()
            fps_show = fps_frames / (now - fps_t0)
            fps_t0 = now
            fps_frames = 0

        if (proc_w, proc_h) != (real_w, real_h):
            proc_bgr = cv2.resize(frame, (proc_w, proc_h))
        else:
            proc_bgr = frame
        rgb = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False

        h, w = frame.shape[:2]
        cx_img = w / 2.0
        person_center = person_bbox = None
        dist_m = dist_src = None
        face_bbox_disp = face_cx = face_cy = None
        used_roi = False
        has_face = False
        dx_n = 0.0
        lin_cmd_out = 0.0
        lin_cmd_target = 0.0
        track_source = "none"

        if phase == Phase.APPROACH:
            track_center = None
            track_dist = None
            track_dist_src = "?"

            res = pose_pipe.process(rgb) if pose_pipe else None
            if res is not None and res.pose_landmarks is not None:
                lm = res.pose_landmarks
                person_center, person_bbox = body_center_and_bbox(lm, w, h)
                track_dist, track_dist_src = estimate_distance(
                    lm, w, h, focal_px_display,
                )
                if person_center is not None and track_dist is not None:
                    track_center = person_center
                    track_source = "pose"
                    face_fallback_logged = False

            if track_center is None:
                # Pose 丢失 -> 人脸估距 + 继续跟随
                if face_detect_pipe is None:
                    rospy.loginfo("[fallback] Pose 丢失,懒加载 Face Detection...")
                    face_detect_pipe = FaceDetectPipeline()
                    face_fallback_logged = True
                elif not face_fallback_logged:
                    rospy.loginfo_throttle(3.0, "[fallback] 使用人脸估距继续跟随")
                    face_fallback_logged = True
                hit = face_detect_pipe.detect_best(
                    rgb, proc_w, proc_h, w, h, focal_px_display,
                )
                if hit is not None:
                    face_cx, face_cy, face_bbox_disp, track_dist = hit
                    track_center = (face_cx, face_cy)
                    track_dist_src = "face"
                    track_source = "face"
                    has_face = True

            if track_center is not None and track_dist is not None:
                if dist_filt is None:
                    dist_filt = track_dist
                else:
                    dist_filt = (
                        (1 - DIST_EMA_ALPHA) * dist_filt
                        + DIST_EMA_ALPHA * track_dist
                    )

                dx_n = clamp(
                    (track_center[0] - cx_img) / (w / 2.0), -1.0, 1.0,
                )
                e_d = dist_filt - target_dist_m
                e_d_eff = 0.0 if abs(e_d) < DIST_DEAD_M else e_d

                lin_cmd_target = clamp(K_LIN * e_d_eff, -MAX_LIN_CMD, MAX_LIN_CMD)
                lin_filt = (
                    (1 - LIN_EMA_ALPHA) * lin_filt
                    + LIN_EMA_ALPHA * lin_cmd_target
                )
                lin_cmd_out = clamp(lin_filt, -MAX_LIN_CMD, MAX_LIN_CMD)
                lin_filt = lin_cmd_out
                vel_cmd.set(lin_cmd_out, 0.0, valid_for_sec=2.0)

                cur_yaw, cur_pitch = neck_target.get()
                new_yaw, new_pitch = update_neck_from_error(
                    cur_yaw, cur_pitch, dx_n, 0.0, neck_state,
                )
                neck_target.set(new_yaw, new_pitch)
                last_person_t = loop_now

                dist_ok = abs(e_d) < DIST_DEAD_M
                ang_ok = abs(dx_n) < ANG_DEAD
                if dist_ok and ang_ok:
                    approach_stable += 1
                else:
                    approach_stable = 0

                if approach_stable >= APPROACH_STABLE_FRAMES:
                    phase, pose_pipe, face_detect_pipe = switch_to_face_track(
                        pose_pipe, face_detect_pipe, face_pipe_holder,
                        vel_cmd, base_ctrl, neck_ctrl, dist_filt,
                    )
                    approach_stable = 0
                    lin_filt = 0.0
                    face_fallback_logged = False
            else:
                approach_stable = 0
                lin_filt = 0.0
                lin_cmd_out = 0.0
                lin_cmd_target = 0.0
                vel_cmd.stop()

        elif phase == Phase.FACE_TRACK:
            lin_cmd_out = 0.0
            lin_cmd_target = 0.0
            vel_cmd.stop()
            face_pipe = face_pipe_holder[0]
            if face_pipe is not None:
                res, used_roi = face_pipe.process(rgb, proc_w, proc_h)
                if res is not None and res.multi_face_landmarks:
                    has_face = True
                    lm = res.multi_face_landmarks[0]
                    fx1, fy1, fx2, fy2 = face_bbox_from_landmarks(lm, w, h, pad=12)
                    face_bbox_disp = (fx1, fy1, fx2, fy2)
                    face_cx = (fx1 + fx2) / 2.0
                    face_cy = (fy1 + fy2) / 2.0
                    dx_n = (face_cx - cx_img) / (w / 2.0)
                    dy_n = (face_cy - h / 2.0) / (h / 2.0)
                    cur_yaw, cur_pitch = neck_target.get()
                    new_yaw, new_pitch = update_neck_from_error(
                        cur_yaw, cur_pitch, dx_n, dy_n, neck_state,
                    )
                    neck_target.set(new_yaw, new_pitch)
                    last_face_t = loop_now
                    homing_logged = False

            if not has_face:
                lost_dur = loop_now - last_face_t
                if (NO_FACE_RETURN_HOME_SEC > 0
                        and lost_dur > NO_FACE_RETURN_HOME_SEC):
                    cur_yaw, cur_pitch = neck_target.get()
                    step_rad = math.radians(RETURN_HOME_RATE_DEG_PER_SEC * dt_frame)
                    new_yaw = (
                        cur_yaw - math.copysign(min(step_rad, abs(cur_yaw)), cur_yaw)
                    ) if abs(cur_yaw) > 1e-4 else 0.0
                    new_pitch = (
                        cur_pitch - math.copysign(
                            min(step_rad, abs(cur_pitch)), cur_pitch,
                        )
                    ) if abs(cur_pitch) > 1e-4 else 0.0
                    neck_target.set(new_yaw, new_pitch)
                    neck_state["yaw_rad"] = new_yaw
                    neck_state["pitch_rad"] = new_pitch
                    if not homing_logged:
                        rospy.loginfo("[homing] 丢失人脸 %.1fs,脖子回中", lost_dur)
                        homing_logged = True
                # 人脸丢失: 底盘保持静止,不回到 APPROACH 重新移动
                vel_cmd.stop()

        tgt_lin, _, vel_stale = vel_cmd.get()
        tgt_yaw, tgt_pitch = neck_target.get()

        if not args.no_gui:
            draw_scale = max(1.0, h / 720.0)
            thick1 = max(1, int(2 * draw_scale))
            thick2 = max(2, int(3 * draw_scale))
            cx_i, cy_i = int(cx_img), int(h / 2)

            cv2.drawMarker(
                frame, (cx_i, cy_i), (255, 255, 255),
                cv2.MARKER_CROSS, max(20, int(30 * draw_scale)), thickness=thick1,
            )

            if phase == Phase.APPROACH and person_bbox is not None:
                x1, y1, x2, y2 = person_bbox
                if dist_filt is None:
                    col = (180, 180, 180)
                elif abs(dist_filt - target_dist_m) < DIST_DEAD_M:
                    col = (0, 220, 0)
                elif dist_filt > target_dist_m:
                    col = (0, 200, 255)
                else:
                    col = (255, 100, 100)
                cv2.rectangle(frame, (x1, y1), (x2, y2), col, thick1)
                if person_center is not None:
                    cv2.circle(
                        frame, (int(person_center[0]), int(person_center[1])),
                        max(5, int(8 * draw_scale)), col, -1,
                    )

            if phase == Phase.APPROACH and track_source == "face" and face_bbox_disp is not None:
                fx1, fy1, fx2, fy2 = face_bbox_disp
                col = (0, 255, 255)
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), col, thick1)
                cv2.circle(
                    frame, (int(face_cx), int(face_cy)),
                    max(4, int(6 * draw_scale)), col, -1,
                )
                cv2.putText(
                    frame, "FACE dist", (fx1, max(0, fy1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * draw_scale, col, thick1,
                )

            if phase == Phase.FACE_TRACK and face_bbox_disp is not None:
                fx1, fy1, fx2, fy2 = face_bbox_disp
                col = (0, 255, 255) if used_roi else (0, 255, 0)
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), col, thick1)
                cv2.circle(frame, (int(face_cx), int(face_cy)),
                           max(4, int(6 * draw_scale)), col, -1)

            base_active = (phase == Phase.APPROACH and base_ctrl.enabled
                           and not dry_run and not vel_stale)
            fsm_text = "off"
            if fsm is not None:
                s = fsm.state
                fsm_text = (
                    f"{FsmStateMonitor.state_name(s)}({s})"
                    if s is not None else "wait"
                )
            lines = [
                f"PHASE {phase.value}",
                f"FPS {fps_show:5.1f}",
            ]
            if phase == Phase.APPROACH:
                lines += [
                    f"track = {track_source}",
                    ("dist = ??.??m" if dist_filt is None
                     else f"dist = {dist_filt:5.2f} m  ({track_dist_src if track_source != 'none' else '?'})"),
                    f"target= {target_dist_m:5.2f} m  stable={approach_stable}/{APPROACH_STABLE_FRAMES}",
                    f"dx = {dx_n:+0.2f}  neck_yaw = {math.degrees(tgt_yaw):+5.1f}",
                ]
            else:
                lines += [
                    f"neck yaw={math.degrees(tgt_yaw):+6.1f} pitch={math.degrees(tgt_pitch):+6.1f}",
                    "face OK" if has_face else f"no face ({loop_now - last_face_t:.1f}s)",
                ]
            lines += [
                f"cmd linear.x = {lin_cmd_out:+.3f}  [{-MAX_LIN_CMD:.0f}, {MAX_LIN_CMD:.0f}]",
                f"cmd target   = {lin_cmd_target:+.3f}",
                "cmd angular.z= +0.000 (neck)",
                f"publish = {'ON' if base_active else 'OFF'}",
            ]
            lines.append(f"FSM {fsm_text}")
            y_text = int(40 * draw_scale)
            for ln in lines:
                cv2.putText(
                    frame, ln, (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * draw_scale,
                    (0, 255, 0), thick1,
                )
                y_text += int(34 * draw_scale)

            # 速度条: -1 ~ +1
            bar_w = int(min(w * 0.35, 420 * draw_scale))
            bar_h = max(12, int(18 * draw_scale))
            bar_x = w - bar_w - int(20 * draw_scale)
            bar_y = h - int(80 * draw_scale)
            cv2.rectangle(
                frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                (60, 60, 60), -1,
            )
            mid_x = bar_x + bar_w // 2
            cv2.line(
                frame, (mid_x, bar_y), (mid_x, bar_y + bar_h),
                (180, 180, 180), 1,
            )
            fill_x = mid_x + int(lin_cmd_out * (bar_w // 2 - 4))
            fill_x = clamp(fill_x, bar_x + 2, bar_x + bar_w - 2)
            bar_col = (0, 220, 255) if lin_cmd_out > 0.01 else (
                (255, 120, 120) if lin_cmd_out < -0.01 else (160, 160, 160)
            )
            cv2.rectangle(
                frame,
                (min(mid_x, fill_x), bar_y + 3),
                (max(mid_x, fill_x), bar_y + bar_h - 3),
                bar_col, -1,
            )
            cv2.putText(
                frame,
                f"linear.x {lin_cmd_out:+.2f}",
                (bar_x, bar_y - int(8 * draw_scale)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8 * draw_scale,
                bar_col, thick1,
            )

            mode_txt = "DRY-RUN" if dry_run else "MOTION"
            cv2.putText(
                frame, mode_txt, (20, h - int(20 * draw_scale)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0 * draw_scale,
                (255, 220, 0) if dry_run else (0, 100, 255), thick2,
            )

            show = fit_letterbox(frame, screen_w, screen_h)
            cv2.imshow(WINDOW_NAME, show)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("f"):
                is_fullscreen = not is_fullscreen
                cv2.setWindowProperty(
                    WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if is_fullscreen else cv2.WINDOW_NORMAL,
                )

        if loop_now - last_log_t > 1.0:
            if phase == Phase.APPROACH:
                rospy.loginfo(
                    "[approach] track=%s dist=%s stable=%d/%d "
                    "cmd_x=%+.3f tgt=%+.3f yaw=%+.1f fps=%.1f",
                    track_source,
                    f"{dist_filt:.2f}m" if dist_filt else "??",
                    approach_stable, APPROACH_STABLE_FRAMES,
                    lin_cmd_out, lin_cmd_target,
                    math.degrees(tgt_yaw), fps_show,
                )
            else:
                rospy.loginfo(
                    "[face] face=%s yaw=%+.1f pitch=%+.1f fps=%.1f",
                    "Y" if has_face else "N",
                    math.degrees(tgt_yaw), math.degrees(tgt_pitch), fps_show,
                )
            last_log_t = loop_now

    rospy.loginfo("[exit] 清理资源")
    if pose_pipe is not None:
        pose_pipe.close()
    if face_detect_pipe is not None:
        face_detect_pipe.close()
    if face_pipe_holder[0] is not None:
        face_pipe_holder[0].close()
    cap.release()
    if not args.no_gui:
        cv2.destroyAllWindows()
    vel_cmd.stop()
    base_ctrl.stop()
    neck_ctrl.stop()
    try:
        base_ctrl.publish_stop_blocking(0.5)
        neck_ctrl.publish_center_blocking(0.5)
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
