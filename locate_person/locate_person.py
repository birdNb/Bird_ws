#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================
# locate_person.py  ——  人物轮廓识别 + 底盘视觉伺服
#
# 任务: 在画面中检测人体,估算 (距离, 横向方向),用底盘速度命令保持
#       与人正面距离约 TARGET_DISTANCE_M 米,且人位于画面正中央。
#
# 输入:  ZED Mini 左眼 (沿用 gaze_robot.py 的相机框架)
# 处理:  MediaPipe Pose lite (Orin Nano 上可达 ~25fps)
# 输出:  geometry_msgs/Twist  ->  /cmd_vel
#        sim2real master 在 EXEC_DEFAULT(5) 下消费 cmd_vel 驱动步态行走
#        (来源: sim2real_master/joy_footstep.yaml: axis1->linear.x, axis3->angular.z)
#
# 距离估计原理:
#   已知真实肩宽 ~0.40m,画面里肩宽像素 = |左肩x - 右肩x| * 显示图宽
#   D = focal_px * 0.40 / shoulder_pix
#   ZED Mini 单眼 720p 水平视场 ~90°,焦距 fx ≈ 640 px(可命令行覆盖)
#   同时用躯干长度做副估计,取中位/加权平均提升鲁棒性
#
# 控制律:
#   linear.x  =  K_LIN * (dist - TARGET_DISTANCE_M)       (>0 前进)
#   angular.z =  -K_ANG * dx_normalized                   (人在右则右转)
#   全部经死区/限幅/EMA 平滑
#
# 安全:
#   - 默认 dry-run:仅计算并显示,不真实发布到 /cmd_vel
#     真正驱动机器人必须显式带 --enable-motion
#   - FSM != EXEC_DEFAULT(5) 时,所有速度命令置零
#   - 人体丢失 > NO_PERSON_STOP_SEC 立即停车
#   - Ctrl+C / ESC 退出: 连发 0 命令 0.5s
#
# 运行:
#   python3 locate_person.py             # dry-run 调试
#   python3 locate_person.py --enable-motion   # 实际发 cmd_vel
# ==============================================================

import argparse
import math
import os
import subprocess
import threading
import time

# ===== SSH 无 DISPLAY 兜底: 必须在 import cv2 之前 =====
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
from std_msgs.msg import Int32

# =====================================================
#                    可调参数
# =====================================================

# ----- ROS 接口 -----
CMD_VEL_TOPIC = "/cmd_vel"

# ----- 相机配置 -----
WIDTH, HEIGHT = 2560, 720    # ZED Mini stereo, 单眼 1280x720
TARGET_FPS = 30
CAM_ID = 0
USE_MJPG = False
ZED_STEREO = True
PROC_MAX_W = 960             # 喂给 MediaPipe 的最大宽度(降采样保流畅)

# ----- 显示 -----
FULLSCREEN = True
WINDOW_NAME = "Locate Person (Orin Nano)"

# ----- MediaPipe Pose -----
POSE_MODEL_COMPLEXITY = 0    # 0=lite, 1=full, 2=heavy
POSE_DET_CONFIDENCE = 0.5
POSE_TRK_CONFIDENCE = 0.5
# 单点 visibility 低于此阈值视为不可信
KP_VIS_TH = 0.45

# ----- 相机内参(用于距离估计) -----
# ZED Mini 单眼 720p 水平 FOV ≈ 90°  =>  fx = (w/2)/tan(45°) ≈ 640 px
# 实际焦距按 (real_w / 1280) * FOCAL_PX_BASE 线性等比例换算
FOCAL_PX_BASE = 640.0
FOCAL_PX_BASE_AT_W = 1280

# ----- 人体真实尺寸(米) -----
SHOULDER_REAL_M = 0.40   # 成年人平均肩宽
TORSO_REAL_M = 0.50      # 肩中点 -> 髋中点 平均长度

# ----- 控制目标 -----
TARGET_DISTANCE_M = 2.0          # 期望距离
DIST_DEAD_M = 0.20               # 距离误差死区(±20cm 不动)
ANG_DEAD = 0.05                  # 横向归一化偏差死区(±5% 画面宽)

# ----- 控制增益 -----
K_LIN = 0.40       # 距离误差 -> 线速度 (m/s per m)
K_ANG = 1.20       # 横向归一化偏差 -> 角速度 (rad/s per unit)

# ----- 速度限幅 -----
MAX_LIN = 0.30     # m/s
MAX_ANG = 0.60     # rad/s

# ----- 平滑(EMA) -----
LIN_EMA_ALPHA = 0.4   # 越小越粘滞
ANG_EMA_ALPHA = 0.5
DIST_EMA_ALPHA = 0.35  # 距离估计本身的滤波

# ----- 控制线程频率 -----
PUBLISH_RATE_HZ = 20

# ----- 失踪超时(秒): 超过则立即停车 -----
NO_PERSON_STOP_SEC = 0.8

# ----- FSM 守门 -----
FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5
FSM_GATE_ENABLED = True
FSM_WAIT_TIMEOUT = 30.0

# =====================================================
#                    工具函数
# =====================================================


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
            ["xrandr"], env=env, stderr=subprocess.DEVNULL, timeout=2
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


# ===================== FSM 状态监听 =====================
class FsmStateMonitor:
    """订阅 /fsm_state 维护最新状态(线程安全)。"""

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


# ===================== 速度共享对象 =====================
class VelCommand:
    """线程安全的 (linear.x, angular.z) 目标。视觉线程写,控制线程读。"""

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
            # 数据陈旧时认为停车
            if self._stale_after > 0 \
                    and time.time() - self._t > self._stale_after:
                return 0.0, 0.0, True
            return self._lin, self._ang, False

    def stop(self):
        self.set(0.0, 0.0, valid_for_sec=0.0)


# ===================== 控制发布线程 =====================
class BaseController(threading.Thread):
    """以 PUBLISH_RATE_HZ 持续把 VelCommand 内的目标速度发布到 cmd_vel。

    - FSM != EXEC_DEFAULT(5)         -> 强制 0
    - dry_run = True                  -> 不真实 publish,仅日志
    """

    def __init__(self, vel: VelCommand,
                 fsm: 'FsmStateMonitor | None',
                 dry_run: bool):
        super().__init__(daemon=True)
        self._vel = vel
        self._fsm = fsm
        self._dry_run = dry_run
        self._pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self._rate = rospy.Rate(PUBLISH_RATE_HZ)
        self._stop_evt = threading.Event()

    @property
    def num_subscribers(self) -> int:
        return self._pub.get_num_connections()

    def stop(self):
        self._stop_evt.set()

    def publish_stop_blocking(self, duration: float = 0.5):
        """阻塞地连发停车命令(用于退出)。"""
        msg = Twist()
        end_t = time.time() + duration
        while time.time() < end_t:
            try:
                if not self._dry_run:
                    self._pub.publish(msg)
            except Exception:
                break
            time.sleep(1.0 / max(PUBLISH_RATE_HZ, 1))

    def run(self):
        msg = Twist()
        warned_no_fsm = False
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            lin, ang, stale = self._vel.get()

            # 安全闸: FSM 没进 EXEC_DEFAULT 一律 0
            fsm_ok = (self._fsm is None) or self._fsm.is_exec_default()
            if not fsm_ok:
                lin, ang = 0.0, 0.0
                if not warned_no_fsm:
                    rospy.logwarn_throttle(
                        2.0,
                        "[ctrl] FSM=%s,暂停下发(等待 EXEC_DEFAULT)",
                        FsmStateMonitor.state_name(
                            self._fsm.state if self._fsm else -1
                        ),
                    )
                    warned_no_fsm = True
            else:
                warned_no_fsm = False

            msg.linear.x = lin
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = ang
            try:
                if not self._dry_run:
                    self._pub.publish(msg)
            except Exception as e:
                rospy.logerr_throttle(2.0, "[ctrl] publish 异常: %s", e)
            self._rate.sleep()


# =====================================================
#                    视觉 / 距离 / 控制律
# =====================================================

# MediaPipe Pose 关键索引(BlazePose)
KP_NOSE = 0
KP_L_SHOULDER = 11
KP_R_SHOULDER = 12
KP_L_HIP = 23
KP_R_HIP = 24

# 用于画整体身体框
BODY_KP_IDX = [
    0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28,
]


def _kp_visible(lm, idx) -> bool:
    """关键点 visibility 是否足够可信。"""
    if idx >= len(lm.landmark):
        return False
    return lm.landmark[idx].visibility >= KP_VIS_TH


def _kp_pix(lm, idx, w, h):
    p = lm.landmark[idx]
    return p.x * w, p.y * h


def estimate_distance(lm, w, h, focal_px):
    """利用 (肩宽) 与 (躯干长度) 两个估计,合并出当前距离 (m)。

    返回 (dist_m, source_tag) 或 (None, "?")
        source_tag: "shoulder" / "torso" / "blend"
    """
    have_sh = _kp_visible(lm, KP_L_SHOULDER) \
        and _kp_visible(lm, KP_R_SHOULDER)
    have_hip = _kp_visible(lm, KP_L_HIP) and _kp_visible(lm, KP_R_HIP)

    d_sh = None
    d_torso = None

    if have_sh:
        lsx, lsy = _kp_pix(lm, KP_L_SHOULDER, w, h)
        rsx, rsy = _kp_pix(lm, KP_R_SHOULDER, w, h)
        # 用横向像素距离更稳(忽略肩膀倾斜)
        sh_pix = abs(lsx - rsx)
        if sh_pix >= 8:
            d_sh = focal_px * SHOULDER_REAL_M / sh_pix

    if have_sh and have_hip:
        lsx, lsy = _kp_pix(lm, KP_L_SHOULDER, w, h)
        rsx, rsy = _kp_pix(lm, KP_R_SHOULDER, w, h)
        lhx, lhy = _kp_pix(lm, KP_L_HIP, w, h)
        rhx, rhy = _kp_pix(lm, KP_R_HIP, w, h)
        sh_cy = (lsy + rsy) / 2.0
        hip_cy = (lhy + rhy) / 2.0
        torso_pix = abs(hip_cy - sh_cy)
        if torso_pix >= 12:
            d_torso = focal_px * TORSO_REAL_M / torso_pix

    if d_sh is None and d_torso is None:
        return None, "?"
    if d_sh is None:
        return d_torso, "torso"
    if d_torso is None:
        return d_sh, "shoulder"

    # 两个都有: 取中点(肩宽更准但侧身会偏大,躯干高更稳)
    return 0.5 * (d_sh + d_torso), "blend"


def body_center_and_bbox(lm, w, h):
    """计算"人物中心点(肩中点)"与可视化用的人体外接框。

    若无可靠肩 keypoint,降级用鼻子;再不行返回 None。
    """
    have_sh = _kp_visible(lm, KP_L_SHOULDER) \
        and _kp_visible(lm, KP_R_SHOULDER)
    if have_sh:
        lsx, lsy = _kp_pix(lm, KP_L_SHOULDER, w, h)
        rsx, rsy = _kp_pix(lm, KP_R_SHOULDER, w, h)
        cx = (lsx + rsx) / 2.0
        cy = (lsy + rsy) / 2.0
    elif _kp_visible(lm, KP_NOSE):
        nx, ny = _kp_pix(lm, KP_NOSE, w, h)
        cx, cy = nx, ny
    else:
        return None, None

    # bbox: 所有 visibility 够高的关键点的外接框
    xs, ys = [], []
    for i in BODY_KP_IDX:
        if i < len(lm.landmark) and _kp_visible(lm, i):
            p = lm.landmark[i]
            xs.append(p.x * w)
            ys.append(p.y * h)
    if len(xs) < 2:
        bbox = None
    else:
        pad = 18
        bbox = (
            max(0, int(min(xs)) - pad),
            max(0, int(min(ys)) - pad),
            min(w - 1, int(max(xs)) + pad),
            min(h - 1, int(max(ys)) + pad),
        )
    return (cx, cy), bbox


# ===================== 主流程 =====================


def main():
    parser = argparse.ArgumentParser(
        description="locate_person: 人体检测 + 底盘视觉伺服 (距离+方向)",
    )
    parser.add_argument(
        "--enable-motion", action="store_true",
        help="真实发布 /cmd_vel 驱动机器人(默认 dry-run 只显示)",
    )
    parser.add_argument(
        "--no-gui", action="store_true",
        help="不开图形窗口(纯日志)",
    )
    parser.add_argument(
        "--no-fsm", action="store_true",
        help="跳过 FSM 守门(谨慎)",
    )
    parser.add_argument(
        "--focal-px", type=float, default=None,
        help=f"覆盖单眼 1280-宽下的水平焦距(默认 {FOCAL_PX_BASE:.0f}px)",
    )
    parser.add_argument(
        "--target-dist", type=float, default=TARGET_DISTANCE_M,
        help=f"期望距离米(默认 {TARGET_DISTANCE_M:.1f})",
    )
    args = parser.parse_args()

    dry_run = not args.enable_motion
    target_dist_m = args.target_dist
    focal_base = args.focal_px if args.focal_px is not None else FOCAL_PX_BASE

    rospy.init_node("locate_person", anonymous=False)

    # ----- 相机 -----
    cap = open_camera()
    if not cap.isOpened():
        rospy.logerr("[cam] 无法打开相机 /dev/video%d", CAM_ID)
        return
    raw_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    raw_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    real_w = raw_w // 2 if ZED_STEREO else raw_w
    real_h = raw_h
    proc_w, proc_h = compute_proc_size(real_w, real_h, PROC_MAX_W)
    # 按显示图实际宽度等比例换算焦距(像素值与图像宽度成正比)
    focal_px_display = focal_base * (real_w / FOCAL_PX_BASE_AT_W)
    rospy.loginfo(
        "[cam] 原始 %dx%d@%.1ffps  左眼 %dx%d  proc %dx%d  fx=%.1fpx",
        raw_w, raw_h, real_fps, real_w, real_h, proc_w, proc_h,
        focal_px_display,
    )

    # ----- MediaPipe Pose -----
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=POSE_MODEL_COMPLEXITY,
        smooth_landmarks=True,
        enable_segmentation=False,
        min_detection_confidence=POSE_DET_CONFIDENCE,
        min_tracking_confidence=POSE_TRK_CONFIDENCE,
    )

    # ----- 显示 -----
    screen_w = screen_h = 0
    is_fullscreen = FULLSCREEN
    if not args.no_gui:
        screen_w, screen_h = detect_screen_size()
        rospy.loginfo(
            "[gui] 屏幕 %dx%d 全屏=%s (ESC/q 退出, f 切全屏)",
            screen_w, screen_h, FULLSCREEN,
        )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        if FULLSCREEN:
            cv2.setWindowProperty(
                WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN,
            )

    # ----- FSM / 控制线程 -----
    fsm = None if args.no_fsm else FsmStateMonitor()
    vel_cmd = VelCommand()
    ctrl = BaseController(vel_cmd, fsm, dry_run=dry_run)

    rospy.logwarn(
        "[mode] %s  target_dist=%.2fm  enable_motion=%s",
        "DRY-RUN(不真实下发 cmd_vel)" if dry_run else "MOTION(实际驱动机器人!)",
        target_dist_m, args.enable_motion,
    )
    if not dry_run:
        rospy.logwarn(
            "[mode] 注意: 已启用真实运动,请确保机器人前方 3m 内无障碍/人员",
        )

    def on_shutdown():
        rospy.logwarn("[shutdown] 退出 -> 停车")
        vel_cmd.stop()
        ctrl.stop()
        try:
            ctrl.publish_stop_blocking(0.5)
        except Exception:
            pass

    rospy.on_shutdown(on_shutdown)
    ctrl.start()

    # 等订阅者
    t0 = time.time()
    while ctrl.num_subscribers == 0 and time.time() - t0 < 3.0 \
            and not rospy.is_shutdown():
        time.sleep(0.1)
    if ctrl.num_subscribers == 0:
        rospy.logwarn(
            "[ctrl] %s 上还没订阅者(master 可能未启动),程序继续",
            CMD_VEL_TOPIC,
        )
    else:
        rospy.loginfo(
            "[ctrl] %d 订阅者已连接到 %s",
            ctrl.num_subscribers, CMD_VEL_TOPIC,
        )

    if fsm is not None:
        rospy.loginfo("[FSM] 等待 EXEC_DEFAULT(5)...")
        fsm.wait_for_exec_default(FSM_WAIT_TIMEOUT)
        rospy.loginfo("[FSM] OK, 开始视觉伺服")

    # ----- 主循环 -----
    last_person_t = time.time()
    last_loop_t = time.time()
    dist_filt = None        # 距离 EMA
    lin_filt = 0.0          # 线速度 EMA
    ang_filt = 0.0          # 角速度 EMA
    fps_t0 = time.time()
    fps_frames = 0
    fps_show = 0.0
    last_log_t = 0.0

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

        # FPS
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
        res = pose.process(rgb)

        h, w = frame.shape[:2]
        cx_img = w / 2.0
        # 显示用 / 距离计算用 都基于 real_w/real_h
        person_center = None
        person_bbox = None
        dist_m = None
        dist_src = "?"
        dx_n = 0.0

        if res.pose_landmarks is not None:
            lm = res.pose_landmarks
            person_center, person_bbox = body_center_and_bbox(lm, w, h)
            dist_m, dist_src = estimate_distance(
                lm, w, h, focal_px_display,
            )

        if person_center is not None and dist_m is not None:
            # EMA 滤波距离(单目噪声大)
            if dist_filt is None:
                dist_filt = dist_m
            else:
                a = DIST_EMA_ALPHA
                dist_filt = (1 - a) * dist_filt + a * dist_m

            dx_n = (person_center[0] - cx_img) / (w / 2.0)
            dx_n = clamp(dx_n, -1.0, 1.0)

            # ----- 控制律 -----
            # 死区
            e_d = dist_filt - target_dist_m
            e_d_eff = 0.0 if abs(e_d) < DIST_DEAD_M else e_d
            e_a_eff = 0.0 if abs(dx_n) < ANG_DEAD else dx_n

            lin_target = clamp(K_LIN * e_d_eff, -MAX_LIN, MAX_LIN)
            ang_target = clamp(-K_ANG * e_a_eff, -MAX_ANG, MAX_ANG)

            # EMA 输出
            lin_filt = (1 - LIN_EMA_ALPHA) * lin_filt \
                + LIN_EMA_ALPHA * lin_target
            ang_filt = (1 - ANG_EMA_ALPHA) * ang_filt \
                + ANG_EMA_ALPHA * ang_target

            vel_cmd.set(lin_filt, ang_filt, valid_for_sec=0.5)
            last_person_t = loop_now
        else:
            # 没人或距离不可估: 超时后立即停车
            lost = loop_now - last_person_t
            if lost > NO_PERSON_STOP_SEC:
                # 线/角速度都软着陆到 0
                lin_filt = (1 - LIN_EMA_ALPHA) * lin_filt
                ang_filt = (1 - ANG_EMA_ALPHA) * ang_filt
                if abs(lin_filt) < 0.02:
                    lin_filt = 0.0
                if abs(ang_filt) < 0.05:
                    ang_filt = 0.0
                vel_cmd.set(lin_filt, ang_filt, valid_for_sec=0.5)

        # ----- 可视化 -----
        if not args.no_gui:
            draw_scale = max(1.0, h / 720.0)
            thick1 = max(1, int(2 * draw_scale))
            thick2 = max(2, int(3 * draw_scale))

            # 画面中心十字
            cx_i, cy_i = int(cx_img), int(h / 2)
            cv2.drawMarker(
                frame, (cx_i, cy_i), (255, 255, 255),
                cv2.MARKER_CROSS, max(20, int(30 * draw_scale)),
                thickness=thick1,
            )
            # 死区方框(横向)
            dx_pix = int(ANG_DEAD * w / 2.0)
            cv2.line(
                frame, (cx_i - dx_pix, 0), (cx_i - dx_pix, h),
                (90, 90, 90), 1,
            )
            cv2.line(
                frame, (cx_i + dx_pix, 0), (cx_i + dx_pix, h),
                (90, 90, 90), 1,
            )

            if person_bbox is not None:
                x1, y1, x2, y2 = person_bbox
                # 颜色: 距离在死区内 = 绿,超出 = 黄/红
                if dist_m is None:
                    col = (180, 180, 180)
                elif abs(dist_filt - target_dist_m) < DIST_DEAD_M:
                    col = (0, 220, 0)
                elif dist_filt > target_dist_m:
                    col = (0, 200, 255)   # 太远,要前进 -> 橙色
                else:
                    col = (255, 100, 100)  # 太近,要后退 -> 蓝色
                cv2.rectangle(frame, (x1, y1), (x2, y2), col, thick1)
                if person_center is not None:
                    cx_p, cy_p = person_center
                    cv2.circle(
                        frame, (int(cx_p), int(cy_p)),
                        max(5, int(8 * draw_scale)), col, -1,
                    )
                    # 中心 -> 人物中心的偏差线
                    cv2.line(
                        frame, (cx_i, int(cy_p)),
                        (int(cx_p), int(cy_p)), col, thick1,
                    )

            # 状态文本
            tgt_lin, tgt_ang, _ = vel_cmd.get()
            fsm_text = "off"
            if fsm is not None:
                s = fsm.state
                fsm_text = (
                    f"{FsmStateMonitor.state_name(s)}({s})"
                    if s is not None else "wait"
                )
            lines = [
                f"FPS {fps_show:5.1f}",
                ("dist = ??.??m" if dist_filt is None
                 else f"dist = {dist_filt:5.2f} m  ({dist_src})"),
                f"target= {target_dist_m:5.2f} m",
                f"dx = {dx_n:+0.2f}",
                f"vx = {tgt_lin:+0.2f} m/s",
                f"wz = {tgt_ang:+0.2f} rad/s",
                f"FSM {fsm_text}",
            ]
            y_text = int(40 * draw_scale)
            for ln in lines:
                cv2.putText(
                    frame, ln, (20, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * draw_scale,
                    (0, 255, 0), thick1,
                )
                y_text += int(34 * draw_scale)

            mode_lines = []
            if dry_run:
                mode_lines.append(("DRY-RUN", (255, 220, 0)))
            else:
                mode_lines.append(("MOTION", (0, 100, 255)))
            if person_center is not None:
                mode_lines.append(("TRACKING", (0, 200, 255)))
            else:
                mode_lines.append((
                    f"NO PERSON ({loop_now - last_person_t:.1f}s)",
                    (0, 0, 255),
                ))
            yy = h - int(20 * draw_scale)
            for txt, col in mode_lines[::-1]:
                cv2.putText(
                    frame, txt, (20, yy),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0 * draw_scale,
                    col, thick2,
                )
                yy -= int(40 * draw_scale)

            show = fit_letterbox(frame, screen_w, screen_h)
            cv2.imshow(WINDOW_NAME, show)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord("f"):
                is_fullscreen = not is_fullscreen
                cv2.setWindowProperty(
                    WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if is_fullscreen
                    else cv2.WINDOW_NORMAL,
                )

        # 终端日志
        if loop_now - last_log_t > 1.0:
            tgt_lin, tgt_ang, _ = vel_cmd.get()
            rospy.loginfo(
                "[track] person=%s  dist=%s  dx=%+0.2f  "
                "vx=%+0.2f m/s  wz=%+0.2f rad/s  fps=%.1f",
                "Y" if person_center is not None else "N",
                f"{dist_filt:.2f}m" if dist_filt is not None else "??",
                dx_n, tgt_lin, tgt_ang, fps_show,
            )
            last_log_t = loop_now

    # 退出清理
    rospy.loginfo("[exit] 主循环结束,清理资源")
    cap.release()
    if not args.no_gui:
        cv2.destroyAllWindows()
    vel_cmd.stop()
    ctrl.stop()
    try:
        ctrl.publish_stop_blocking(0.5)
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
