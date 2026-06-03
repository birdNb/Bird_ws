#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// ----- 手势 / 仲裁 (对齐 zed_gesture_recognition.py) -----
constexpr int NUM_GESTURE_CLASSES = 6;
constexpr int GESTURE_SMOOTH_FRAMES = 3;
constexpr float GESTURE_CONFIDENCE_THRESH = 0.5f;
constexpr int HAND_LOST_GRACE_MS = 450;
constexpr float GESTURE_LOG_INTERVAL_MS = 250;
constexpr int ACTION_COOLDOWN_MS = 2000;
constexpr float ACTION_DURATION_SEC = 5.0f;
constexpr int JOY_ACTION_COOLDOWN_MS = 6000;
constexpr float JOY_ACTION_PULSE_SEC = 0.5f;
constexpr int JOY_ACTION_PUBLISH_HZ = 20;
/** G4 踢球：policy_change 话题（非 /joy_msg 按键） */
constexpr const char* ACTION_CONFIG_TOPIC = "/action_config";
constexpr const char* KICK_POLICY_NAME = "byd_small_kick";
/** 伴生模式：每 N 帧做一次脸 IPC（其余帧外推误差，仍每帧跟颈） */
constexpr int FACE_DETECT_EVERY_N = 2;
constexpr int FACE_EVERY_N = FACE_DETECT_EVERY_N;
constexpr int GESTURE_IPC_MAX_W = 480;
constexpr int FACE_IPC_MAX_W = 480;
constexpr int GESTURE_HOLD_MS = 2000;
constexpr float JOY_ACTIVE_THRESH = 0.15f;
constexpr int JOY_TRIGGER_AXIS_LT = 2;
constexpr int JOY_TRIGGER_AXIS_RT = 5;
constexpr float JOY_TRIGGER_REST = 1.0f;
constexpr float JOY_TRIGGER_ACTIVE_MARGIN = 0.35f;

// ----- 相机 (ZED Mini 左眼 V4L2) -----
constexpr int CAMERA_INDEX = 0;
constexpr int CAMERA_WIDTH = 2560;
constexpr int CAMERA_HEIGHT = 720;
constexpr bool ZED_STEREO = true;
constexpr int PROC_MAX_W = 560;
// ZED 左眼 16:9 显示（960x540，比 640x360 更清晰）
constexpr int DISPLAY_W = 960;
constexpr int DISPLAY_H = 540;
constexpr int MAIN_LOOP_FPS = 15;
constexpr int CAMERA_TARGET_FPS = 15;
constexpr int HAND_LANDMARK_COUNT = 21;

// ----- 人脸跟踪 (对齐 locate_face.py) -----
constexpr int FACE_PROC_MAX_W = 480;
constexpr float FACE_DETECT_SCORE_THRESH = 0.4f;
constexpr float FACE_NMS_THRESH = 0.3f;
constexpr int FACE_DETECT_TOP_K = 5000;
constexpr float FACE_ROI_PAD_RATIO = 0.30f;
constexpr float FACE_TRACK_GRACE_SEC = 1.2f;
constexpr float DEAD_BAND_X = 0.025f;
constexpr float DEAD_BAND_Y = 0.03f;
constexpr float K_YAW_DEG = 26.0f;
constexpr float K_PITCH_DEG = 18.0f;
constexpr float MAX_STEP_YAW_DEG = 8.0f;
constexpr float MAX_STEP_PITCH_DEG = 6.5f;
/** 有新鲜脸检测时更快跟随；跳帧外推时用略低 alpha 防抖 */
constexpr float TARGET_EMA_ALPHA = 0.55f;
constexpr float TARGET_EMA_ALPHA_FRESH = 0.72f;
constexpr float FACE_PREDICT_MAX_SEC = 0.12f;
constexpr float YAW_DX_SIGN = 1.0f;
constexpr float YAW_LIMIT_DEG = 80.0f;
constexpr float PITCH_UP_DEG = -40.0f;
constexpr float PITCH_DOWN_DEG = 60.0f;
constexpr int NECK_PUBLISH_RATE_HZ = 50;
constexpr float NO_FACE_RETURN_HOME_SEC = 1.0f;
constexpr float RETURN_HOME_RATE_DEG_PER_SEC = 45.0f;

// ----- 五指跟手 (distance_hold.py) -----
constexpr float TARGET_DISTANCE_M = 0.50f;
constexpr float DIST_DEADBAND_M = 0.10f;
constexpr float LATERAL_DEADBAND_NORM = 0.20f;
constexpr float LINEAR_X_MAG = 0.5f;
constexpr float ANGULAR_Z_MAG = 1.5f;
constexpr float HAND_TRACK_LOG_HZ = 5.0f;
constexpr int PALM_LOST_RESET_MS = 600;
constexpr int GESTURE_FOLLOW_HOLD_MS = 5000;
constexpr int GESTURE_FOLLOW_LOST_MS = 8000;
constexpr int HAND_TRACKING_JOY_IDLE_MS = 5000;
constexpr int JOY_IDLE_MS = HAND_TRACKING_JOY_IDLE_MS;

constexpr const char* ABSOLUTE_TOPIC = "/pi_plus_absolute";
constexpr const char* CMD_VEL_TOPIC = "/cmd_vel";
constexpr const char* JOY_TOPIC = "/joy";
constexpr const char* JOY_MSG_TOPIC = "/joy_msg";
constexpr const char* HEAD_YAW_JOINT = "head_yaw_joint";
constexpr const char* HEAD_PITCH_JOINT = "head_pitch_joint";
constexpr const char* WAIST_YAW_JOINT = "waist_yaw_joint";

// ----- 手势1 撒娇扭腰 -----
constexpr float COQUETTE_SWAY_AMPLITUDE_DEG = 45.0f;
constexpr int COQUETTE_SWAY_CYCLES = 2;
constexpr float COQUETTE_SWAY_VEL_DEG_PER_SEC = 60.0f;
constexpr float COQUETTE_CHEER_DURATION_SEC = 5.0f;
constexpr float COQUETTE_TRIGGER_PULSE_SEC = 0.5f;
constexpr float COQUETTE_ARM_RESET_SEC = 0.5f;
constexpr int COQUETTE_JOY_PUBLISH_HZ = 20;
constexpr int COQUETTE_WAIST_PUBLISH_HZ = 50;

enum GestureID {
    GESTURE_NONE = -1,
    GESTURE_0 = 0,
    GESTURE_1 = 1,
    GESTURE_2 = 2,
    GESTURE_3 = 3,
    GESTURE_4 = 4,
    GESTURE_5 = 5
};

struct HandDetectResult {
    int gesture_id = GESTURE_NONE;
    int raw_gesture_id = GESTURE_NONE;
    float confidence = 0.0f;
    cv::Rect hand_rect;
    bool has_hand = false;
    bool has_landmarks = false;
    bool in_range = true;
    bool palm_or_back_facing = true;
    float distance_m = 0.0f;
    float dx_norm = 0.0f;
    float dy_norm = 0.0f;
    std::vector<cv::Point> landmarks;
};

/** 等比 letterbox；始终写入新 Mat，避免 src/dst 同一缓冲区导致黑屏 */
inline void resizeLetterbox(const cv::Mat& src, cv::Mat& dst, int target_w, int target_h) {
    if (src.empty() || target_w <= 0 || target_h <= 0) {
        dst.release();
        return;
    }
    if (src.cols == target_w && src.rows == target_h) {
        dst = src;
        return;
    }
    const float scale =
        std::min(target_w / static_cast<float>(src.cols), target_h / static_cast<float>(src.rows));
    const int nw = std::max(1, static_cast<int>(src.cols * scale));
    const int nh = std::max(1, static_cast<int>(src.rows * scale));
    cv::Mat out(target_h, target_w, src.type(), cv::Scalar(0, 0, 0));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_AREA);
    const int x = (target_w - nw) / 2;
    const int y = (target_h - nh) / 2;
    resized.copyTo(out(cv::Rect(x, y, nw, nh)));
    dst = out;
}

/** 手势 0~5 画面叠加颜色 BGR（与 zed_gesture_recognition.py 一致） */
inline cv::Scalar gestureColorBgr(int gesture) {
    static const cv::Scalar colors[6] = {
        cv::Scalar(0, 0, 255),      // 0 红
        cv::Scalar(0, 165, 255),    // 1 橙
        cv::Scalar(0, 255, 255),    // 2 黄
        cv::Scalar(0, 255, 0),      // 3 绿
        cv::Scalar(255, 0, 0),      // 4 蓝
        cv::Scalar(255, 0, 255),    // 5 紫
    };
    if (gesture < 0 || gesture > 5) {
        return cv::Scalar(128, 128, 128);
    }
    return colors[gesture];
}

inline long long getCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

inline float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / static_cast<float>(M_PI); }

inline std::string projectRoot() {
    const char* env = std::getenv("HAND_IDENTIFY_CPP_ROOT");
    if (env && env[0]) return std::string(env);
    return std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/Bird_ws/hand_identify_cpp";
}

inline void computeProcSize(int src_w, int src_h, int max_w, int& proc_w, int& proc_h) {
    if (src_w <= max_w) {
        proc_w = src_w;
        proc_h = src_h;
        return;
    }
    proc_w = max_w;
    proc_h = std::max(1, static_cast<int>(src_h * max_w / src_w));
}
