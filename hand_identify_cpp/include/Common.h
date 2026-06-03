#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// ----- 手势 / 仲裁 -----
constexpr int NUM_GESTURE_CLASSES = 6;
constexpr float GESTURE_CONFIDENCE_THRESH = 0.75f;
constexpr int ACTION_COOLDOWN_MS = 2000;
constexpr int GESTURE_HOLD_MS = 2000;
constexpr int JOY_IDLE_MS = 5000;
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
constexpr int DISPLAY_W = 640;
constexpr int DISPLAY_H = 480;

// ----- 人脸跟踪 (对齐 locate_face.py) -----
constexpr int FACE_PROC_MAX_W = 640;
constexpr float FACE_DETECT_SCORE_THRESH = 0.4f;
constexpr float FACE_NMS_THRESH = 0.3f;
constexpr int FACE_DETECT_TOP_K = 5000;
constexpr float FACE_ROI_PAD_RATIO = 0.30f;
constexpr float FACE_TRACK_GRACE_SEC = 1.2f;
constexpr float DEAD_BAND_X = 0.04f;
constexpr float DEAD_BAND_Y = 0.05f;
constexpr float K_YAW_DEG = 20.0f;
constexpr float K_PITCH_DEG = 15.0f;
constexpr float MAX_STEP_YAW_DEG = 6.0f;
constexpr float MAX_STEP_PITCH_DEG = 5.0f;
constexpr float TARGET_EMA_ALPHA = 0.6f;
constexpr float YAW_DX_SIGN = 1.0f;
constexpr float YAW_LIMIT_DEG = 80.0f;
constexpr float PITCH_UP_DEG = -40.0f;
constexpr float PITCH_DOWN_DEG = 60.0f;
constexpr int NECK_PUBLISH_RATE_HZ = 50;
constexpr float NO_FACE_RETURN_HOME_SEC = 1.0f;
constexpr float RETURN_HOME_RATE_DEG_PER_SEC = 45.0f;

// ----- 五指底盘跟随 (distance_hold.py) -----
constexpr float LATERAL_DEADBAND_NORM = 0.20f;
constexpr float ANGULAR_Z_MAG = 1.5f;
constexpr float LINEAR_X_MAG = 0.5f;
constexpr float TARGET_DISTANCE_M = 0.50f;
constexpr float DIST_DEADBAND_M = 0.10f;

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
    float confidence = 0.0f;
    cv::Rect hand_rect;
    bool has_hand = false;
    bool in_range = true;
    float distance_m = 0.0f;
    float dx_norm = 0.0f;
};

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
