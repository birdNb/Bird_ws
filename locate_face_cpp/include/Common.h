#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// ----- 相机 (ZED Mini 左眼, 对齐 locate_face.py) -----
constexpr int CAMERA_INDEX = 0;
constexpr int CAMERA_WIDTH = 2560;
constexpr int CAMERA_HEIGHT = 720;
constexpr int CAMERA_TARGET_FPS = 30;
constexpr bool ZED_STEREO = true;
constexpr int PROC_MAX_W = 960;

// ----- 人脸检测 / 控制 (对齐 locate_face.py) -----
constexpr int FACE_PROC_MAX_W = PROC_MAX_W;
constexpr float FACE_DETECT_SCORE_THRESH = 0.4f;
constexpr float FACE_NMS_THRESH = 0.3f;
constexpr int FACE_DETECT_TOP_K = 5000;
constexpr float FACE_ROI_PAD_RATIO = 0.30f;
constexpr float DEAD_BAND_X = 0.04f;
constexpr float DEAD_BAND_Y = 0.05f;
constexpr float K_YAW_DEG = 20.0f;
constexpr float K_PITCH_DEG = 15.0f;
constexpr float MAX_STEP_YAW_DEG = 6.0f;
constexpr float MAX_STEP_PITCH_DEG = 5.0f;
constexpr float TARGET_EMA_ALPHA = 0.60f;
constexpr float YAW_LIMIT_DEG = 80.0f;
constexpr float PITCH_UP_DEG = -40.0f;
constexpr float PITCH_DOWN_DEG = 60.0f;
constexpr int NECK_PUBLISH_RATE_HZ = 50;
constexpr float NO_FACE_RETURN_HOME_SEC = 1.0f;
constexpr float RETURN_HOME_RATE_DEG_PER_SEC = 45.0f;

// ----- ROS -----
constexpr const char* ABSOLUTE_TOPIC = "/pi_plus_absolute";
constexpr const char* FSM_STATE_TOPIC = "/fsm_state";
constexpr int FSM_EXEC_DEFAULT = 5;
constexpr float FSM_WAIT_TIMEOUT_SEC = 30.0f;
constexpr const char* HEAD_YAW_JOINT = "head_yaw_joint";
constexpr const char* HEAD_PITCH_JOINT = "head_pitch_joint";
constexpr const char* NECK_STATE_FILE = "/tmp/locate_face_neck.state";

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
    const char* env = std::getenv("LOCATE_FACE_CPP_ROOT");
    if (env && env[0]) {
        return std::string(env);
    }
    return std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/Bird_ws/locate_face_cpp";
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

inline void fitLetterbox(const cv::Mat& src, cv::Mat& dst, int target_w, int target_h) {
    if (src.empty() || target_w <= 0 || target_h <= 0) {
        dst.release();
        return;
    }
    const float scale = std::min(
        target_w / static_cast<float>(src.cols), target_h / static_cast<float>(src.rows));
    const int nw = std::max(1, static_cast<int>(src.cols * scale));
    const int nh = std::max(1, static_cast<int>(src.rows * scale));
    cv::Mat canvas(target_h, target_w, src.type(), cv::Scalar(0, 0, 0));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    const int x = (target_w - nw) / 2;
    const int y = (target_h - nh) / 2;
    resized.copyTo(canvas(cv::Rect(x, y, nw, nh)));
    dst = canvas;
}
