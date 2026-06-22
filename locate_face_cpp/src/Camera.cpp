#include "Camera.h"

Camera::Camera() {
    cap_.open(CAMERA_INDEX, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        cap_.open(CAMERA_INDEX);
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap_.set(cv::CAP_PROP_FPS, CAMERA_TARGET_FPS);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    is_opened_ = cap_.isOpened();
    if (!is_opened_) {
        ROS_ERROR("[cam] 无法打开相机 /dev/video%d", CAMERA_INDEX);
        return;
    }
    frame_w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    frame_h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const int eye_w = ZED_STEREO ? frame_w_ / 2 : frame_w_;
    ROS_INFO(
        "[cam] 原始 %dx%d  左眼 %dx%d  MediaPipe 最大宽 %d",
        frame_w_,
        frame_h_,
        eye_w,
        frame_h_,
        FACE_PROC_MAX_W);
}

Camera::~Camera() { release(); }

bool Camera::read(cv::Mat& frame) {
    if (!is_opened_) {
        return false;
    }
    cv::Mat raw;
    if (!cap_.read(raw) || raw.empty()) {
        return false;
    }
    if (ZED_STEREO && raw.cols >= 2) {
        frame = raw(cv::Rect(0, 0, raw.cols / 2, raw.rows)).clone();
    } else {
        frame = raw;
    }
    frame_w_ = frame.cols;
    frame_h_ = frame.rows;
    return true;
}

void Camera::release() {
    if (is_opened_) {
        cap_.release();
        is_opened_ = false;
    }
}
