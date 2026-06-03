#include "Camera.h"

Camera::Camera() {
    cap_.open(CAMERA_INDEX, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        cap_.open(CAMERA_INDEX);
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    is_opened_ = cap_.isOpened();
    if (!is_opened_) {
        ROS_ERROR("camera open failed index=%d", CAMERA_INDEX);
        return;
    }
    frame_w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    frame_h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("camera opened %dx%d (ZED_STEREO=%d)", frame_w_, frame_h_, ZED_STEREO ? 1 : 0);
}

Camera::~Camera() { release(); }

bool Camera::read(cv::Mat& frame) {
    if (!is_opened_) return false;
    cv::Mat raw;
    if (!cap_.read(raw) || raw.empty()) return false;

    if (ZED_STEREO && raw.cols >= 2 && raw.rows >= 2) {
        int half = raw.cols / 2;
        frame = raw(cv::Rect(0, 0, half, raw.rows)).clone();
    } else {
        frame = raw;
    }

    if (frame.cols != DISPLAY_W || frame.rows != DISPLAY_H) {
        cv::resize(frame, frame, cv::Size(DISPLAY_W, DISPLAY_H), 0, 0, cv::INTER_AREA);
    }
    return true;
}

void Camera::release() {
    if (is_opened_) {
        cap_.release();
        is_opened_ = false;
    }
}
