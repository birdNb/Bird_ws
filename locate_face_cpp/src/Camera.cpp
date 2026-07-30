#include "Camera.h"

#include <algorithm>
#include <cmath>
#include <unistd.h>
#include <vector>

namespace {
void setupCapture(cv::VideoCapture& cap) {
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, CAMERA_TARGET_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
}
}  // namespace

Camera::Camera() {
    is_opened_ = openAnyCamera();
    if (!is_opened_) {
        ROS_ERROR("[cam] 无法打开 D435i RGB 节点（尝试过 /dev/video%d 等）", CAMERA_INDEX);
        ROS_ERROR("[cam] 请检查: D435i 是否接好、是否被其它进程占用、设备节点是否存在");
        return;
    }
    frame_w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    frame_h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    // ZED 等并排双目：宽约等于高的 2 倍以上，或编译期强制 ZED_STEREO
    sbs_stereo_ = ZED_STEREO || (frame_w_ >= frame_h_ * 2);
    const int eye_w = sbs_stereo_ ? frame_w_ / 2 : frame_w_;
    ROS_INFO(
        "[cam] 使用 /dev/video%d, 原始 %dx%d  %s %dx%d  检测最大宽 %d",
        camera_index_,
        frame_w_,
        frame_h_,
        sbs_stereo_ ? "左眼" : "画面",
        eye_w,
        frame_h_,
        FACE_PROC_MAX_W);
}

Camera::~Camera() { release(); }

bool Camera::looksLikeColor(const cv::Mat& frame) const {
    if (frame.empty() || frame.channels() != 3) {
        return false;
    }
    std::vector<cv::Mat> ch;
    cv::split(frame, ch);
    cv::Mat diff_rb;
    cv::absdiff(ch[2], ch[0], diff_rb);
    cv::Scalar mean_diff = cv::mean(diff_rb);
    // 红外 GREY 通常三通道几乎完全一致；彩色流 R/B 有明显差异
    return mean_diff[0] > 2.0;
}

bool Camera::tryOpenIndex(int index) {
    const std::string dev = "/dev/video" + std::to_string(index);
    if (access(dev.c_str(), F_OK) != 0) {
        return false;
    }

    cv::VideoCapture cap(index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(index);
    }
    if (!cap.isOpened()) {
        return false;
    }

    setupCapture(cap);
    cv::Mat probe;
    bool got = false;
    for (int i = 0; i < 8; ++i) {
        if (cap.read(probe) && !probe.empty()) {
            got = true;
            break;
        }
        usleep(60 * 1000);
    }
    if (!got || !looksLikeColor(probe)) {
        cap.release();
        return false;
    }

    cap_ = std::move(cap);
    camera_index_ = index;
    return true;
}

bool Camera::openAnyCamera() {
    // Orin+ZED 常见 /dev/video0；RK+D435i 常见 /dev/video4
    std::vector<int> candidates = {CAMERA_INDEX, 0, 4, 6, 2, 8};
    for (int i = 0; i <= 12; ++i) {
        if (std::find(candidates.begin(), candidates.end(), i) == candidates.end()) {
            candidates.push_back(i);
        }
    }
    for (int idx : candidates) {
        if (tryOpenIndex(idx)) {
            return true;
        }
    }
    return false;
}

bool Camera::read(cv::Mat& frame) {
    if (!is_opened_) {
        return false;
    }
    cv::Mat raw;
    if (!cap_.read(raw) || raw.empty()) {
        return false;
    }
    const bool sbs = sbs_stereo_ || (raw.cols >= raw.rows * 2);
    if (sbs && raw.cols >= 2) {
        sbs_stereo_ = true;
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
