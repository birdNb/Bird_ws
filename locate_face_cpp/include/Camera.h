#pragma once

#include "Common.h"

class Camera {
public:
    Camera();
    ~Camera();
    bool read(cv::Mat& frame);
    void release();
    int frameWidth() const { return frame_w_; }
    int frameHeight() const { return frame_h_; }

private:
    bool openAnyCamera();
    bool tryOpenIndex(int index);
    bool looksLikeColor(const cv::Mat& frame) const;

    cv::VideoCapture cap_;
    bool is_opened_ = false;
    bool sbs_stereo_ = false;  // 并排双目（ZED 等），取左半幅
    int camera_index_ = -1;
    int frame_w_ = 0;
    int frame_h_ = 0;
};
