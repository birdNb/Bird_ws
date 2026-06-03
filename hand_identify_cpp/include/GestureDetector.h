#pragma once

#include "Common.h"
#include "MediaPipeGestureBridge.h"

class GestureDetector {
public:
    GestureDetector();
    /** MediaPipe 多手取面积最大；返回是否有效识别(in_range + 平滑手势>=0) */
    bool detectMaxHand(const cv::Mat& frame, HandDetectResult& out);
    bool isReady() const { return bridge_.isRunning(); }

private:
    MediaPipeGestureBridge bridge_;
};
