#pragma once

#include "Common.h"

class GestureDetector {
public:
    GestureDetector();
    /** 多手时取画面面积最大的手，返回是否达到置信度阈值 */
    bool detectMaxHand(const cv::Mat& frame, HandDetectResult& out);

private:
    int recognizeFingers(const cv::Mat& hand_roi, const cv::Rect& bbox) const;
    int smoothGesture(int raw);
    float estimateConfidence(int gesture, const cv::Rect& bbox, int defects) const;

    std::vector<int> hist_;
    static constexpr int kSmoothWindow = 5;
    cv::dnn::Net onnx_net_;
    bool use_onnx_ = false;
};
