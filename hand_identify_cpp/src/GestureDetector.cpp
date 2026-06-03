#include "GestureDetector.h"

#include <fstream>
#include <map>

namespace {

cv::Mat largestSkinHand(const cv::Mat& bgr, cv::Rect& out_bbox) {
    cv::Mat hsv, mask, open_mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 60), cv::Scalar(25, 255, 255), mask);
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(160, 30, 60), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask, mask2, mask);
    cv::morphologyEx(mask, open_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::morphologyEx(open_mask, open_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(open_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0.0;
    out_bbox = cv::Rect();
    for (const auto& c : contours) {
        cv::Rect r = cv::boundingRect(c);
        double area = r.area();
        if (area < bgr.cols * bgr.rows * 0.01) continue;
        if (area > best_area) {
            best_area = area;
            out_bbox = r;
        }
    }
    return open_mask;
}

}  // namespace

GestureDetector::GestureDetector() {
    const std::string onnx_path = projectRoot() + "/model/gesture.onnx";
    if (!std::ifstream(onnx_path).good()) {
        ROS_WARN("ONNX not found: %s, using contour fingers", onnx_path.c_str());
        return;
    }
    try {
        onnx_net_ = cv::dnn::readNetFromONNX(onnx_path);
        if (onnx_net_.empty()) {
            ROS_WARN("readNetFromONNX empty: %s", onnx_path.c_str());
            return;
        }
        use_onnx_ = true;
        ROS_INFO("gesture ONNX loaded: %s (1x3x224x224 -> 6 classes)", onnx_path.c_str());
    } catch (const cv::Exception& e) {
        ROS_WARN("ONNX load failed (%s), using contour fingers", e.what());
    }
}

int GestureDetector::smoothGesture(int raw) {
    if (raw < 0) {
        hist_.clear();
        return GESTURE_NONE;
    }
    hist_.push_back(raw);
    if (static_cast<int>(hist_.size()) > kSmoothWindow) {
        hist_.erase(hist_.begin());
    }
    std::map<int, int> cnt;
    for (int g : hist_) cnt[g]++;
    int best = raw;
    int best_n = 0;
    for (const auto& kv : cnt) {
        if (kv.second > best_n) {
            best_n = kv.second;
            best = kv.first;
        }
    }
    return best;
}

float GestureDetector::estimateConfidence(int gesture, const cv::Rect& bbox, int defects) const {
    if (bbox.area() <= 0) return 0.0f;
    float area_ratio = static_cast<float>(bbox.area()) / static_cast<float>(640 * 480);
    float area_score = clampf(area_ratio * 8.0f, 0.0f, 1.0f);
    float defect_score = (gesture >= 0 && gesture <= 5 && defects >= 0)
                             ? 1.0f - std::abs(defects - gesture) * 0.15f
                             : 0.3f;
    return clampf(0.55f * area_score + 0.45f * defect_score, 0.0f, 1.0f);
}

int GestureDetector::recognizeFingers(const cv::Mat& hand_roi, const cv::Rect& bbox) const {
    if (hand_roi.empty() || bbox.width < 30 || bbox.height < 30) return GESTURE_NONE;

    cv::Mat gray, blur, thresh;
    cv::cvtColor(hand_roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0);
    cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return GESTURE_NONE;

    auto it = std::max_element(contours.begin(), contours.end(),
                               [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                   return cv::contourArea(a) < cv::contourArea(b);
                               });
    std::vector<cv::Point> hull;
    cv::convexHull(*it, hull);
    std::vector<int> hull_idx;
    std::vector<cv::Vec4i> defects;
    cv::convexHull(*it, hull_idx);
    if (hull_idx.size() > 3 && cv::contourArea(*it) > 1000) {
        cv::convexityDefects(*it, hull_idx, defects);
    }
    int finger_count = 0;
    for (const auto& d : defects) {
        float depth = d[3] / 256.0f;
        if (depth > 12.0f) finger_count++;
    }
    finger_count = std::min(5, std::max(0, finger_count));
    if (finger_count == 0 && cv::contourArea(*it) > bbox.area() * 0.35f) {
        return GESTURE_5;
    }
    return finger_count;
}

bool GestureDetector::detectMaxHand(const cv::Mat& frame, HandDetectResult& out) {
    out = HandDetectResult{};
    if (frame.empty()) return false;

    int proc_w = frame.cols;
    int proc_h = frame.rows;
    cv::Mat proc = frame;
    if (frame.cols > PROC_MAX_W) {
        proc_w = PROC_MAX_W;
        proc_h = std::max(1, static_cast<int>(frame.rows * PROC_MAX_W / frame.cols));
        cv::resize(frame, proc, cv::Size(proc_w, proc_h), 0, 0, cv::INTER_AREA);
    }

    cv::Rect bbox;
    largestSkinHand(proc, bbox);
    if (bbox.area() <= 0) {
        out.gesture_id = smoothGesture(GESTURE_NONE);
        return false;
    }

    float sx = static_cast<float>(frame.cols) / proc_w;
    float sy = static_cast<float>(frame.rows) / proc_h;
    out.hand_rect = cv::Rect(
        static_cast<int>(bbox.x * sx),
        static_cast<int>(bbox.y * sy),
        static_cast<int>(bbox.width * sx),
        static_cast<int>(bbox.height * sy));
    out.has_hand = true;

    int cx = out.hand_rect.x + out.hand_rect.width / 2;
    out.dx_norm = (cx - frame.cols / 2) / (frame.cols / 2.0f);
    out.distance_m = TARGET_DISTANCE_M;

    cv::Mat roi = proc(bbox).clone();
    int raw = recognizeFingers(roi, bbox);
    out.confidence = estimateConfidence(raw, out.hand_rect, raw);

    if (use_onnx_ && !onnx_net_.empty()) {
        cv::Rect roi = out.hand_rect;
        roi &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (roi.area() > 0) {
            cv::Mat hand_bgr = frame(roi).clone();
            cv::Mat blob = cv::dnn::blobFromImage(
                hand_bgr, 1.0 / 255.0, cv::Size(224, 224), cv::Scalar(), true, false);
            onnx_net_.setInput(blob);
            cv::Mat output = onnx_net_.forward();
            cv::Mat probs = output.reshape(1, static_cast<int>(output.total()));
            if (probs.cols >= NUM_GESTURE_CLASSES) {
                cv::Mat row = probs.colRange(0, NUM_GESTURE_CLASSES);
                cv::Point max_loc;
                double max_val = 0.0;
                cv::minMaxLoc(row, nullptr, &max_val, nullptr, &max_loc);
                raw = max_loc.x;
                out.confidence = static_cast<float>(max_val);
            }
        }
    }

    out.gesture_id = smoothGesture(raw);
    out.in_range = true;
    bool ok = out.confidence >= GESTURE_CONFIDENCE_THRESH;
    return ok;
}
