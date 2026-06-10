#include "GestureDetector.h"

#include "GestureDecision.h"

#include <exception>

GestureDetector::GestureDetector() {
    const std::string script = projectRoot() + "/scripts/gesture_mediapipe_worker.py";
    if (bridge_.start(script)) {
        ROS_INFO(
            "gesture backend: MediaPipe Hands (zed_gesture_recognition.py), smooth=%d",
            GESTURE_SMOOTH_FRAMES);
    } else {
        ROS_ERROR("gesture MediaPipe worker failed: %s", script.c_str());
    }
}

bool GestureDetector::detectMaxHand(const cv::Mat& frame, HandDetectResult& out) {
    out = HandDetectResult{};
    if (frame.empty()) {
        return false;
    }
    if (!bridge_.isRunning()) {
        const std::string script = projectRoot() + "/scripts/gesture_mediapipe_worker.py";
        if (!bridge_.start(script)) {
            return false;
        }
        ROS_WARN("[gesture] worker restarted");
    }

    try {
        MediaPipeGestureResult mp;
        if (!bridge_.detect(frame, mp) || !mp.has_hand) {
            return false;
        }

    out.has_hand = true;
    out.hand_rect = mp.hand_rect & cv::Rect(0, 0, frame.cols, frame.rows);
    if (static_cast<int>(mp.landmarks.size()) == HAND_LANDMARK_COUNT) {
        out.landmarks = mp.landmarks;
        out.has_landmarks = true;
    }
    out.dx_norm = mp.dx_norm;
    out.dy_norm = mp.dy_norm;
    out.distance_m = mp.distance_m;
    out.in_range = mp.in_range;
    out.palm_or_back_facing = true;
    out.raw_gesture_id =
        (mp.raw_gesture_id >= 0 && mp.raw_gesture_id <= 5) ? mp.raw_gesture_id : GESTURE_NONE;
    out.gesture_id =
        (mp.gesture_id >= 0 && mp.gesture_id <= 5) ? mp.gesture_id : GESTURE_NONE;

    if (out.gesture_id < 0 && out.raw_gesture_id >= 0) {
        out.gesture_id = out.raw_gesture_id;
    }
    if (out.gesture_id < 0) {
        out.gesture_id = GESTURE_NONE;
    }

        applyGestureRangePolicy(out);
        out.confidence = computeGestureConfidence(out);
        return true;
    } catch (const cv::Exception& e) {
        ROS_WARN_THROTTLE(2.0, "[gesture] OpenCV error: %s", e.what());
        bridge_.stop();
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(2.0, "[gesture] error: %s", e.what());
        bridge_.stop();
    }
    return false;
}
