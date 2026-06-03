#pragma once

#include <algorithm>
#include <string>

#include <opencv2/opencv.hpp>

#include "Common.h"
#include "DebugMode.h"
#include "FaceTracker.h"
#include "GestureDecision.h"
#include "HandTracker.h"
#include "JoyMonitor.h"

/** 连续无 G5 超时后结束跟手（distance_hold.py GestureFiveLostWatch） */
class GestureFiveLostWatch {
public:
    bool shouldEndFollow(bool follow_engaged, bool is_gesture_five) {
        if (!follow_engaged) {
            lost_since_ms_ = 0;
            return false;
        }
        const long long now = getCurrentTimeMs();
        if (is_gesture_five) {
            lost_since_ms_ = 0;
            return false;
        }
        if (lost_since_ms_ <= 0) {
            lost_since_ms_ = now;
        }
        return (now - lost_since_ms_) >= GESTURE_FOLLOW_LOST_MS;
    }

    float lostElapsedSec() const {
        if (lost_since_ms_ <= 0) {
            return 0.0f;
        }
        return (getCurrentTimeMs() - lost_since_ms_) / 1000.0f;
    }

    float lostRemainingSec() const {
        if (lost_since_ms_ <= 0) {
            return 0.0f;
        }
        return std::max(
            0.0f,
            (GESTURE_FOLLOW_LOST_MS - (getCurrentTimeMs() - lost_since_ms_)) / 1000.0f);
    }

    void reset() { lost_since_ms_ = 0; }

private:
    long long lost_since_ms_ = 0;
};

/** 手掌入画即进入跟手（distance_hold PalmBootState） */
class PalmBootState {
public:
    bool isEngaged() const { return engaged_; }

    void reset() {
        engaged_ = false;
        lost_since_ms_ = 0;
    }

    void update(bool has_palm) {
        const long long now = getCurrentTimeMs();
        if (has_palm) {
            lost_since_ms_ = 0;
        } else {
            if (lost_since_ms_ <= 0) {
                lost_since_ms_ = now;
            }
            if (now - lost_since_ms_ > PALM_LOST_RESET_MS) {
                reset();
                return;
            }
        }

        if (!engaged_ && has_palm) {
            engaged_ = true;
            ROS_INFO("[htrack] 识别手掌，进入手部跟踪");
        }
    }

private:
    bool engaged_ = false;
    long long lost_since_ms_ = 0;
};

/**
 * 五指跟手：distance_hold 控制律 + G5 连续 5s 确认后脖子复位。
 */
class HandFollowController {
public:
    bool g5Confirmed() const { return g5_confirmed_; }
    bool holdPending() const { return hold_candidate_ == GESTURE_5; }
    bool palmEngaged() const { return palm_boot_.isEngaged(); }

    int holdProgressPct() const {
        if (!holdPending() || hold_since_ms_ <= 0) {
            return 0;
        }
        const long long now = getCurrentTimeMs();
        return std::min(
            100,
            static_cast<int>(100 * (now - hold_since_ms_) / std::max(1, GESTURE_FOLLOW_HOLD_MS)));
    }

    const std::string& statusMode() const { return status_mode_; }

    bool shouldPauseCompanionFace(bool /*companion_face*/) const { return g5_confirmed_; }

    static void pauseCompanionFace(FaceTracker& face_tracker) {
        face_tracker.setEnabled(false);
        face_tracker.stopNeck();
    }

    void reset(HandTracker& hand_tracker, FaceTracker* face_tracker = nullptr) {
        g5_confirmed_ = false;
        resetG5Hold();
        palm_boot_.reset();
        g5_watch_.reset();
        status_mode_ = "idle";
        hand_tracker.stopChassis();
        if (face_tracker != nullptr) {
            face_tracker->setEnabled(true);
            ROS_INFO("[htrack] resume face track");
        }
    }

    void onJoyTakeoverEdge(JoyMonitor& joy) {
        if (joy.pollTakeoverEdge()) {
            ROS_INFO(
                "[htrack] joy takeover: pause /cmd_vel %ds (no zero twist)",
                HAND_TRACKING_JOY_IDLE_MS / 1000);
        }
    }

    bool update(
        cv::Mat& frame,
        HandTracker& hand_tracker,
        FaceTracker& face_tracker,
        JoyMonitor& joy_monitor,
        const AppConfig& cfg,
        const GestureDecision& decision,
        const HandDetectResult& hand,
        bool companion_face) {
        const bool joy_blocking = !cfg.no_joy && joy_monitor.blocksHandTracking();
        onJoyTakeoverEdge(joy_monitor);

        if (!cfg.no_joy && joy_monitor.blocksHandTracking()) {
            resetG5Hold();
        }

        const bool instant_g5 = (cfg.mode == RunMode::HandFollow);

        if (!g5_confirmed_) {
            if (instant_g5 && decision.follow_ready) {
                g5_confirmed_ = true;
                pauseCompanionFace(face_tracker);
                ROS_INFO("[htrack] G5 ready -> distance_hold follow");
            } else if (!tryConfirmG5(decision, hand, joy_monitor, cfg)) {
                status_mode_ = joy_blocking ? "joy_wait_g5" : "g5_hold";
                hand_tracker.tickPublish(HandFollowCmd(), joy_blocking);
                hand_tracker.logFollowStatus(hand, HandFollowCmd(), joy_blocking, 0.0f, 0.0f);
                return false;
            } else {
                g5_confirmed_ = true;
                pauseCompanionFace(face_tracker);
                ROS_INFO(
                    "[htrack] G5 confirmed %.1fs -> neck reset, distance_hold",
                    GESTURE_FOLLOW_HOLD_MS / 1000.0f);
            }
        }

        const bool has_palm = hand.has_hand && hand.in_range;
        palm_boot_.update(has_palm);

        const bool is_gesture_five =
            hand.has_hand && hand.in_range && hand.gesture_id == GESTURE_5;

        if (g5_watch_.shouldEndFollow(g5_confirmed_, is_gesture_five)) {
            ROS_INFO(
                "[htrack] no G5 for %.0fs -> resume face",
                GESTURE_FOLLOW_LOST_MS / 1000.0f);
            reset(hand_tracker, companion_face ? &face_tracker : nullptr);
            return false;
        }

        const bool engaged = palm_boot_.isEngaged() && g5_confirmed_;
        const bool dist_follow = engaged && is_gesture_five;

        HandFollowCmd cmd;
        if (joy_blocking) {
            status_mode_ = "joy";
        } else if (!engaged) {
            status_mode_ = has_palm ? "detect" : "idle";
        } else {
            cmd = hand_tracker.computeDistanceHold(hand, engaged, dist_follow);
            status_mode_ = cmd.mode;
        }

        hand_tracker.tickPublish(cmd, joy_blocking);
        hand_tracker.drawFollowOverlay(frame, hand);

        const float joy_wait = joy_blocking
            ? static_cast<float>((joy_monitor.idleRemainingMs() + 999) / 1000)
            : 0.0f;
        const float g5_back = (engaged && !is_gesture_five) ? g5_watch_.lostRemainingSec() : 0.0f;
        hand_tracker.logFollowStatus(hand, cmd, joy_blocking, joy_wait, g5_back);

        return engaged && cmd.hasMotion() && !joy_blocking;
    }

private:
    bool g5_confirmed_ = false;
    int hold_candidate_ = GESTURE_NONE;
    long long hold_since_ms_ = 0;
    long long hold_lost_since_ms_ = 0;
    std::string status_mode_ = "idle";
    PalmBootState palm_boot_;
    GestureFiveLostWatch g5_watch_;

    void resetG5Hold() {
        hold_candidate_ = GESTURE_NONE;
        hold_since_ms_ = 0;
        hold_lost_since_ms_ = 0;
    }

    static bool trackingLost(bool has_hand, bool in_range, long long now, long long& lost_since) {
        if (has_hand && in_range) {
            lost_since = 0;
            return false;
        }
        if (lost_since <= 0) {
            lost_since = now;
        }
        return (now - lost_since) >= HAND_LOST_GRACE_MS;
    }

    bool tryConfirmG5(
        const GestureDecision& decision,
        const HandDetectResult& hand,
        const JoyMonitor& joy,
        const AppConfig& cfg) {
        if (!cfg.no_joy && joy.blocksHandTracking()) {
            return false;
        }
        if (!decision.follow_ready || decision.gesture != GESTURE_5) {
            resetG5Hold();
            return false;
        }
        const long long now = getCurrentTimeMs();
        if (trackingLost(hand.has_hand, hand.in_range, now, hold_lost_since_ms_)) {
            resetG5Hold();
            return false;
        }
        if (decision.gesture != hold_candidate_) {
            hold_candidate_ = decision.gesture;
            hold_since_ms_ = now;
            return false;
        }
        if (hold_since_ms_ <= 0) {
            hold_since_ms_ = now;
        }
        return (now - hold_since_ms_) >= GESTURE_FOLLOW_HOLD_MS;
    }
};
