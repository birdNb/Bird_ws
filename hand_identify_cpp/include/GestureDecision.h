#pragma once

#include "Common.h"

/** 仅五指跟手(G5)做 bbox 距离/面积门控；0~4 不判距 */
inline bool gestureRequiresDistanceGate(int gesture_id) {
    return gesture_id == GESTURE_5;
}

inline void applyGestureRangePolicy(HandDetectResult& hand) {
    if (!hand.has_hand) {
        return;
    }
    if (!gestureRequiresDistanceGate(hand.gesture_id)) {
        hand.in_range = true;
    }
}

inline float computeGestureConfidence(const HandDetectResult& hand) {
    if (!hand.has_hand || hand.gesture_id < GESTURE_0) {
        return 0.0f;
    }
    if (gestureRequiresDistanceGate(hand.gesture_id) && !hand.in_range) {
        return 0.55f;
    }
    if (hand.raw_gesture_id >= GESTURE_0 && hand.raw_gesture_id != hand.gesture_id) {
        return 0.85f;
    }
    return 1.0f;
}

/** G0~G4 动作/脸跟踪阶段：有手即可，不判距离 */
inline bool gestureActionReady(const HandDetectResult& hand) {
    if (!hand.has_hand || hand.gesture_id < GESTURE_0 || hand.gesture_id > GESTURE_4) {
        return false;
    }
    return computeGestureConfidence(hand) >= GESTURE_CONFIDENCE_THRESH;
}

/** G5 五指跟手：必须 in_range */
inline bool gestureFollowReady(const HandDetectResult& hand) {
    if (!hand.has_hand || hand.gesture_id != GESTURE_5) {
        return false;
    }
    return hand.in_range && computeGestureConfidence(hand) >= GESTURE_CONFIDENCE_THRESH;
}

struct GestureDecision {
    int gesture = GESTURE_NONE;
    bool has_hand = false;
    bool action_ready = false;
    bool follow_ready = false;
    /** G0~4 且可触发动作/脸跟踪 */
    bool companion_face_phase = false;
};

inline GestureDecision evaluateGestureDecision(const HandDetectResult& hand) {
    GestureDecision d;
    d.has_hand = hand.has_hand;
    d.gesture = hand.has_hand ? hand.gesture_id : GESTURE_NONE;
    d.action_ready = gestureActionReady(hand);
    d.follow_ready = gestureFollowReady(hand);
    d.companion_face_phase =
        d.action_ready && d.gesture >= GESTURE_0 && d.gesture <= GESTURE_4;
    return d;
}

/** 伴生模式：tick%2==0 脸检，==1 手势检，同帧不双 IPC */
constexpr int GESTURE_DETECT_SLOT = 1;
constexpr int HAND_CACHE_MAX_AGE_MS = 300;

inline bool shouldRunGestureDetectThisFrame(int tick, bool need_gesture, bool companion_mode) {
    if (!need_gesture) {
        return false;
    }
    if (!companion_mode) {
        return true;
    }
    return (tick % FACE_DETECT_EVERY_N) != GESTURE_DETECT_SLOT;
}

inline bool shouldRunFaceDetectThisFrame(int tick, bool companion_mode) {
    if (!companion_mode) {
        return true;
    }
    return (tick % FACE_DETECT_EVERY_N) == GESTURE_DETECT_SLOT;
}
