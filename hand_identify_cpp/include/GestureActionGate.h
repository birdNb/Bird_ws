#pragma once

#include "Controller.h"
#include "GestureDecision.h"
#include "RobotOutputGate.h"

/** 手势稳定：平滑后 ID 与 raw 一致，降低误触发动作库/策略 */
inline bool gestureStableForAction(const HandDetectResult& hand) {
    if (!hand.has_hand || hand.gesture_id < GESTURE_0 || hand.gesture_id > GESTURE_4) {
        return false;
    }
    if (hand.raw_gesture_id < GESTURE_0) {
        return true;
    }
    return hand.raw_gesture_id == hand.gesture_id;
}

inline int gestureHoldMsFor(int gesture) {
    return gesture == GESTURE_4 ? GESTURE_POLICY_HOLD_MS : GESTURE_HOLD_MS;
}

class GestureActionGate {
public:
    void process(
        Controller& controller,
        const RobotOutputGate& gate,
        const GestureDecision& decision,
        const HandDetectResult& hand,
        int& hold_candidate,
        long long& hold_since_ms);

    void reset() {
        hold_candidate_ = GESTURE_NONE;
        hold_since_ms_ = 0;
        last_fired_confirmed_ = GESTURE_NONE;
        lost_since_ms_ = 0;
    }

private:
    int hold_candidate_ = GESTURE_NONE;
    long long hold_since_ms_ = 0;
    long long lost_since_ms_ = 0;
    int last_fired_confirmed_ = GESTURE_NONE;

    bool handLostTooLong(bool has_hand, long long now);
    int updateHold(int gesture, bool has_hand, long long now);
};
