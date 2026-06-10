#include "GestureActionGate.h"

bool GestureActionGate::handLostTooLong(bool has_hand, long long now) {
    if (has_hand) {
        lost_since_ms_ = 0;
        return false;
    }
    if (lost_since_ms_ <= 0) {
        lost_since_ms_ = now;
    }
    return (now - lost_since_ms_) >= HAND_LOST_GRACE_MS;
}

int GestureActionGate::updateHold(int gesture, bool has_hand, long long now) {
    if (handLostTooLong(has_hand, now)) {
        hold_candidate_ = GESTURE_NONE;
        hold_since_ms_ = 0;
        return GESTURE_NONE;
    }
    if (gesture < GESTURE_1 || gesture > GESTURE_4) {
        hold_candidate_ = GESTURE_NONE;
        hold_since_ms_ = 0;
        return GESTURE_NONE;
    }
    if (gesture != hold_candidate_) {
        hold_candidate_ = gesture;
        hold_since_ms_ = now;
        return GESTURE_NONE;
    }
    if (hold_since_ms_ <= 0) {
        hold_since_ms_ = now;
    }
    if (now - hold_since_ms_ >= gestureHoldMsFor(gesture)) {
        return gesture;
    }
    return GESTURE_NONE;
}

void GestureActionGate::process(
    Controller& controller,
    const RobotOutputGate& gate,
    const GestureDecision& decision,
    const HandDetectResult& hand,
    int& hold_candidate,
    long long& hold_since_ms) {
    hold_candidate = hold_candidate_;
    hold_since_ms = hold_since_ms_;

    const int gesture = decision.gesture;
    if (!decision.action_ready || gesture < GESTURE_0 || gesture > GESTURE_4) {
        last_fired_confirmed_ = GESTURE_NONE;
        return;
    }
    if (!gestureStableForAction(hand)) {
        last_fired_confirmed_ = GESTURE_NONE;
        return;
    }

    if (gesture == GESTURE_0) {
        if (gate.allowGestureSideEffects()) {
            controller.abortActions();
            ROS_INFO_THROTTLE(1.0, "[action] G0 estop -> abort");
        }
        last_fired_confirmed_ = GESTURE_NONE;
        return;
    }

    if (!gate.allowGestureSideEffects()) {
        last_fired_confirmed_ = GESTURE_NONE;
        return;
    }

    const long long now = getCurrentTimeMs();
    const int confirmed = updateHold(gesture, hand.has_hand, now);
    hold_candidate = hold_candidate_;
    hold_since_ms = hold_since_ms_;

    if (gesture != confirmed) {
        last_fired_confirmed_ = GESTURE_NONE;
    }

    if (confirmed >= GESTURE_1) {
        if (confirmed != last_fired_confirmed_ && !controller.isActionBusy()) {
            if (controller.onConfirmedGesture(confirmed)) {
                last_fired_confirmed_ = confirmed;
            }
        }
    }
}
