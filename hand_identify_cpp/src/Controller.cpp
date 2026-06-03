#include "Controller.h"

#ifdef HAVE_SIM2REAL_MSG
#include <sim2real_msg/Joy.h>
#endif

Controller::Controller(ros::NodeHandle& nh, FaceTracker& face, HandTracker& hand)
    : face_tracker_(face),
      hand_tracker_(hand),
      coquette_player_(nh),
      joy_actions_(nh) {
#ifdef HAVE_SIM2REAL_MSG
    ROS_INFO(
        "actions: %s G2/G3; %s G4 policy (%.0fs)",
        JOY_MSG_TOPIC,
        ACTION_CONFIG_TOPIC,
        ACTION_DURATION_SEC);
#else
    ROS_WARN("sim2real_msg not linked; gestures 2-4 log only");
#endif
}

bool Controller::isActionBusy() const {
    return coquette_player_.isBusy() || joy_actions_.isBusy();
}

void Controller::abortActions() {
    joy_actions_.abort(false);
    coquette_player_.abort(false);
}

void Controller::releaseJoyMsg() {
#ifdef HAVE_SIM2REAL_MSG
    sim2real_msg::Joy release;
    // joy release handled inside JoyGestureActionPlayer::publishRelease on abort
    (void)release;
#endif
}

void Controller::stopAll() {
    face_tracker_.stopNeck();
    hand_tracker_.stopChassis();
    abortActions();
}

bool Controller::onConfirmedGesture(int gesture_id) {
    if (gesture_id <= GESTURE_0 || isActionBusy()) {
        return false;
    }

    if (gesture_id == GESTURE_1) {
        if (coquette_player_.start()) {
            ROS_INFO("[action] confirmed G1 -> coquette sway");
            return true;
        }
        return false;
    }

    if (gesture_id == GESTURE_2 || gesture_id == GESTURE_3) {
        if (joy_actions_.startTimedAction(gesture_id)) {
            ROS_INFO("[action] confirmed G%d -> timed joy action", gesture_id);
            return true;
        }
        return false;
    }

    if (gesture_id == GESTURE_4) {
        if (joy_actions_.startPolicyAction(GESTURE_4)) {
            ROS_INFO(
                "[action] confirmed G4 -> kick (%s@%s, %.0fs)",
                KICK_POLICY_NAME,
                ACTION_CONFIG_TOPIC,
                ACTION_DURATION_SEC);
            return true;
        }
    }
    return false;
}
