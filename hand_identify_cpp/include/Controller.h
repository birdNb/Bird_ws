#pragma once

#include "Common.h"
#include "FaceTracker.h"
#include "HandTracker.h"
#include "GestureDetector.h"
#include "WaistCoquettePlayer.h"

class Controller {
public:
    Controller(ros::NodeHandle& nh, FaceTracker& face, HandTracker& hand);
    void onConfirmedGesture(int gesture_id);
    bool isActionBusy() const;
    void abortActions();
    void releaseJoyMsg();
    void stopAll();

private:
    void fireJoyMsgAction(int gesture_id);
#ifdef HAVE_SIM2REAL_MSG
    void publishJoyCombo(const std::string& combo, bool pressed);
#else
    void publishJoyCombo(const std::string& combo, bool pressed) {}
#endif

    FaceTracker& face_tracker_;
    HandTracker& hand_tracker_;
    WaistCoquettePlayer coquette_player_;
    ros::Publisher joy_msg_pub_;
    long long last_action_ms_ = 0;
    int hold_candidate_ = GESTURE_NONE;
    long long hold_since_ms_ = 0;
    int fired_episode_ = GESTURE_NONE;
    bool fired_ = false;
};
