#pragma once

#include "Common.h"
#include "FaceTracker.h"
#include "HandTracker.h"
#include "JoyGestureActionPlayer.h"
#include "WaistCoquettePlayer.h"

class Controller {
public:
    Controller(ros::NodeHandle& nh, FaceTracker& face, HandTracker& hand);
    /** @return 是否已下发动作（对齐 gesture_motion ConfirmedActionGate） */
    bool onConfirmedGesture(int gesture_id);
    bool isActionBusy() const;
    void abortActions();
    void releaseJoyMsg();
    void stopAll();

private:
    FaceTracker& face_tracker_;
    HandTracker& hand_tracker_;
    WaistCoquettePlayer coquette_player_;
    JoyGestureActionPlayer joy_actions_;
};
