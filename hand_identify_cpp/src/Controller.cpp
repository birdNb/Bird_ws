#include "Controller.h"

#ifdef HAVE_SIM2REAL_MSG
#include <sim2real_msg/Joy.h>
#endif

Controller::Controller(ros::NodeHandle& nh, FaceTracker& face, HandTracker& hand)
    : face_tracker_(face),
      hand_tracker_(hand),
      coquette_player_(nh) {
#ifdef HAVE_SIM2REAL_MSG
    joy_msg_pub_ = nh.advertise<sim2real_msg::Joy>(JOY_MSG_TOPIC, 1);
    ROS_INFO("动作库发布: %s", JOY_MSG_TOPIC);
#else
    ROS_WARN("未链接 sim2real_msg，手势2~4 仅日志");
#endif
}

bool Controller::isActionBusy() const {
    if (coquette_player_.isBusy()) return true;
    return getCurrentTimeMs() - last_action_ms_ < ACTION_COOLDOWN_MS;
}

void Controller::abortActions() {
    coquette_player_.abort(false);
#ifdef HAVE_SIM2REAL_MSG
    sim2real_msg::Joy release;
    joy_msg_pub_.publish(release);
#endif
}

void Controller::stopAll() {
    face_tracker_.stopNeck();
    hand_tracker_.stopChassis();
    abortActions();
}

#ifdef HAVE_SIM2REAL_MSG
void Controller::publishJoyCombo(const std::string& combo, bool pressed) {
    sim2real_msg::Joy msg;
    auto set_btn = [&](const char* key, float v) {
        std::string k(key);
        if (k == "a") msg.a = v;
        else if (k == "b") msg.b = v;
        else if (k == "x") msg.x = v;
        else if (k == "y") msg.y = v;
        else if (k == "rt") msg.rt = v;
        else if (k == "lt") msg.lt = v;
    };
    const float press = 1.0f;
    const float release = 0.0f;
    const float trig_press = -1.0f;
    const float trig_release = 1.0f;

    size_t start = 0;
    while (start < combo.size()) {
        size_t plus = combo.find('+', start);
        std::string token = combo.substr(start, plus == std::string::npos ? std::string::npos : plus - start);
        if (!token.empty()) {
            float v = release;
            if (token == "rt" || token == "lt") {
                v = pressed ? trig_press : trig_release;
            } else {
                v = pressed ? press : release;
            }
            set_btn(token.c_str(), v);
        }
        if (plus == std::string::npos) break;
        start = plus + 1;
    }
    joy_msg_pub_.publish(msg);
}

void Controller::fireJoyMsgAction(int gesture_id) {
    struct Spec { int g; const char* combo; const char* label; };
    static const Spec kSpecs[] = {
        {2, "rt+x", "hello"},
        {3, "rt+a", "cheer"},
        {4, "x", "byd_small_kick"},
    };
    for (const auto& s : kSpecs) {
        if (s.g != gesture_id) continue;
        publishJoyCombo(s.combo, true);
        ros::Duration(COQUETTE_TRIGGER_PULSE_SEC).sleep();
        publishJoyCombo(s.combo, false);
        ROS_INFO("触发动作 %s (%s)", s.label, s.combo);
        return;
    }
}
#else
void Controller::fireJoyMsgAction(int gesture_id) {
    (void)gesture_id;
}
#endif

void Controller::onConfirmedGesture(int gesture_id) {
    if (gesture_id <= GESTURE_0 || isActionBusy()) return;

    if (gesture_id != fired_episode_) {
        fired_episode_ = gesture_id;
        fired_ = false;
    }
    if (fired_) return;

    if (gesture_id == GESTURE_1) {
        if (coquette_player_.start()) {
            fired_ = true;
            last_action_ms_ = getCurrentTimeMs();
        }
        return;
    }

    if (gesture_id >= GESTURE_2 && gesture_id <= GESTURE_4) {
        fireJoyMsgAction(gesture_id);
        fired_ = true;
        last_action_ms_ = getCurrentTimeMs();
    }
}
