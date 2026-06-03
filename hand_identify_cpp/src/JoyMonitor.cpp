#include "JoyMonitor.h"

JoyMonitor::JoyMonitor(ros::NodeHandle& nh) {
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 10, &JoyMonitor::joyCallback, this);
    last_joy_ms_ = 0;
    ROS_INFO("joy gate: %s idle %d ms to resume vision", JOY_TOPIC, JOY_IDLE_MS);
}

bool JoyMonitor::axisActive(int idx, float val) const {
    if (idx == JOY_TRIGGER_AXIS_LT || idx == JOY_TRIGGER_AXIS_RT) {
        return val < (JOY_TRIGGER_REST - JOY_TRIGGER_ACTIVE_MARGIN);
    }
    return std::abs(val) > JOY_ACTIVE_THRESH;
}

bool JoyMonitor::axesButtonsActive(const sensor_msgs::Joy& msg) const {
    for (size_t i = 0; i < msg.axes.size(); ++i) {
        if (axisActive(static_cast<int>(i), static_cast<float>(msg.axes[i]))) return true;
    }
    for (int b : msg.buttons) {
        if (b != 0) return true;
    }
    return false;
}

void JoyMonitor::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    if (!msg || !axesButtonsActive(*msg)) return;
    std::lock_guard<std::mutex> lk(mu_);
    last_joy_ms_ = getCurrentTimeMs();
}

bool JoyMonitor::allowProgramControl() const {
    std::lock_guard<std::mutex> lk(mu_);
    if (last_joy_ms_ <= 0) return true;
    return (getCurrentTimeMs() - last_joy_ms_) >= JOY_IDLE_MS;
}

bool JoyMonitor::isActiveNow() const {
    std::lock_guard<std::mutex> lk(mu_);
    if (last_joy_ms_ <= 0) return false;
    return (getCurrentTimeMs() - last_joy_ms_) < 200;
}

long long JoyMonitor::idleRemainingMs() const {
    std::lock_guard<std::mutex> lk(mu_);
    if (last_joy_ms_ <= 0) return 0;
    long long elapsed = getCurrentTimeMs() - last_joy_ms_;
    return std::max(0LL, static_cast<long long>(JOY_IDLE_MS) - elapsed);
}

void JoyMonitor::resetTimer() {
    std::lock_guard<std::mutex> lk(mu_);
    last_joy_ms_ = getCurrentTimeMs();
}
