#pragma once

#include "Common.h"
#include <mutex>
#include <sensor_msgs/Joy.h>

class JoyMonitor {
public:
    explicit JoyMonitor(ros::NodeHandle& nh);
    bool allowProgramControl() const;
    bool blocksHandTracking() const { return !allowProgramControl(); }
    long long msSinceLastActive() const;
    bool pollTakeoverEdge();
    bool isActiveNow() const;
    long long idleRemainingMs() const;
    void resetTimer();

private:
    bool axisActive(int idx, float val) const;
    bool axesButtonsActive(const sensor_msgs::Joy& msg) const;
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    ros::Subscriber joy_sub_;
    mutable std::mutex mu_;
    long long last_joy_ms_ = 0;
    bool was_blocking_ = false;
};
