#pragma once

#include <mutex>

#include "Common.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class FsmMonitor {
public:
    explicit FsmMonitor(ros::NodeHandle& nh, bool enabled = true);

    bool enabled() const { return enabled_; }
    int state() const;
    bool isExecDefault() const;
    bool waitForExecDefault(float timeout_sec = FSM_WAIT_TIMEOUT_SEC);
    static const char* stateName(int state);

private:
    void onState(const std_msgs::Int32::ConstPtr& msg);

    bool enabled_;
    mutable std::mutex mu_;
    int state_ = -1;
    ros::Subscriber sub_;
};
