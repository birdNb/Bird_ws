#include "FsmMonitor.h"

#include <ros/ros.h>

FsmMonitor::FsmMonitor(ros::NodeHandle& nh, bool enabled) : enabled_(enabled) {
    if (!enabled_) {
        return;
    }
    sub_ = nh.subscribe(FSM_STATE_TOPIC, 1, &FsmMonitor::onState, this);
}

void FsmMonitor::onState(const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    state_ = msg->data;
}

int FsmMonitor::state() const {
    std::lock_guard<std::mutex> lk(mu_);
    return state_;
}

bool FsmMonitor::isExecDefault() const {
    if (!enabled_) {
        return true;
    }
    std::lock_guard<std::mutex> lk(mu_);
    return state_ == FSM_EXEC_DEFAULT;
}

const char* FsmMonitor::stateName(int state) {
    switch (state) {
        case 0: return "INIT";
        case 1: return "ERROR";
        case 2: return "CANDIDATE_DEFAULT";
        case 3: return "CANDIDATE_CUSTOM";
        case 4: return "CANDIDATE_REMOTE";
        case 5: return "EXEC_DEFAULT";
        case 6: return "EXEC_CUSTOM";
        case 7: return "EXEC_REMOTE";
        case 8: return "PROTECTION_SHUTDOWN";
        case 9: return "CANDIDATE_CALIBRATION";
        case 10: return "EXEC_CALIBRATING";
        case 11: return "EXEC_CALIB_OK";
        case 12: return "EXEC_CALIB_FAILED";
        case 13: return "CANDIDATE_TEACHING";
        case 14: return "EXEC_TEACHING";
        case 15: return "CANDIDATE_DEVELOP";
        case 16: return "EXEC_DEVELOP";
        default: return "UNKNOWN";
    }
}

bool FsmMonitor::waitForExecDefault(float timeout_sec) {
    if (!enabled_) {
        return true;
    }
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    ros::Time last_log = ros::Time(0);
    bool warned_timeout = false;
    while (ros::ok()) {
        if (isExecDefault()) {
            return true;
        }
        const ros::Time now = ros::Time::now();
        if ((now - last_log).toSec() >= 1.0) {
            const int s = state();
            if (s < 0) {
                ROS_WARN("[FSM] 还没收到 %s, 请确认 sim2real_master 已启动", FSM_STATE_TOPIC);
            } else {
                ROS_WARN("[FSM] 当前 %s(%d) != EXEC_DEFAULT(%d)", stateName(s), s, FSM_EXEC_DEFAULT);
            }
            last_log = now;
        }
        if (!warned_timeout && now > deadline) {
            ROS_ERROR("[FSM] 等待 %.0fs 仍未进入 EXEC_DEFAULT, 继续等...", timeout_sec);
            warned_timeout = true;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return false;
}
