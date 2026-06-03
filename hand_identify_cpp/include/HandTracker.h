#pragma once

#include "Common.h"
#include <geometry_msgs/Twist.h>

class HandTracker {
public:
    explicit HandTracker(ros::NodeHandle& nh);
    void followMaxHand(const cv::Mat& frame, const HandDetectResult& hand);
    void stopChassis();

private:
    ros::Publisher chassis_pub_;
};
