#pragma once

#include "Common.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <string>

/** /cmd_vel：distance_hold — angular.z(左右居中) + 手势5时 linear.x(距离) */
struct HandFollowCmd {
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    std::string mode = "idle";

    bool hasMotion() const { return linear_x != 0.0f || angular_z != 0.0f; }
};

class HandTracker {
public:
    explicit HandTracker(ros::NodeHandle& nh);

    /**
     * @param engaged 手掌跟手已激活（入画且未丢失）
     * @param dist_follow 手势5：额外前后距离保持
     */
    HandFollowCmd computeDistanceHold(
        const HandDetectResult& hand,
        bool engaged,
        bool dist_follow) const;

    bool tickPublish(const HandFollowCmd& cmd, bool joy_blocking);
    void stopChassis();
    void drawFollowOverlay(cv::Mat& frame, const HandDetectResult& hand) const;

    void logFollowStatus(
        const HandDetectResult& hand,
        const HandFollowCmd& cmd,
        bool joy_blocking,
        float joy_wait_sec,
        float g5_lost_left_sec) const;

    const std::string& lastMode() const { return last_mode_; }
    float lastLinearX() const { return last_linear_x_; }
    float lastAngularZ() const { return last_angular_z_; }

private:
    static float strongLinearCmd(float err_m);
    static float strongAngularCmd(float dx_norm);
    void publishStopOnce();

    ros::Publisher chassis_pub_;
    bool last_pub_active_ = false;
    mutable long long last_log_ms_ = 0;
    std::string last_mode_ = "idle";
    float last_linear_x_ = 0.0f;
    float last_angular_z_ = 0.0f;
};
