#include "HandTracker.h"

HandTracker::HandTracker(ros::NodeHandle& nh) {
    chassis_pub_ = nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10);
}

void HandTracker::followMaxHand(const cv::Mat& frame, const HandDetectResult& hand) {
    if (!hand.has_hand || hand.hand_rect.empty()) {
        stopChassis();
        return;
    }

    float dx_norm = hand.dx_norm;
    float angular_z = 0.0f;
    if (std::abs(dx_norm) > LATERAL_DEADBAND_NORM) {
        angular_z = (dx_norm > 0.0f) ? -ANGULAR_Z_MAG : ANGULAR_Z_MAG;
    }

    float dist_err = hand.distance_m - TARGET_DISTANCE_M;
    float linear_x = 0.0f;
    if (std::abs(dist_err) > DIST_DEADBAND_M) {
        linear_x = (dist_err > 0.0f) ? LINEAR_X_MAG : -LINEAR_X_MAG;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = clampf(linear_x, -LINEAR_X_MAG, LINEAR_X_MAG);
    msg.angular.z = clampf(angular_z, -ANGULAR_Z_MAG, ANGULAR_Z_MAG);
    chassis_pub_.publish(msg);

    cv::rectangle(const_cast<cv::Mat&>(frame), hand.hand_rect, cv::Scalar(255, 0, 0), 2);
}

void HandTracker::stopChassis() {
    geometry_msgs::Twist msg;
    chassis_pub_.publish(msg);
}
