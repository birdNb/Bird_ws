#include "HandTracker.h"

#include <cstdio>
#include <cmath>

HandTracker::HandTracker(ros::NodeHandle& nh) {
    chassis_pub_ = nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10);
    ROS_INFO(
        "[htrack] distance_hold: dx deadband=%.0f%% angular.z=±%.1f | "
        "G5 linear.x=±%.1f target Z=%.2fm deadband=%.2fm",
        LATERAL_DEADBAND_NORM * 100.0f,
        ANGULAR_Z_MAG,
        LINEAR_X_MAG,
        TARGET_DISTANCE_M,
        DIST_DEADBAND_M);
}

float HandTracker::strongLinearCmd(float err_m) {
    if (std::abs(err_m) <= DIST_DEADBAND_M) {
        return 0.0f;
    }
    return (err_m > 0.0f) ? LINEAR_X_MAG : -LINEAR_X_MAG;
}

float HandTracker::strongAngularCmd(float dx_norm) {
    if (std::abs(dx_norm) <= LATERAL_DEADBAND_NORM) {
        return 0.0f;
    }
    return (dx_norm > 0.0f) ? -ANGULAR_Z_MAG : ANGULAR_Z_MAG;
}

HandFollowCmd HandTracker::computeDistanceHold(
    const HandDetectResult& hand,
    bool engaged,
    bool dist_follow) const {
    HandFollowCmd out;
    if (!engaged || !hand.has_hand || !hand.in_range) {
        return out;
    }

    out.angular_z = strongAngularCmd(hand.dx_norm);
    out.mode = "yaw";

    if (dist_follow && hand.gesture_id == GESTURE_5) {
        const float err = hand.distance_m - TARGET_DISTANCE_M;
        out.linear_x = strongLinearCmd(err);
        out.mode = "yaw+distance";
    }
    return out;
}

bool HandTracker::tickPublish(const HandFollowCmd& cmd, bool joy_blocking) {
    if (joy_blocking) {
        last_pub_active_ = false;
        last_mode_ = "joy";
        last_linear_x_ = 0.0f;
        last_angular_z_ = 0.0f;
        return false;
    }

    if (cmd.hasMotion()) {
        geometry_msgs::Twist msg;
        msg.linear.x = cmd.linear_x;
        msg.angular.z = cmd.angular_z;
        chassis_pub_.publish(msg);
        last_pub_active_ = true;
        last_mode_ = cmd.mode;
        last_linear_x_ = cmd.linear_x;
        last_angular_z_ = cmd.angular_z;
        return true;
    }

    if (last_pub_active_) {
        publishStopOnce();
        last_pub_active_ = false;
    }
    last_mode_ = "idle";
    last_linear_x_ = 0.0f;
    last_angular_z_ = 0.0f;
    return false;
}

void HandTracker::publishStopOnce() {
    chassis_pub_.publish(geometry_msgs::Twist());
}

void HandTracker::stopChassis() {
    if (last_pub_active_) {
        publishStopOnce();
        last_pub_active_ = false;
    }
    last_mode_ = "idle";
    last_linear_x_ = 0.0f;
    last_angular_z_ = 0.0f;
}

void HandTracker::logFollowStatus(
    const HandDetectResult& hand,
    const HandFollowCmd& cmd,
    bool joy_blocking,
    float joy_wait_sec,
    float g5_lost_left_sec) const {
    const long long now = getCurrentTimeMs();
    const long long interval_ms = static_cast<long long>(1000.0f / HAND_TRACK_LOG_HZ);
    if (last_log_ms_ > 0 && (now - last_log_ms_) < interval_ms) {
        return;
    }
    last_log_ms_ = now;

    char buf[160];
    std::snprintf(
        buf,
        sizeof(buf),
        "[htrack] g=%d dx=%+.2f z=%.2fm cmd_x=%+.2f cmd_z=%+.2f mode=%s",
        hand.gesture_id,
        hand.dx_norm,
        hand.distance_m,
        cmd.linear_x,
        cmd.angular_z,
        cmd.mode.c_str());
    std::string line(buf);
    if (g5_lost_left_sec > 0.0f) {
        line += " g5_back=" + std::to_string(static_cast<int>(g5_lost_left_sec)) + "s";
    }
    if (joy_blocking) {
        line += " joy_wait=" + std::to_string(static_cast<int>(joy_wait_sec)) + "s";
    }
    std::fprintf(stderr, "\r%-100s", line.c_str());
    std::fflush(stderr);
}

void HandTracker::drawFollowOverlay(cv::Mat& frame, const HandDetectResult& hand) const {
    if (hand.hand_rect.area() > 0) {
        cv::rectangle(frame, hand.hand_rect, cv::Scalar(255, 0, 0), 2);
    }
    if (!hand.has_hand) {
        return;
    }
    char label[80];
    std::snprintf(
        label,
        sizeof(label),
        "G:%d dx:%+.2f Z:%.2fm",
        hand.gesture_id,
        hand.dx_norm,
        hand.distance_m);
    cv::putText(
        frame,
        label,
        cv::Point(10, 40),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA);
}
