#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "Common.h"
#include <sensor_msgs/JointState.h>

class FaceTracker {
public:
    explicit FaceTracker(ros::NodeHandle& nh);
    ~FaceTracker();
    void setEnabled(bool on);
    bool isEnabled() const { return enabled_.load(); }
    void trackAndControlNeck(const cv::Mat& frame);
    void stopNeck();
    void shutdown();

private:
    void publisherLoop();
    void publishNeck(float yaw_rad, float pitch_rad);
    void updateTargetFromError(float dx_n, float dy_n, float dt);

    cv::CascadeClassifier face_cascade_;
    ros::Publisher neck_pub_;
    std::thread pub_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> enabled_{true};
    std::mutex target_mu_;
    float target_yaw_ = 0.0f;
    float target_pitch_ = 0.0f;
    float ctrl_yaw_ = 0.0f;
    float ctrl_pitch_ = 0.0f;
    long long last_face_ms_ = 0;
};
