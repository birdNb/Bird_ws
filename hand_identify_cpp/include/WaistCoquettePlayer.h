#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "Common.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#ifdef HAVE_SIM2REAL_MSG
#include <sim2real_msg/Joy.h>
#endif

/** 手势1：waist_yaw ±45° 来回 2 次 + cheer(rt+a)，仅控腰不控头 */
class WaistCoquettePlayer {
public:
    explicit WaistCoquettePlayer(ros::NodeHandle& nh);
    ~WaistCoquettePlayer();

    bool start();
    void abort(bool fast = false);
    bool isBusy() const;
    float busyRemainingSec() const;
    static float actionTotalSec();

private:
    void workerLoop();
    void pulseJoyCombo(const std::string& combo, float duration_sec);
    float runUniformSway(const std::vector<float>& waypoints_rad);

    static std::vector<float> swayWaypointsRad(float amplitude_rad);
    static float swayMotionSec(const std::vector<float>& waypoints_rad);
    static float interpWaypoints(
        const std::vector<float>& pts,
        const std::vector<float>& seg_starts,
        float elapsed);

    ros::Publisher waist_pub_;
#ifdef HAVE_SIM2REAL_MSG
    ros::Publisher joy_pub_;
#endif
    std::thread worker_;
    mutable std::mutex mu_;
    std::atomic<bool> abort_flag_{false};
    std::atomic<bool> running_{false};
    long long busy_until_ms_ = 0;
    long long last_fire_ms_ = 0;
};
