#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "Common.h"
#include <ros/ros.h>

#ifdef HAVE_SIM2REAL_MSG
#include <sim2real_msg/Joy.h>
#endif

/** G2/G3: /joy_msg；G4: /action_config policy（对齐 gesture_actions.py） */
class JoyGestureActionPlayer {
public:
    explicit JoyGestureActionPlayer(ros::NodeHandle& nh);
    ~JoyGestureActionPlayer();

    /** hello / cheer：后台 5s 定时停止 */
    bool startTimedAction(int gesture_id);
    /** G4 踢球：发布 policy 名，等待 ACTION_DURATION_SEC */
    bool startPolicyAction(int gesture_id);
    /** 未使用 timed_stop=false 的动作 */
    bool pulseOnce(int gesture_id);
    void abort(bool fast = false, bool publish_neutral = true);
    bool isBusy() const;

private:
    struct ActionSpec {
        int gesture = -1;
        const char* combo = "";
        const char* label = "";
        bool timed_stop = true;
    };

    static const ActionSpec* specFor(int gesture_id);
    void workerTimed(const std::string& combo, const std::string& label);
    void workerPolicy(const std::string& policy_name, const std::string& label);
    void pulseCombo(const std::string& combo, float duration_sec);
    void publishPolicyName(const std::string& policy_name);
    void publishRelease();

    ros::Publisher joy_pub_;
    ros::Publisher policy_pub_;
    std::thread worker_;
    std::mutex mu_;
    std::atomic<bool> abort_flag_{false};
    std::atomic<bool> running_{false};
    std::string active_combo_;
    long long busy_until_ms_ = 0;
    long long last_fire_ms_ = 0;
};
