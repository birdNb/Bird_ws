#include "JoyGestureActionPlayer.h"

#include <std_msgs/String.h>

namespace {

constexpr float kBusyExtraSec = 0.5f;

}  // namespace

const JoyGestureActionPlayer::ActionSpec* JoyGestureActionPlayer::specFor(int gesture_id) {
    static const ActionSpec kSpecs[] = {
        {2, "rt+x", "hello", true},
        {3, "rt+a", "cheer", true},
    };
    for (const auto& s : kSpecs) {
        if (s.gesture == gesture_id) {
            return &s;
        }
    }
    return nullptr;
}

JoyGestureActionPlayer::JoyGestureActionPlayer(ros::NodeHandle& nh) {
#ifdef HAVE_SIM2REAL_MSG
    joy_pub_ = nh.advertise<sim2real_msg::Joy>(JOY_MSG_TOPIC, 1);
    const ros::Time t_wait_start = ros::Time::now();
    while (joy_pub_.getNumSubscribers() == 0 && ros::ok()
           && (ros::Time::now() - t_wait_start).toSec() < 5.0) {
        ros::Duration(0.05).sleep();
    }
    if (joy_pub_.getNumSubscribers() == 0) {
        ROS_WARN("[joy_action] no /joy_msg subscriber yet; joy actions may be ignored");
    }
#else
    (void)nh;
    ROS_WARN("[joy_action] sim2real_msg missing; joy actions log only");
#endif
    policy_pub_ = nh.advertise<std_msgs::String>(ACTION_CONFIG_TOPIC, 1);
    const ros::Time t_policy = ros::Time::now();
    while (policy_pub_.getNumSubscribers() == 0 && ros::ok()
           && (ros::Time::now() - t_policy).toSec() < 5.0) {
        ros::Duration(0.05).sleep();
    }
    if (policy_pub_.getNumSubscribers() == 0) {
        ROS_WARN(
            "[joy_action] no %s subscriber yet; policy actions may be ignored",
            ACTION_CONFIG_TOPIC);
    }
    ROS_INFO(
        "[joy_action] G2/G3 joy %.0fs pulse+hold+re-pulse; G4 policy %s@%s",
        ACTION_DURATION_SEC,
        KICK_POLICY_NAME,
        ACTION_CONFIG_TOPIC);
}

JoyGestureActionPlayer::~JoyGestureActionPlayer() { abort(true); }

bool JoyGestureActionPlayer::isBusy() const {
    if (running_.load()) {
        return true;
    }
    return getCurrentTimeMs() < busy_until_ms_;
}

void JoyGestureActionPlayer::publishRelease() {
#ifdef HAVE_SIM2REAL_MSG
    sim2real_msg::Joy release;
    for (int i = 0; i < 3 && ros::ok(); ++i) {
        joy_pub_.publish(release);
        ros::Duration(1.0 / JOY_ACTION_PUBLISH_HZ).sleep();
    }
#endif
}

void JoyGestureActionPlayer::publishPolicyName(const std::string& policy_name) {
    if (policy_name.empty() || !ros::ok() || abort_flag_.load()) {
        return;
    }
    std_msgs::String msg;
    msg.data = policy_name;
    ros::Rate rate(JOY_ACTION_PUBLISH_HZ);
    for (int i = 0; i < 3 && ros::ok() && !abort_flag_.load(); ++i) {
        policy_pub_.publish(msg);
        rate.sleep();
    }
}

void JoyGestureActionPlayer::pulseCombo(const std::string& combo, float duration_sec) {
#ifdef HAVE_SIM2REAL_MSG
    if (abort_flag_.load() || !ros::ok()) {
        return;
    }

    auto make_msg = [&](bool pressed) {
        sim2real_msg::Joy msg;
        size_t start = 0;
        while (start < combo.size()) {
            size_t plus = combo.find('+', start);
            std::string token = combo.substr(
                start, plus == std::string::npos ? std::string::npos : plus - start);
            float v = 0.0f;
            if (token == "rt" || token == "lt") {
                v = pressed ? -1.0f : 1.0f;
            } else if (!token.empty()) {
                v = pressed ? 1.0f : 0.0f;
            }
            if (token == "a") msg.a = v;
            else if (token == "b") msg.b = v;
            else if (token == "x") msg.x = v;
            else if (token == "y") msg.y = v;
            else if (token == "rt") msg.rt = v;
            else if (token == "lt") msg.lt = v;
            if (plus == std::string::npos) break;
            start = plus + 1;
        }
        return msg;
    };

    ros::Rate rate(JOY_ACTION_PUBLISH_HZ);
    const ros::Time end =
        ros::Time::now() + ros::Duration(std::max(0.05, static_cast<double>(duration_sec)));
    const sim2real_msg::Joy press = make_msg(true);
    while (ros::ok() && !abort_flag_.load() && ros::Time::now() < end) {
        joy_pub_.publish(press);
        rate.sleep();
    }
    const sim2real_msg::Joy release = make_msg(false);
    for (int i = 0; i < 3 && ros::ok() && !abort_flag_.load(); ++i) {
        joy_pub_.publish(release);
        rate.sleep();
    }
#else
    (void)combo;
    ros::Duration(std::min(duration_sec, 0.05f)).sleep();
#endif
}

void JoyGestureActionPlayer::workerTimed(const std::string& combo, const std::string& label) {
    ROS_INFO("[joy_action] start %s (%s)", label.c_str(), combo.c_str());
    active_combo_ = combo;

    pulseCombo(combo, JOY_ACTION_PULSE_SEC);

    const ros::Time t0 = ros::Time::now();
    while (ros::ok() && !abort_flag_.load()
           && (ros::Time::now() - t0).toSec() < ACTION_DURATION_SEC) {
        ros::Duration(0.05).sleep();
    }

    if (!abort_flag_.load()) {
        ROS_INFO(
            "[joy_action] stop %s (%.0fs, re-pulse %s)",
            label.c_str(),
            ACTION_DURATION_SEC,
            combo.c_str());
        pulseCombo(combo, JOY_ACTION_PULSE_SEC);
    }

    active_combo_.clear();
    publishRelease();
    running_.store(false);
    ROS_INFO("[joy_action] done %s", label.c_str());
}

void JoyGestureActionPlayer::workerPolicy(
    const std::string& policy_name,
    const std::string& label) {
    ROS_INFO(
        "[joy_action] start %s (policy %s -> %s)",
        label.c_str(),
        policy_name.c_str(),
        ACTION_CONFIG_TOPIC);
    active_combo_.clear();

    publishPolicyName(policy_name);

    const ros::Time t0 = ros::Time::now();
    while (ros::ok() && !abort_flag_.load()
           && (ros::Time::now() - t0).toSec() < ACTION_DURATION_SEC) {
        ros::Duration(0.05).sleep();
    }

    if (!abort_flag_.load()) {
        ROS_INFO(
            "[joy_action] done %s (policy %s, controller auto back_to_walk)",
            label.c_str(),
            policy_name.c_str());
    }

    running_.store(false);
}

void JoyGestureActionPlayer::abort(bool fast) {
    const bool was_active = running_.load() || isBusy();
    if (!was_active) {
        return;
    }
    abort_flag_.store(true);

    std::thread t;
    {
        std::lock_guard<std::mutex> lk(mu_);
        t = std::move(worker_);
    }
    if (t.joinable()) {
        t.join();
    }

#ifdef HAVE_SIM2REAL_MSG
    if (!fast && !active_combo_.empty()) {
        pulseCombo(active_combo_, JOY_ACTION_PULSE_SEC);
    }
#endif
    active_combo_.clear();
    publishRelease();

    running_.store(false);
    busy_until_ms_ = 0;
    if (!fast && was_active) {
        ROS_WARN("[joy_action] aborted");
    }
    abort_flag_.store(false);
}

bool JoyGestureActionPlayer::startTimedAction(int gesture_id) {
    const ActionSpec* spec = specFor(gesture_id);
    if (spec == nullptr || !spec->timed_stop) {
        return false;
    }

    const long long now = getCurrentTimeMs();
    if (isBusy() || now - last_fire_ms_ < JOY_ACTION_COOLDOWN_MS) {
        return false;
    }

    abort(false);

    last_fire_ms_ = now;
    busy_until_ms_ = now + static_cast<long long>(
        (ACTION_DURATION_SEC + JOY_ACTION_PULSE_SEC * 2.0f + kBusyExtraSec) * 1000.0f);

    abort_flag_.store(false);
    running_.store(true);
    const std::string combo = spec->combo;
    const std::string label = spec->label;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (worker_.joinable()) {
            worker_.join();
        }
        worker_ = std::thread(&JoyGestureActionPlayer::workerTimed, this, combo, label);
    }
    return true;
}

bool JoyGestureActionPlayer::startPolicyAction(int gesture_id) {
    if (gesture_id != GESTURE_4) {
        return false;
    }

    const long long now = getCurrentTimeMs();
    if (isBusy() || now - last_fire_ms_ < JOY_ACTION_COOLDOWN_MS) {
        return false;
    }

    abort(false);

    last_fire_ms_ = now;
    busy_until_ms_ =
        now + static_cast<long long>((ACTION_DURATION_SEC + kBusyExtraSec) * 1000.0f);

    abort_flag_.store(false);
    running_.store(true);
    const std::string policy = KICK_POLICY_NAME;
    const std::string label = "byd_small_kick";
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (worker_.joinable()) {
            worker_.join();
        }
        worker_ = std::thread(&JoyGestureActionPlayer::workerPolicy, this, policy, label);
    }
    return true;
}

bool JoyGestureActionPlayer::pulseOnce(int gesture_id) {
    const ActionSpec* spec = specFor(gesture_id);
    if (spec == nullptr || spec->timed_stop) {
        return false;
    }

    const long long now = getCurrentTimeMs();
    if (isBusy() || now - last_fire_ms_ < JOY_ACTION_COOLDOWN_MS) {
        return false;
    }

    last_fire_ms_ = now;
    busy_until_ms_ = now + static_cast<long long>((JOY_ACTION_PULSE_SEC + 1.0f) * 1000.0f);

    ROS_INFO("[joy_action] trigger %s (%s) one-shot", spec->label, spec->combo);
    pulseCombo(spec->combo, JOY_ACTION_PULSE_SEC);
    return true;
}
