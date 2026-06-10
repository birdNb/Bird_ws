#include "WaistCoquettePlayer.h"

#include "JoyMsgUtil.h"

#include <cmath>

namespace {

constexpr float kCoquetteBusyExtraSec = 1.5f;
constexpr float kCoquetteCooldownExtraSec = 0.5f;

}  // namespace

std::vector<float> WaistCoquettePlayer::swayWaypointsRad(float amplitude_rad) {
    std::vector<float> pts = {0.0f};
    for (int i = 0; i < COQUETTE_SWAY_CYCLES * 2; ++i) {
        pts.push_back((i % 2 == 0) ? amplitude_rad : -amplitude_rad);
    }
    if (std::abs(pts.back()) > 1e-9f) {
        pts.push_back(0.0f);
    }
    return pts;
}

float WaistCoquettePlayer::swayMotionSec(const std::vector<float>& waypoints_rad) {
    const float speed = deg2rad(std::max(COQUETTE_SWAY_VEL_DEG_PER_SEC, 1e-3f));
    float total = 0.0f;
    for (size_t i = 1; i < waypoints_rad.size(); ++i) {
        total += std::abs(waypoints_rad[i] - waypoints_rad[i - 1]) / speed;
    }
    return total;
}

float WaistCoquettePlayer::actionTotalSec() {
    const auto wp = swayWaypointsRad(deg2rad(COQUETTE_SWAY_AMPLITUDE_DEG));
    const float sway_sec = swayMotionSec(wp);
    return COQUETTE_TRIGGER_PULSE_SEC + sway_sec + COQUETTE_CHEER_DURATION_SEC
           + COQUETTE_ARM_RESET_SEC + COQUETTE_TRIGGER_PULSE_SEC;
}

WaistCoquettePlayer::WaistCoquettePlayer(ros::NodeHandle& nh) {
    waist_pub_ = nh.advertise<sensor_msgs::JointState>(ABSOLUTE_TOPIC, 10);
#ifdef HAVE_SIM2REAL_MSG
    joy_pub_ = nh.advertise<sim2real_msg::Joy>(JOY_MSG_TOPIC, 1);
#endif
    ROS_INFO(
        "[coquette] ready amp=+/-%.0f deg x%d cycles total~%.1fs",
        COQUETTE_SWAY_AMPLITUDE_DEG,
        COQUETTE_SWAY_CYCLES,
        actionTotalSec());
}

WaistCoquettePlayer::~WaistCoquettePlayer() { abort(true); }

bool WaistCoquettePlayer::isBusy() const {
    if (running_.load()) return true;
    return getCurrentTimeMs() < busy_until_ms_;
}

float WaistCoquettePlayer::busyRemainingSec() const {
    long long rem = busy_until_ms_ - getCurrentTimeMs();
    return rem > 0 ? rem / 1000.0f : 0.0f;
}

bool WaistCoquettePlayer::start() {
    std::lock_guard<std::mutex> lk(mu_);
    if (worker_.joinable()) {
        worker_.join();
    }
    if (running_.load()) {
        ROS_WARN("[coquette] previous sway still running, skip");
        return false;
    }
    const long long now = getCurrentTimeMs();
    const long long cooldown_ms = static_cast<long long>(
        (actionTotalSec() + kCoquetteBusyExtraSec + kCoquetteCooldownExtraSec) * 1000.0f);
    if (now - last_fire_ms_ < cooldown_ms) {
        return false;
    }
    last_fire_ms_ = now;
    busy_until_ms_ = now + static_cast<long long>(
        (actionTotalSec() + kCoquetteBusyExtraSec) * 1000.0f);
    abort_flag_.store(false);
    running_.store(true);
    worker_ = std::thread(&WaistCoquettePlayer::workerLoop, this);
    return true;
}

void WaistCoquettePlayer::abort(bool fast) {
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
    running_.store(false);
    busy_until_ms_ = 0;
    if (!fast) {
        sensor_msgs::JointState center;
        center.name = {WAIST_YAW_JOINT};
        center.position = {0.0};
        center.header.stamp = ros::Time::now();
        waist_pub_.publish(center);
        ROS_WARN("[coquette] sway aborted");
    }
}

float WaistCoquettePlayer::interpWaypoints(
    const std::vector<float>& pts,
    const std::vector<float>& seg_starts,
    float elapsed) {
    if (elapsed >= seg_starts.back()) return pts.back();
    for (size_t i = 1; i < seg_starts.size(); ++i) {
        if (elapsed <= seg_starts[i]) {
            const float t0 = seg_starts[i - 1];
            const float t1 = seg_starts[i];
            const float alpha = (elapsed - t0) / std::max(t1 - t0, 1e-9f);
            return pts[i - 1] + (pts[i] - pts[i - 1]) * alpha;
        }
    }
    return pts.back();
}

#ifdef HAVE_SIM2REAL_MSG
void WaistCoquettePlayer::pulseJoyCombo(const std::string& combo, float duration_sec) {
    if (abort_flag_.load() || !ros::ok()) return;

    ros::Rate rate(COQUETTE_JOY_PUBLISH_HZ);
    const ros::Time end = ros::Time::now() + ros::Duration(std::max(0.05, static_cast<double>(duration_sec)));
    const sim2real_msg::Joy press = makeJoyCombo(combo, true);
    while (ros::ok() && !abort_flag_.load() && ros::Time::now() < end) {
        joy_pub_.publish(press);
        rate.sleep();
    }
    const sim2real_msg::Joy release = makeJoyCombo(combo, false);
    for (int i = 0; i < 3 && ros::ok() && !abort_flag_.load(); ++i) {
        joy_pub_.publish(release);
        rate.sleep();
    }
}
#else
void WaistCoquettePlayer::pulseJoyCombo(const std::string& combo, float duration_sec) {
    (void)combo;
    ros::Duration(std::min(duration_sec, 0.1f)).sleep();
}
#endif

float WaistCoquettePlayer::runUniformSway(const std::vector<float>& waypoints_rad) {
    if (waypoints_rad.size() < 2) {
        return waypoints_rad.empty() ? 0.0f : waypoints_rad.front();
    }

    const float speed = deg2rad(std::max(COQUETTE_SWAY_VEL_DEG_PER_SEC, 1e-3f));
    std::vector<float> seg_starts = {0.0f};
    for (size_t i = 1; i < waypoints_rad.size(); ++i) {
        const float dt = std::abs(waypoints_rad[i] - waypoints_rad[i - 1]) / speed;
        seg_starts.push_back(seg_starts.back() + dt);
    }
    const float total = seg_starts.back();

    sensor_msgs::JointState msg;
    msg.name = {WAIST_YAW_JOINT};
    msg.velocity.clear();
    msg.effort.clear();

    ros::Rate rate(COQUETTE_WAIST_PUBLISH_HZ);
    const ros::Time t0 = ros::Time::now();
    float pos = waypoints_rad.front();

    while (ros::ok() && !abort_flag_.load()) {
        const float elapsed = static_cast<float>((ros::Time::now() - t0).toSec());
        pos = interpWaypoints(waypoints_rad, seg_starts, elapsed);
        msg.header.stamp = ros::Time::now();
        msg.position = {pos};
        waist_pub_.publish(msg);
        if (elapsed >= total) break;
        rate.sleep();
    }
    return pos;
}

void WaistCoquettePlayer::workerLoop() {
    ROS_INFO(">>> coquette sway [EXEC]");
    const auto waypoints = swayWaypointsRad(deg2rad(COQUETTE_SWAY_AMPLITUDE_DEG));

    pulseJoyCombo("rt+a", COQUETTE_TRIGGER_PULSE_SEC);

    const ros::Time sway_t0 = ros::Time::now();
    runUniformSway(waypoints);

    if (!abort_flag_.load()) {
        const float cheer_remain = std::max(
            0.0f,
            COQUETTE_CHEER_DURATION_SEC - static_cast<float>((ros::Time::now() - sway_t0).toSec()));
        ros::Time end = ros::Time::now() + ros::Duration(cheer_remain);
        while (ros::ok() && !abort_flag_.load() && ros::Time::now() < end) {
            ros::Duration(0.05).sleep();
        }
    }

    if (!abort_flag_.load()) {
        pulseJoyCombo("rt+a", COQUETTE_TRIGGER_PULSE_SEC);
        ros::Duration(COQUETTE_ARM_RESET_SEC).sleep();
        sensor_msgs::JointState center;
        center.name = {WAIST_YAW_JOINT};
        center.position = {0.0};
        center.header.stamp = ros::Time::now();
        waist_pub_.publish(center);
        ROS_INFO(">>> coquette sway done");
    }

    running_.store(false);
}
