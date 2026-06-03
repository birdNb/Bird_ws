#include "FaceTracker.h"

FaceTracker::FaceTracker(ros::NodeHandle& nh) {
    std::vector<std::string> paths = {
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        cv::samples::findFile("haarcascades/haarcascade_frontalface_default.xml"),
    };
    bool loaded = false;
    for (const auto& p : paths) {
        if (!p.empty() && face_cascade_.load(p)) {
            loaded = true;
            ROS_INFO("人脸检测器: %s", p.c_str());
            break;
        }
    }
    if (!loaded) {
        ROS_ERROR("人脸 Haar 模型加载失败");
    }
    neck_pub_ = nh.advertise<sensor_msgs::JointState>(ABSOLUTE_TOPIC, 10);
    last_face_ms_ = getCurrentTimeMs();
    pub_thread_ = std::thread(&FaceTracker::publisherLoop, this);
}

FaceTracker::~FaceTracker() { shutdown(); }

void FaceTracker::setEnabled(bool on) { enabled_.store(on); }

void FaceTracker::shutdown() {
    running_.store(false);
    if (pub_thread_.joinable()) pub_thread_.join();
    stopNeck();
}

void FaceTracker::publishNeck(float yaw_rad, float pitch_rad) {
    sensor_msgs::JointState msg;
    msg.name = {HEAD_YAW_JOINT, HEAD_PITCH_JOINT};
    msg.position = {yaw_rad, pitch_rad};
    msg.header.stamp = ros::Time::now();
    neck_pub_.publish(msg);
}

void FaceTracker::publisherLoop() {
    ros::Rate rate(NECK_PUBLISH_RATE_HZ);
    while (running_.load() && ros::ok()) {
        float y = 0.0f, p = 0.0f;
        {
            std::lock_guard<std::mutex> lk(target_mu_);
            y = target_yaw_;
            p = target_pitch_;
        }
        publishNeck(y, p);
        rate.sleep();
    }
}

void FaceTracker::updateTargetFromError(float dx_n, float dy_n, float dt) {
    if (std::abs(dx_n) < DEAD_BAND_X) dx_n = 0.0f;
    if (std::abs(dy_n) < DEAD_BAND_Y) dy_n = 0.0f;

    float dx_ctrl = YAW_DX_SIGN * dx_n;
    float delta_yaw_deg = clampf(-K_YAW_DEG * dx_ctrl, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG);
    float delta_pitch_deg = clampf(K_PITCH_DEG * dy_n, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG);

    std::lock_guard<std::mutex> lk(target_mu_);
    float raw_yaw = ctrl_yaw_ + deg2rad(delta_yaw_deg);
    float raw_pitch = ctrl_pitch_ + deg2rad(delta_pitch_deg);
    ctrl_yaw_ = ctrl_yaw_ * (1.0f - TARGET_EMA_ALPHA) + raw_yaw * TARGET_EMA_ALPHA;
    ctrl_pitch_ = ctrl_pitch_ * (1.0f - TARGET_EMA_ALPHA) + raw_pitch * TARGET_EMA_ALPHA;
    ctrl_yaw_ = clampf(ctrl_yaw_, -deg2rad(YAW_LIMIT_DEG), deg2rad(YAW_LIMIT_DEG));
    ctrl_pitch_ = clampf(ctrl_pitch_, deg2rad(PITCH_UP_DEG), deg2rad(PITCH_DOWN_DEG));
    target_yaw_ = ctrl_yaw_;
    target_pitch_ = ctrl_pitch_;
}

void FaceTracker::trackAndControlNeck(const cv::Mat& frame) {
    if (!enabled_.load() || face_cascade_.empty() || frame.empty()) return;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(gray, faces, 1.1, 4, 0, cv::Size(40, 40));

    long long now_ms = getCurrentTimeMs();
    float dt = 0.033f;

    if (faces.empty()) {
        float lost_sec = (now_ms - last_face_ms_) / 1000.0f;
        if (lost_sec > NO_FACE_RETURN_HOME_SEC) {
            std::lock_guard<std::mutex> lk(target_mu_);
            float step = deg2rad(RETURN_HOME_RATE_DEG_PER_SEC * dt);
            if (std::abs(target_yaw_) > 1e-4f) {
                target_yaw_ -= std::copysign(std::min(step, std::abs(target_yaw_)), target_yaw_);
                ctrl_yaw_ = target_yaw_;
            }
            if (std::abs(target_pitch_) > 1e-4f) {
                target_pitch_ -= std::copysign(std::min(step, std::abs(target_pitch_)), target_pitch_);
                ctrl_pitch_ = target_pitch_;
            }
        }
        return;
    }

    cv::Rect max_face = *std::max_element(
        faces.begin(), faces.end(),
        [](const cv::Rect& a, const cv::Rect& b) { return a.area() < b.area(); });

    int cx = max_face.x + max_face.width / 2;
    int cy = max_face.y + max_face.height / 2;
    float dx_n = (cx - frame.cols / 2.0f) / (frame.cols / 2.0f);
    float dy_n = (cy - frame.rows / 2.0f) / (frame.rows / 2.0f);
    updateTargetFromError(dx_n, dy_n, dt);
    last_face_ms_ = now_ms;

    cv::rectangle(const_cast<cv::Mat&>(frame), max_face, cv::Scalar(0, 255, 0), 2);
    cv::circle(const_cast<cv::Mat&>(frame), cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
}

void FaceTracker::stopNeck() {
    std::lock_guard<std::mutex> lk(target_mu_);
    target_yaw_ = 0.0f;
    target_pitch_ = 0.0f;
    ctrl_yaw_ = 0.0f;
    ctrl_pitch_ = 0.0f;
    publishNeck(0.0f, 0.0f);
}
