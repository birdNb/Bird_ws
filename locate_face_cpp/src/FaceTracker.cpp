#include "FaceTracker.h"

#include <climits>
#include <cmath>
#include <cstdio>
#include <unistd.h>

namespace {

cv::Rect clampRect(const cv::Rect& r, const cv::Size& limit) {
    int x = std::max(0, r.x);
    int y = std::max(0, r.y);
    int w = std::min(r.width, limit.width - x);
    int h = std::min(r.height, limit.height - y);
    if (w <= 0 || h <= 0) {
        return cv::Rect();
    }
    return cv::Rect(x, y, w, h);
}

}  // namespace

bool FaceTracker::probeYuNet(cv::Ptr<cv::FaceDetectorYN> detector) {
    if (detector.empty()) {
        return false;
    }
    try {
        // Jetson 上用小图 probe 会触发 DNN shape 异常，用接近实际的尺寸
        cv::Mat img(240, 320, CV_8UC3, cv::Scalar(32, 32, 32));
        detector->setInputSize(img.size());
        cv::Mat faces;
        detector->detect(img, faces);
        return true;
    } catch (const cv::Exception& e) {
        ROS_WARN("YuNet probe failed: %s", e.what());
        return false;
    }
}

bool FaceTracker::initFaceBackend() {
    const std::string model_path = projectRoot() + "/model/face_detection_yunet_2023mar.onnx";
    const std::string yunet_py = projectRoot() + "/scripts/face_yunet_worker.py";
    const std::string mp_py = projectRoot() + "/scripts/face_mediapipe_worker.py";

    // Jetson 上 C++ FaceDetectorYN 易崩溃，优先 Python YuNet 子进程
    if (access(yunet_py.c_str(), R_OK) == 0 && access(model_path.c_str(), R_OK) == 0) {
        if (mp_bridge_.start(yunet_py)) {
            backend_ = FaceDetectBackend::YuNetPython;
            ROS_INFO("[face] backend YuNet (Python worker, Jetson 推荐)");
            return true;
        }
    }

    try {
        detector_ = cv::FaceDetectorYN::create(
            model_path,
            "",
            cv::Size(320, 320),
            FACE_DETECT_SCORE_THRESH,
            FACE_NMS_THRESH,
            FACE_DETECT_TOP_K);
        if (!detector_.empty() && probeYuNet(detector_)) {
            backend_ = FaceDetectBackend::YuNet;
            ROS_INFO("[face] backend YuNet C++ model=%s", model_path.c_str());
            return true;
        }
    } catch (const cv::Exception& e) {
        ROS_WARN("YuNet C++ create failed: %s", e.what());
    }
    detector_.release();

    if (access(mp_py.c_str(), R_OK) == 0 && mp_bridge_.start(mp_py)) {
        backend_ = FaceDetectBackend::MediaPipe;
        ROS_INFO("[face] backend MediaPipe worker");
        return true;
    }

    backend_ = FaceDetectBackend::None;
    ROS_ERROR("[face] 人脸后端初始化失败，请运行 ./build.sh");
    return false;
}

FaceTracker::FaceTracker(ros::NodeHandle& nh, FsmMonitor* fsm) : fsm_(fsm) {
    initFaceBackend();
    neck_pub_ = nh.advertise<sensor_msgs::JointState>(ABSOLUTE_TOPIC, 10);
    last_face_ms_ = getCurrentTimeMs();
}

FaceTracker::~FaceTracker() { shutdown(); }

void FaceTracker::startPublisher() {
    if (pub_thread_.joinable()) {
        return;
    }
    running_.store(true);
    pub_thread_ = std::thread(&FaceTracker::publisherLoop, this);
}

void FaceTracker::stepHoming(float dt) {
    const float step = deg2rad(RETURN_HOME_RATE_DEG_PER_SEC * dt);
    std::lock_guard<std::mutex> lk(target_mu_);
    if (std::abs(target_yaw_) > 1e-4f) {
        target_yaw_ -= std::copysign(std::min(step, std::abs(target_yaw_)), target_yaw_);
        ctrl_yaw_ = target_yaw_;
    } else {
        target_yaw_ = 0.0f;
        ctrl_yaw_ = 0.0f;
    }
    if (std::abs(target_pitch_) > 1e-4f) {
        target_pitch_ -= std::copysign(std::min(step, std::abs(target_pitch_)), target_pitch_);
        ctrl_pitch_ = target_pitch_;
    } else {
        target_pitch_ = 0.0f;
        ctrl_pitch_ = 0.0f;
    }
}

bool FaceTracker::isAtCenter() {
    std::lock_guard<std::mutex> lk(target_mu_);
    return std::abs(target_yaw_) <= 1e-4f && std::abs(target_pitch_) <= 1e-4f;
}

void FaceTracker::writeNeckStateFile() {
    float y = 0.0f;
    float p = 0.0f;
    {
        std::lock_guard<std::mutex> lk(target_mu_);
        y = target_yaw_;
        p = target_pitch_;
    }
    FILE* f = std::fopen(NECK_STATE_FILE, "w");
    if (f == nullptr) {
        return;
    }
    std::fprintf(f, "%.3f %.3f\n", rad2deg(y), rad2deg(p));
    std::fclose(f);
}

void FaceTracker::returnHomeBlocking() {
    float y = 0.0f;
    float p = 0.0f;
    {
        std::lock_guard<std::mutex> lk(target_mu_);
        y = target_yaw_;
        p = target_pitch_;
    }
    if (std::abs(y) <= 1e-4f && std::abs(p) <= 1e-4f) {
        return;
    }

    ROS_INFO(
        "[homing] 关闭跟踪 -> 平滑回中 (yaw=%+.1f pitch=%+.1f)",
        rad2deg(y),
        rad2deg(p));

    homing_active_.store(true);
    const float dt = 1.0f / static_cast<float>(NECK_PUBLISH_RATE_HZ);
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(static_cast<int>(8000.0f));
    while (running_.load() && std::chrono::steady_clock::now() < deadline) {
        stepHoming(dt);
        float y = 0.0f;
        float p = 0.0f;
        {
            std::lock_guard<std::mutex> lk(target_mu_);
            y = target_yaw_;
            p = target_pitch_;
        }
        publishNeck(y, p);
        writeNeckStateFile();
        if (isAtCenter()) {
            break;
        }
        ros::Duration(dt).sleep();
    }

    {
        std::lock_guard<std::mutex> lk(target_mu_);
        target_yaw_ = 0.0f;
        target_pitch_ = 0.0f;
        ctrl_yaw_ = 0.0f;
        ctrl_pitch_ = 0.0f;
    }
    for (int i = 0; i < NECK_PUBLISH_RATE_HZ / 2; ++i) {
        publishNeck(0.0f, 0.0f);
        ros::Duration(dt).sleep();
    }
    writeNeckStateFile();
    ROS_INFO("[homing] 回中完成");
    homing_active_.store(false);
}

void FaceTracker::shutdown() {
    if (running_.load()) {
        returnHomeBlocking();
    }
    running_.store(false);
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }
    mp_bridge_.stop();
    for (int i = 0; i < 10; ++i) {
        publishNeck(0.0f, 0.0f);
        ros::Duration(0.02).sleep();
    }
}

FaceTelemetry FaceTracker::getTelemetry() const {
    std::lock_guard<std::mutex> lk(telem_mu_);
    return telem_;
}

cv::Rect FaceTracker::expandRoi(const cv::Rect& box, float pad_ratio, const cv::Size& frame_size) {
    if (box.area() <= 0) {
        return cv::Rect();
    }
    const float pad_x = box.width * pad_ratio;
    const float pad_y = box.height * pad_ratio;
    cv::Rect roi(
        static_cast<int>(box.x - pad_x),
        static_cast<int>(box.y - pad_y),
        static_cast<int>(box.width + 2.0f * pad_x),
        static_cast<int>(box.height + 2.0f * pad_y));
    return clampRect(roi, frame_size);
}

cv::Rect FaceTracker::mapProcRectToFrame(
    const cv::Rect& proc_rect,
    int proc_w,
    int proc_h,
    int frame_w,
    int frame_h) {
    if (proc_rect.area() <= 0 || proc_w <= 0 || proc_h <= 0) {
        return cv::Rect();
    }
    const float sx = static_cast<float>(frame_w) / static_cast<float>(proc_w);
    const float sy = static_cast<float>(frame_h) / static_cast<float>(proc_h);
    return cv::Rect(
        static_cast<int>(proc_rect.x * sx),
        static_cast<int>(proc_rect.y * sy),
        static_cast<int>(proc_rect.width * sx),
        static_cast<int>(proc_rect.height * sy));
}

bool FaceTracker::runYuNet(
    const cv::Mat& proc_bgr,
    const cv::Rect& roi,
    std::vector<FaceDet>& out) const {
    out.clear();
    if (detector_.empty() || proc_bgr.empty()) {
        return false;
    }

    cv::Mat input = proc_bgr;
    cv::Rect use_roi = clampRect(roi, proc_bgr.size());
    int off_x = 0;
    int off_y = 0;
    if (use_roi.area() > 0
        && (use_roi.x > 0 || use_roi.y > 0 || use_roi.width < proc_bgr.cols
            || use_roi.height < proc_bgr.rows)) {
        input = proc_bgr(use_roi);
        off_x = use_roi.x;
        off_y = use_roi.y;
    }

    detector_->setInputSize(input.size());
    cv::Mat faces;
    detector_->detect(input, faces);
    if (faces.empty() || faces.rows <= 0) {
        return false;
    }

    for (int i = 0; i < faces.rows; ++i) {
        const float score = faces.at<float>(i, 14);
        if (score < FACE_DETECT_SCORE_THRESH) {
            continue;
        }
        const int x = static_cast<int>(faces.at<float>(i, 0));
        const int y = static_cast<int>(faces.at<float>(i, 1));
        const int w = static_cast<int>(faces.at<float>(i, 2));
        const int h = static_cast<int>(faces.at<float>(i, 3));
        if (w < 20 || h < 20) {
            continue;
        }
        FaceDet det;
        det.bbox = cv::Rect(x + off_x, y + off_y, w, h);
        det.score = score;
        out.push_back(det);
    }
    return !out.empty();
}

cv::Rect FaceTracker::pickBestDet(const std::vector<FaceDet>& dets) {
    if (dets.empty()) {
        return cv::Rect();
    }
    const FaceDet* best = &dets[0];
    for (const auto& d : dets) {
        if (d.score > best->score) {
            best = &d;
        }
    }
    return best->bbox;
}

bool FaceTracker::detectWithYuNet(const cv::Mat& frame, float& dx_n, float& dy_n) {
    int proc_w = 0;
    int proc_h = 0;
    computeProcSize(frame.cols, frame.rows, FACE_PROC_MAX_W, proc_w, proc_h);

    cv::Mat proc_bgr;
    if (proc_w != frame.cols || proc_h != frame.rows) {
        cv::resize(frame, proc_bgr, cv::Size(proc_w, proc_h), 0, 0, cv::INTER_AREA);
    } else {
        proc_bgr = frame;
    }

    std::vector<FaceDet> dets;
    runYuNet(proc_bgr, cv::Rect(), dets);

    if (dets.empty() && has_last_bbox_) {
        const cv::Rect roi = expandRoi(last_face_bbox_, FACE_ROI_PAD_RATIO, frame.size());
        cv::Rect proc_roi;
        const float sx = static_cast<float>(proc_w) / static_cast<float>(frame.cols);
        const float sy = static_cast<float>(proc_h) / static_cast<float>(frame.rows);
        proc_roi = cv::Rect(
            static_cast<int>(roi.x * sx),
            static_cast<int>(roi.y * sy),
            static_cast<int>(roi.width * sx),
            static_cast<int>(roi.height * sy));
        runYuNet(proc_bgr, proc_roi, dets);
    }

    const cv::Rect face_proc = pickBestDet(dets);
    if (face_proc.area() <= 0) {
        return false;
    }

    const cv::Rect face_disp = mapProcRectToFrame(
        face_proc, proc_w, proc_h, frame.cols, frame.rows);
    last_face_bbox_ = face_disp;
    has_last_bbox_ = true;

    const float face_cx = face_disp.x + face_disp.width / 2.0f;
    const float face_cy = face_disp.y + face_disp.height / 2.0f;
    const float cx_img = frame.cols / 2.0f;
    const float cy_img = frame.rows / 2.0f;
    dx_n = (face_cx - cx_img) / (frame.cols / 2.0f);
    dy_n = (face_cy - cy_img) / (frame.rows / 2.0f);
    return true;
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
    bool warned_fsm = false;
    int publish_tick = 0;
    while (running_.load()) {
        if (fsm_ != nullptr && fsm_->enabled() && !homing_active_.load()) {
            const int st = fsm_->state();
            if (st >= 0 && st != FSM_EXEC_DEFAULT) {
                if (!warned_fsm) {
                    ROS_WARN_THROTTLE(
                        2.0,
                        "[ctrl] FSM %s(%d), 暂停脖子下发(需 EXEC_DEFAULT)",
                        FsmMonitor::stateName(st),
                        st);
                    warned_fsm = true;
                }
                rate.sleep();
                continue;
            }
        }
        warned_fsm = false;
        float y = 0.0f;
        float p = 0.0f;
        {
            std::lock_guard<std::mutex> lk(target_mu_);
            y = target_yaw_;
            p = target_pitch_;
        }
        publishNeck(y, p);
        if (homing_active_.load() || (publish_tick++ % 5) == 0) {
            writeNeckStateFile();
        }
        rate.sleep();
    }
}

void FaceTracker::updateTargetFromError(float dx_n, float dy_n) {
    if (std::abs(dx_n) < DEAD_BAND_X) {
        dx_n = 0.0f;
    }
    if (std::abs(dy_n) < DEAD_BAND_Y) {
        dy_n = 0.0f;
    }

    float delta_yaw_deg = -K_YAW_DEG * dx_n;
    float delta_pitch_deg = K_PITCH_DEG * dy_n;
    delta_yaw_deg = clampf(delta_yaw_deg, -MAX_STEP_YAW_DEG, MAX_STEP_YAW_DEG);
    delta_pitch_deg = clampf(delta_pitch_deg, -MAX_STEP_PITCH_DEG, MAX_STEP_PITCH_DEG);

    const float yaw_lim_rad = deg2rad(YAW_LIMIT_DEG);
    const float pitch_up_rad = deg2rad(PITCH_UP_DEG);
    const float pitch_dn_rad = deg2rad(PITCH_DOWN_DEG);

    std::lock_guard<std::mutex> lk(target_mu_);
    const float raw_yaw_rad = ctrl_yaw_ + deg2rad(delta_yaw_deg);
    const float raw_pitch_rad = ctrl_pitch_ + deg2rad(delta_pitch_deg);
    const float a = TARGET_EMA_ALPHA;
    ctrl_yaw_ = ctrl_yaw_ * (1.0f - a) + raw_yaw_rad * a;
    ctrl_pitch_ = ctrl_pitch_ * (1.0f - a) + raw_pitch_rad * a;
    ctrl_yaw_ = clampf(ctrl_yaw_, -yaw_lim_rad, yaw_lim_rad);
    ctrl_pitch_ = clampf(ctrl_pitch_, pitch_up_rad, pitch_dn_rad);
    target_yaw_ = ctrl_yaw_;
    target_pitch_ = ctrl_pitch_;
}

void FaceTracker::applyFaceTracking(float dx_n, float dy_n, long long now_ms) {
    updateTargetFromError(dx_n, dy_n);
    last_face_ms_ = now_ms;

    float y = 0.0f;
    float p = 0.0f;
    {
        std::lock_guard<std::mutex> lk(target_mu_);
        y = target_yaw_;
        p = target_pitch_;
    }
    {
        std::lock_guard<std::mutex> lk(telem_mu_);
        telem_.has_face = true;
        telem_.dx_norm = dx_n;
        telem_.dy_norm = dy_n;
        telem_.yaw_deg = rad2deg(y);
        telem_.pitch_deg = rad2deg(p);
        telem_.lost_sec = 0.0f;
        telem_.state = FaceTrackState::Tracking;
    }
}

void FaceTracker::applyNoFace(long long now_ms, float dt) {
    const float lost_sec = (now_ms - last_face_ms_) / 1000.0f;
    FaceTrackState st = FaceTrackState::NoFace;
    if (lost_sec > NO_FACE_RETURN_HOME_SEC) {
        st = FaceTrackState::Homing;
        stepHoming(dt);
    }

    float y = 0.0f;
    float p = 0.0f;
    {
        std::lock_guard<std::mutex> lk(target_mu_);
        y = target_yaw_;
        p = target_pitch_;
    }
    {
        std::lock_guard<std::mutex> lk(telem_mu_);
        telem_.has_face = false;
        telem_.dx_norm = 0.0f;
        telem_.dy_norm = 0.0f;
        telem_.yaw_deg = rad2deg(y);
        telem_.pitch_deg = rad2deg(p);
        telem_.lost_sec = lost_sec;
        telem_.state = st;
    }
}

void FaceTracker::trackFrame(const cv::Mat& frame) {
    if (backend_ == FaceDetectBackend::None || frame.empty()) {
        return;
    }

    const long long now_ms = getCurrentTimeMs();
    float dt = 0.033f;
    if (last_track_ms_ > 0) {
        dt = std::max(0.02f, std::min(0.2f, (now_ms - last_track_ms_) / 1000.0f));
    }
    last_track_ms_ = now_ms;

    float dx_n = 0.0f;
    float dy_n = 0.0f;
    bool found = false;

    if (backend_ == FaceDetectBackend::MediaPipe || backend_ == FaceDetectBackend::YuNetPython) {
        float face_cx = 0.0f;
        float face_cy = 0.0f;
        found = mp_bridge_.detect(frame, dx_n, dy_n, face_cx, face_cy);
    } else {
        found = detectWithYuNet(frame, dx_n, dy_n);
    }

    if (found) {
        applyFaceTracking(dx_n, dy_n, now_ms);
    } else {
        applyNoFace(now_ms, dt);
    }
}
