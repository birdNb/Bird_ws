#include "FaceTracker.h"

#include <climits>

namespace {

cv::Rect clampRect(const cv::Rect& r, const cv::Size& limit) {
    int x = std::max(0, r.x);
    int y = std::max(0, r.y);
    int w = std::min(r.width, limit.width - x);
    int h = std::min(r.height, limit.height - y);
    if (w <= 0 || h <= 0) return cv::Rect();
    return cv::Rect(x, y, w, h);
}

}  // namespace

bool FaceTracker::probeYuNet(cv::Ptr<cv::FaceDetectorYN> detector) {
    if (detector.empty()) {
        return false;
    }
    try {
        cv::Mat img(120, 160, CV_8UC3, cv::Scalar(0, 0, 0));
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
            ROS_INFO(
                "face backend: YuNet score>=%.2f proc_max_w=%d",
                FACE_DETECT_SCORE_THRESH,
                FACE_PROC_MAX_W);
            return true;
        }
    } catch (const cv::Exception& e) {
        ROS_WARN("YuNet create failed: %s", e.what());
    }
    detector_.release();

    const std::string script = projectRoot() + "/scripts/face_mediapipe_worker.py";
    if (mp_bridge_.start(script)) {
        backend_ = FaceDetectBackend::MediaPipe;
        ROS_INFO(
            "face backend: MediaPipe worker (locate_face.py); "
            "system OpenCV YuNet DNN unavailable, using python3");
        return true;
    }

    backend_ = FaceDetectBackend::None;
    ROS_ERROR("face backend init failed");
    return false;
}

FaceTracker::FaceTracker(ros::NodeHandle& nh) {
    initFaceBackend();
    if (backend_ != FaceDetectBackend::None) {
        ROS_INFO(
            "face control: K_yaw=%.0f ema=%.2f deadband=%.2f/%.2f",
            K_YAW_DEG,
            TARGET_EMA_ALPHA,
            DEAD_BAND_X,
            DEAD_BAND_Y);
    }

    neck_pub_ = nh.advertise<sensor_msgs::JointState>(ABSOLUTE_TOPIC, 10);
    last_face_ms_ = getCurrentTimeMs();
    pub_thread_ = std::thread(&FaceTracker::publisherLoop, this);
}

FaceTracker::~FaceTracker() { shutdown(); }

void FaceTracker::setEnabled(bool on) { enabled_.store(on); }

FaceTelemetry FaceTracker::getTelemetry() const {
    std::lock_guard<std::mutex> lk(telem_mu_);
    return telem_;
}

cv::Rect FaceTracker::expandRoi(const cv::Rect& box, float pad_ratio, const cv::Size& frame_size) {
    if (box.area() <= 0) return cv::Rect();
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

cv::Rect FaceTracker::mapFrameRectToProc(
    const cv::Rect& frame_rect,
    int proc_w,
    int proc_h,
    int frame_w,
    int frame_h) {
    if (frame_rect.area() <= 0 || frame_w <= 0 || frame_h <= 0) {
        return cv::Rect();
    }
    const float sx = static_cast<float>(proc_w) / static_cast<float>(frame_w);
    const float sy = static_cast<float>(proc_h) / static_cast<float>(frame_h);
    return cv::Rect(
        static_cast<int>(frame_rect.x * sx),
        static_cast<int>(frame_rect.y * sy),
        static_cast<int>(frame_rect.width * sx),
        static_cast<int>(frame_rect.height * sy));
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

cv::Rect FaceTracker::pickBestDet(
    const std::vector<FaceDet>& dets,
    bool prefer_last,
    const cv::Rect& last_bbox) {
    if (dets.empty()) {
        return cv::Rect();
    }

    if (prefer_last && last_bbox.area() > 0) {
        const int lx = last_bbox.x + last_bbox.width / 2;
        const int ly = last_bbox.y + last_bbox.height / 2;
        const FaceDet* best = &dets[0];
        int best_dist = INT_MAX;
        for (const auto& d : dets) {
            const int cx = d.bbox.x + d.bbox.width / 2;
            const int cy = d.bbox.y + d.bbox.height / 2;
            const int dist = (cx - lx) * (cx - lx) + (cy - ly) * (cy - ly);
            if (dist < best_dist) {
                best_dist = dist;
                best = &d;
            }
        }
        return best->bbox;
    }

    const FaceDet* best = &dets[0];
    for (const auto& d : dets) {
        if (d.score > best->score) {
            best = &d;
        }
    }
    return best->bbox;
}

bool FaceTracker::detectWithYuNet(
    const cv::Mat& frame,
    float& dx_n,
    float& dy_n,
    cv::Rect& face_disp,
    float& score) {
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

    const long long now_ms = getCurrentTimeMs();
    const float lost_sec = (now_ms - last_face_ms_) / 1000.0f;
    const bool can_roi_retry =
        dets.empty() && has_last_bbox_ && lost_sec < FACE_TRACK_GRACE_SEC;

    cv::Rect last_proc_bbox;
    if (has_last_bbox_) {
        last_proc_bbox =
            mapFrameRectToProc(last_face_bbox_, proc_w, proc_h, frame.cols, frame.rows);
    }

    if (can_roi_retry && last_proc_bbox.area() > 0) {
        const cv::Rect roi = expandRoi(last_proc_bbox, FACE_ROI_PAD_RATIO, proc_bgr.size());
        runYuNet(proc_bgr, roi, dets);
    }

    const cv::Rect face_proc = pickBestDet(dets, has_last_bbox_, last_proc_bbox);
    if (face_proc.area() <= 0) {
        return false;
    }

    score = FACE_DETECT_SCORE_THRESH;
    const int cx = face_proc.x + face_proc.width / 2;
    const int cy = face_proc.y + face_proc.height / 2;
    for (const auto& d : dets) {
        const int dcx = d.bbox.x + d.bbox.width / 2;
        const int dcy = d.bbox.y + d.bbox.height / 2;
        if (d.bbox == face_proc || (dcx == cx && dcy == cy)) {
            score = std::max(score, d.score);
        }
    }

    face_disp = mapProcRectToFrame(face_proc, proc_w, proc_h, frame.cols, frame.rows);
    const float face_cx = face_disp.x + face_disp.width / 2.0f;
    const float face_cy = face_disp.y + face_disp.height / 2.0f;
    const float cx_img = frame.cols / 2.0f;
    const float cy_img = frame.rows / 2.0f;
    dx_n = (face_cx - cx_img) / (frame.cols / 2.0f);
    dy_n = (face_cy - cy_img) / (frame.rows / 2.0f);
    return true;
}

void FaceTracker::shutdown() {
    running_.store(false);
    if (pub_thread_.joinable()) pub_thread_.join();
    mp_bridge_.stop();
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

void FaceTracker::updateTargetFromError(float dx_n, float dy_n) {
    if (std::abs(dx_n) < DEAD_BAND_X) dx_n = 0.0f;
    if (std::abs(dy_n) < DEAD_BAND_Y) dy_n = 0.0f;

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

    ctrl_yaw_ = ctrl_yaw_ * (1.0f - TARGET_EMA_ALPHA) + raw_yaw_rad * TARGET_EMA_ALPHA;
    ctrl_pitch_ = ctrl_pitch_ * (1.0f - TARGET_EMA_ALPHA) + raw_pitch_rad * TARGET_EMA_ALPHA;
    ctrl_yaw_ = clampf(ctrl_yaw_, -yaw_lim_rad, yaw_lim_rad);
    ctrl_pitch_ = clampf(ctrl_pitch_, pitch_up_rad, pitch_dn_rad);
    target_yaw_ = ctrl_yaw_;
    target_pitch_ = ctrl_pitch_;
}

void FaceTracker::applyFaceTracking(
    const cv::Mat& frame,
    float dx_n,
    float dy_n,
    const cv::Rect& face_disp,
    float score,
    long long now_ms) {
    updateTargetFromError(dx_n, dy_n);
    last_face_ms_ = now_ms;
    last_face_bbox_ = face_disp;
    has_last_bbox_ = true;

    float y = 0.0f, p = 0.0f;
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
        telem_.detect_score = score;
        telem_.state = FaceTrackState::Tracking;
    }

    if (face_disp.area() > 0) {
        cv::rectangle(const_cast<cv::Mat&>(frame), face_disp, cv::Scalar(0, 255, 0), 2);
        const int cx = face_disp.x + face_disp.width / 2;
        const int cy = face_disp.y + face_disp.height / 2;
        cv::circle(const_cast<cv::Mat&>(frame), cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
    }
}

void FaceTracker::applyNoFace(long long now_ms, float dt) {
    const float lost_sec = (now_ms - last_face_ms_) / 1000.0f;

    FaceTrackState st = FaceTrackState::NoFace;
    if (lost_sec > NO_FACE_RETURN_HOME_SEC) {
        st = FaceTrackState::Homing;
        std::lock_guard<std::mutex> lk(target_mu_);
        const float step = deg2rad(RETURN_HOME_RATE_DEG_PER_SEC * dt);
        if (std::abs(target_yaw_) > 1e-4f) {
            target_yaw_ -= std::copysign(std::min(step, std::abs(target_yaw_)), target_yaw_);
            ctrl_yaw_ = target_yaw_;
        }
        if (std::abs(target_pitch_) > 1e-4f) {
            target_pitch_ -= std::copysign(std::min(step, std::abs(target_pitch_)), target_pitch_);
            ctrl_pitch_ = target_pitch_;
        }
    }

    float y = 0.0f, p = 0.0f;
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
        telem_.detect_score = 0.0f;
        telem_.state = st;
    }

    if (lost_sec > FACE_TRACK_GRACE_SEC) {
        has_last_bbox_ = false;
    }
}

void FaceTracker::trackAndControlNeck(const cv::Mat& frame) {
    if (!enabled_.load() || backend_ == FaceDetectBackend::None || frame.empty()) {
        return;
    }

    const long long now_ms = getCurrentTimeMs();
    const float dt = 0.033f;

    if (backend_ == FaceDetectBackend::MediaPipe) {
        float dx_n = 0.0f;
        float dy_n = 0.0f;
        float face_cx = 0.0f;
        float face_cy = 0.0f;
        if (mp_bridge_.detect(frame, dx_n, dy_n, face_cx, face_cy)) {
            constexpr int kPad = 28;
            cv::Rect face_disp(
                static_cast<int>(face_cx) - kPad,
                static_cast<int>(face_cy) - kPad,
                kPad * 2,
                kPad * 2);
            face_disp = clampRect(face_disp, frame.size());
            applyFaceTracking(frame, dx_n, dy_n, face_disp, 1.0f, now_ms);
        } else {
            applyNoFace(now_ms, dt);
        }
        return;
    }

    float dx_n = 0.0f;
    float dy_n = 0.0f;
    cv::Rect face_disp;
    float score = 0.0f;
    if (detectWithYuNet(frame, dx_n, dy_n, face_disp, score)) {
        applyFaceTracking(frame, dx_n, dy_n, face_disp, score, now_ms);
    } else {
        applyNoFace(now_ms, dt);
    }
}

void FaceTracker::stopNeck() {
    std::lock_guard<std::mutex> lk(target_mu_);
    target_yaw_ = 0.0f;
    target_pitch_ = 0.0f;
    ctrl_yaw_ = 0.0f;
    ctrl_pitch_ = 0.0f;
    has_last_bbox_ = false;
    publishNeck(0.0f, 0.0f);
}
