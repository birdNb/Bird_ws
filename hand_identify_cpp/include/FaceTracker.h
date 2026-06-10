#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "Common.h"
#include "MediaPipeFaceBridge.h"
#include <opencv2/objdetect/face.hpp>
#include <sensor_msgs/JointState.h>

enum class FaceTrackState { Tracking, NoFace, Homing };

enum class FaceDetectBackend { None, YuNet, MediaPipe };

struct FaceTelemetry {
    bool has_face = false;
    float dx_norm = 0.0f;
    float dy_norm = 0.0f;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float lost_sec = 0.0f;
    float detect_score = 0.0f;
    FaceTrackState state = FaceTrackState::NoFace;
};

class FaceTracker {
public:
    explicit FaceTracker(ros::NodeHandle& nh);
    ~FaceTracker();
    void setEnabled(bool on);
    bool isEnabled() const { return enabled_.load(); }
    void trackAndControlNeck(const cv::Mat& frame, bool run_detect = true);

    void trackAndControlNeckImpl(const cv::Mat& frame, bool run_detect);
    FaceTelemetry getTelemetry() const;
    void stopNeck();
    void shutdown();

private:
    struct FaceDet {
        cv::Rect bbox;
        float score = 0.0f;
    };

    static bool probeYuNet(cv::Ptr<cv::FaceDetectorYN> detector);
    bool initFaceBackend();
    void applyFaceTracking(
        const cv::Mat& frame,
        float dx_n,
        float dy_n,
        const cv::Rect& face_disp,
        float score,
        long long now_ms,
        float ema_alpha = TARGET_EMA_ALPHA);
    void applyNoFace(long long now_ms, float dt);

    void publisherLoop();
    void publishNeck(float yaw_rad, float pitch_rad);
    void updateTargetFromError(float dx_n, float dy_n, float ema_alpha = TARGET_EMA_ALPHA);
    float predictHoldDx(float age_sec) const;
    float predictHoldDy(float age_sec) const;
    bool runYuNet(
        const cv::Mat& proc_bgr,
        const cv::Rect& roi,
        std::vector<FaceDet>& out) const;
    bool detectWithYuNet(const cv::Mat& frame, float& dx_n, float& dy_n, cv::Rect& face_disp, float& score);

    static cv::Rect expandRoi(const cv::Rect& box, float pad_ratio, const cv::Size& frame_size);
    static cv::Rect mapProcRectToFrame(
        const cv::Rect& proc_rect,
        int proc_w,
        int proc_h,
        int frame_w,
        int frame_h);
    static cv::Rect mapFrameRectToProc(
        const cv::Rect& frame_rect,
        int proc_w,
        int proc_h,
        int frame_w,
        int frame_h);
    static cv::Rect pickBestDet(
        const std::vector<FaceDet>& dets,
        bool prefer_last,
        const cv::Rect& last_bbox);

    FaceDetectBackend backend_ = FaceDetectBackend::None;
    cv::Ptr<cv::FaceDetectorYN> detector_;
    MediaPipeFaceBridge mp_bridge_;

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
    long long last_track_ms_ = 0;
    cv::Rect last_face_bbox_;
    bool has_last_bbox_ = false;
    bool hold_face_valid_ = false;
    float hold_dx_norm_ = 0.0f;
    float hold_dy_norm_ = 0.0f;
    float prev_hold_dx_norm_ = 0.0f;
    float prev_hold_dy_norm_ = 0.0f;
    float hold_vx_norm_ = 0.0f;
    float hold_vy_norm_ = 0.0f;
    long long last_detect_ms_ = 0;
    mutable std::mutex telem_mu_;
    FaceTelemetry telem_;
};
