#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "Common.h"
#include "FsmMonitor.h"
#include "MediaPipeFaceBridge.h"
#include <sensor_msgs/JointState.h>

enum class FaceTrackState { Tracking, NoFace, Homing };

enum class FaceDetectBackend { None, YuNet, YuNetPython, MediaPipe };

struct FaceTelemetry {
    bool has_face = false;
    float dx_norm = 0.0f;
    float dy_norm = 0.0f;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float lost_sec = 0.0f;
    FaceTrackState state = FaceTrackState::NoFace;
};

class FaceTracker {
public:
    FaceTracker(ros::NodeHandle& nh, FsmMonitor* fsm);
    ~FaceTracker();

    void startPublisher();
    void trackFrame(const cv::Mat& frame);
    FaceTelemetry getTelemetry() const;
    /** 平滑回中（与丢失人脸超时后相同速率），退出前调用 */
    void returnHomeBlocking();
    void shutdown();
    void writeNeckStateFile();

private:
    bool initFaceBackend();
    void publisherLoop();
    void publishNeck(float yaw_rad, float pitch_rad);
    void updateTargetFromError(float dx_n, float dy_n);
    void applyFaceTracking(float dx_n, float dy_n, long long now_ms);
    void applyNoFace(long long now_ms, float dt);
    void stepHoming(float dt);
    bool isAtCenter();

    FaceDetectBackend backend_ = FaceDetectBackend::None;
    MediaPipeFaceBridge mp_bridge_;
    FsmMonitor* fsm_ = nullptr;

    ros::Publisher neck_pub_;
    std::thread pub_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> homing_active_{false};

    std::mutex target_mu_;
    float target_yaw_ = 0.0f;
    float target_pitch_ = 0.0f;
    float ctrl_yaw_ = 0.0f;
    float ctrl_pitch_ = 0.0f;
    long long last_face_ms_ = 0;
    long long last_track_ms_ = 0;

    mutable std::mutex telem_mu_;
    FaceTelemetry telem_;
};
