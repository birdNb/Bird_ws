#pragma once

#include <opencv2/core.hpp>
#include <string>

struct MediaPipeGestureResult {
    bool has_hand = false;
    int gesture_id = -1;
    int raw_gesture_id = -1;
    bool in_range = true;
    float dx_norm = 0.0f;
    float dy_norm = 0.0f;
    float distance_m = 0.0f;
    cv::Rect hand_rect;
    std::vector<cv::Point> landmarks;
};

class MediaPipeGestureBridge {
public:
    ~MediaPipeGestureBridge();
    bool start(const std::string& script_path);
    void stop();
    bool isRunning() const { return child_pid_ > 0; }
    bool detect(const cv::Mat& bgr, MediaPipeGestureResult& out);

private:
    static bool writeAll(int fd, const void* data, size_t len);
    static bool readAll(int fd, void* data, size_t len);

    int child_pid_ = -1;
    int stdin_fd_ = -1;
    int stdout_fd_ = -1;
};
