#pragma once

#include <opencv2/core.hpp>
#include <string>

/** 通过常驻 Python 子进程调用 MediaPipe（规避 Jetson C++ OpenCV YuNet DNN 崩溃）。 */
class MediaPipeFaceBridge {
public:
    MediaPipeFaceBridge() = default;
    ~MediaPipeFaceBridge();

    bool start(const std::string& script_path);
    void stop();
    bool isRunning() const { return child_pid_ > 0; }

    /** 输入 BGR 帧；成功时填充归一化偏差与像素中心。 */
    bool detect(
        const cv::Mat& bgr,
        float& dx_norm,
        float& dy_norm,
        float& face_cx,
        float& face_cy);

private:
    static bool writeAll(int fd, const void* data, size_t len);
    static bool readAll(int fd, void* data, size_t len);

    int child_pid_ = -1;
    int stdin_fd_ = -1;
    int stdout_fd_ = -1;
};
