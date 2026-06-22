#include "MediaPipeFaceBridge.h"

#include "Common.h"

#include <ros/ros.h>

#include <cerrno>
#include <unistd.h>

#include <sys/types.h>
#include <sys/wait.h>

namespace {

constexpr int FACE_IPC_MAX_W = 480;

bool readLineReady(int fd) {
    char ch = 0;
    std::string line;
    while (true) {
        const ssize_t n = read(fd, &ch, 1);
        if (n <= 0) {
            return false;
        }
        if (ch == '\n') {
            break;
        }
        line.push_back(ch);
        if (line.size() > 64) {
            return false;
        }
    }
    return line == "READY";
}

}  // namespace

MediaPipeFaceBridge::~MediaPipeFaceBridge() { stop(); }

bool MediaPipeFaceBridge::writeAll(int fd, const void* data, size_t len) {
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        const ssize_t n = write(fd, p, len);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (n == 0) {
            return false;
        }
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

bool MediaPipeFaceBridge::readAll(int fd, void* data, size_t len) {
    char* p = static_cast<char*>(data);
    while (len > 0) {
        const ssize_t n = read(fd, p, len);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (n == 0) {
            return false;
        }
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

bool MediaPipeFaceBridge::start(const std::string& script_path) {
    stop();

    int pipe_to_child[2] = {-1, -1};
    int pipe_from_child[2] = {-1, -1};
    if (pipe(pipe_to_child) != 0 || pipe(pipe_from_child) != 0) {
        ROS_ERROR("MediaPipe bridge: pipe() failed");
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        ROS_ERROR("MediaPipe bridge: fork() failed");
        close(pipe_to_child[0]);
        close(pipe_to_child[1]);
        close(pipe_from_child[0]);
        close(pipe_from_child[1]);
        return false;
    }

    if (pid == 0) {
        close(pipe_to_child[1]);
        close(pipe_from_child[0]);
        dup2(pipe_to_child[0], STDIN_FILENO);
        dup2(pipe_from_child[1], STDOUT_FILENO);
        close(pipe_to_child[0]);
        close(pipe_from_child[1]);
        execlp("python3", "python3", script_path.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }

    close(pipe_to_child[0]);
    close(pipe_from_child[1]);
    stdin_fd_ = pipe_to_child[1];
    stdout_fd_ = pipe_from_child[0];
    child_pid_ = pid;

    if (!readLineReady(stdout_fd_)) {
        ROS_ERROR("MediaPipe bridge: worker did not send READY (mediapipe 未安装?)");
        stop();
        return false;
    }
    return true;
}

void MediaPipeFaceBridge::stop() {
    if (stdin_fd_ >= 0) {
        close(stdin_fd_);
        stdin_fd_ = -1;
    }
    if (stdout_fd_ >= 0) {
        close(stdout_fd_);
        stdout_fd_ = -1;
    }
    if (child_pid_ > 0) {
        kill(child_pid_, SIGTERM);
        int status = 0;
        waitpid(child_pid_, &status, 0);
        child_pid_ = -1;
    }
}

bool MediaPipeFaceBridge::detect(
    const cv::Mat& bgr,
    float& dx_norm,
    float& dy_norm,
    float& face_cx,
    float& face_cy) {
    if (!isRunning() || bgr.empty() || bgr.type() != CV_8UC3) {
        return false;
    }

    int ipc_w = 0;
    int ipc_h = 0;
    computeProcSize(bgr.cols, bgr.rows, FACE_IPC_MAX_W, ipc_w, ipc_h);
    cv::Mat ipc;
    const float scale_x =
        ipc_w != bgr.cols ? static_cast<float>(bgr.cols) / static_cast<float>(ipc_w) : 1.0f;
    const float scale_y =
        ipc_h != bgr.rows ? static_cast<float>(bgr.rows) / static_cast<float>(ipc_h) : 1.0f;
    if (ipc_w != bgr.cols || ipc_h != bgr.rows) {
        cv::resize(bgr, ipc, cv::Size(ipc_w, ipc_h), 0, 0, cv::INTER_AREA);
    } else {
        ipc = bgr;
    }

    cv::Mat contiguous = ipc;
    if (!ipc.isContinuous()) {
        contiguous = ipc.clone();
    }

    const uint32_t w = static_cast<uint32_t>(contiguous.cols);
    const uint32_t h = static_cast<uint32_t>(contiguous.rows);
    if (!writeAll(stdin_fd_, &w, sizeof(w)) || !writeAll(stdin_fd_, &h, sizeof(h))) {
        return false;
    }
    const size_t nbytes = static_cast<size_t>(w) * h * 3;
    if (!writeAll(stdin_fd_, contiguous.data, nbytes)) {
        return false;
    }

    uint8_t has = 0;
    if (!readAll(stdout_fd_, &has, 1)) {
        return false;
    }
    if (has == 0) {
        return false;
    }

    float vals[4] = {};
    if (!readAll(stdout_fd_, vals, sizeof(vals))) {
        return false;
    }
    dx_norm = vals[0];
    dy_norm = vals[1];
    face_cx = vals[2] * scale_x;
    face_cy = vals[3] * scale_y;
    return true;
}
