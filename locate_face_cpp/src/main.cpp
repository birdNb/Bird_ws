#include "Camera.h"
#include "Common.h"
#include "FaceTracker.h"
#include "FsmMonitor.h"

#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>

namespace {

std::atomic<bool> g_request_stop{false};

void onSignal(int sig) {
    (void)sig;
    g_request_stop.store(true);
}

struct Options {
    bool show_gui = false;
    bool no_fsm = false;
};

Options parseArgs(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--gui") == 0) {
            opt.show_gui = true;
        } else if (std::strcmp(argv[i], "--no-gui") == 0) {
            opt.show_gui = false;
        } else if (std::strcmp(argv[i], "--no-fsm") == 0) {
            opt.no_fsm = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::printf(
                "用法: locate_face [--gui] [--no-fsm]\n"
                "  默认后台运行（无 OpenCV 窗口）\n"
                "  --gui      显示全屏预览\n"
                "  --no-gui   显式关闭预览（默认）\n"
                "  --no-fsm   跳过 FSM 守门(谨慎)\n");
            std::exit(0);
        }
    }
    return opt;
}

int detectScreenWidth() {
    return 1920;
}

int detectScreenHeight() {
    return 1080;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);
    ros::init(argc, argv, "locate_face_cpp", ros::init_options::NoSigintHandler);
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
    ros::NodeHandle nh;

    Camera cam;
    cv::Mat probe;
    if (!cam.read(probe) || probe.empty()) {
        ROS_ERROR("[cam] 无法打开相机或抓帧失败，退出");
        return 1;
    }

    FsmMonitor fsm(nh, !opt.no_fsm);

    if (!opt.no_fsm) {
        ROS_INFO("[FSM] 等待 EXEC_DEFAULT(%d)...", FSM_EXEC_DEFAULT);
        if (!fsm.waitForExecDefault(FSM_WAIT_TIMEOUT_SEC)) {
            ROS_ERROR("[FSM] 未进入默认执行态，头追退出（请先 BLE: M_default）");
            return 2;
        }
        ROS_INFO("[FSM] OK, 进入视觉伺服");
    }

    FaceTracker tracker(nh, opt.no_fsm ? nullptr : &fsm);
    tracker.startPublisher();

    const bool use_gui = opt.show_gui && std::getenv("DISPLAY") != nullptr;
    int screen_w = 0;
    int screen_h = 0;
    if (use_gui) {
        screen_w = detectScreenWidth();
        screen_h = detectScreenHeight();
        cv::namedWindow("Locate Face (C++)", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Locate Face (C++)", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        ROS_INFO("[gui] 全屏预览 %dx%d", screen_w, screen_h);
    } else {
        ROS_INFO("[gui] 后台模式 (--no-gui)");
    }

    ros::Rate loop_rate(CAMERA_TARGET_FPS);
    long long fps_t0 = getCurrentTimeMs();
    int fps_frames = 0;
    float fps_show = 0.0f;
    long long last_log_ms = 0;

    while (ros::ok() && !g_request_stop.load()) {
        cv::Mat frame;
        if (!cam.read(frame) || frame.empty()) {
            ROS_WARN_THROTTLE(2.0, "[cam] 抓帧失败");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        tracker.trackFrame(frame);
        ros::spinOnce();

        ++fps_frames;
        const long long now_ms = getCurrentTimeMs();
        if (fps_frames >= 10) {
            fps_show = fps_frames * 1000.0f / std::max(1LL, now_ms - fps_t0);
            fps_t0 = now_ms;
            fps_frames = 0;
        }

        if (use_gui) {
            const FaceTelemetry telem = tracker.getTelemetry();
            cv::Mat show = frame.clone();
            const int cx = show.cols / 2;
            const int cy = show.rows / 2;
            cv::drawMarker(show, cv::Point(cx, cy), cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 24, 2);
            const int dx_pix = static_cast<int>(DEAD_BAND_X * show.cols / 2.0f);
            const int dy_pix = static_cast<int>(DEAD_BAND_Y * show.rows / 2.0f);
            cv::rectangle(
                show,
                cv::Point(cx - dx_pix, cy - dy_pix),
                cv::Point(cx + dx_pix, cy + dy_pix),
                cv::Scalar(90, 90, 90),
                1);

            char buf[160];
            std::snprintf(
                buf,
                sizeof(buf),
                "FPS %.1f  yaw=%+.1f pitch=%+.1f  %s",
                fps_show,
                telem.yaw_deg,
                telem.pitch_deg,
                telem.has_face ? "TRACKING" : "NO_FACE");
            cv::putText(
                show, buf, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            cv::Mat letterbox;
            fitLetterbox(show, letterbox, screen_w, screen_h);
            cv::imshow("Locate Face (C++)", letterbox);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q') {
                break;
            }
        }

        if (now_ms - last_log_ms >= 1000) {
            const FaceTelemetry telem = tracker.getTelemetry();
            ROS_INFO(
                "[track] face=%s  FPS=%.1f  yaw=%+.1f  pitch=%+.1f  dx=%+.2f dy=%+.2f",
                telem.has_face ? "Y" : "N",
                fps_show,
                telem.yaw_deg,
                telem.pitch_deg,
                telem.dx_norm,
                telem.dy_norm);
            last_log_ms = now_ms;
        }

        loop_rate.sleep();
    }

    ROS_INFO("[exit] 退出 -> 回中");
    tracker.shutdown();
    ros::shutdown();
    cam.release();
    if (use_gui) {
        cv::destroyAllWindows();
    }
    return 0;
}
