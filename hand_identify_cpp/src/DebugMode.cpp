#include "DebugMode.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

const char* runModeName(RunMode mode) {
    switch (mode) {
        case RunMode::All: return "full";
        case RunMode::LocateFace: return "locate_face";
        case RunMode::GestureOnly: return "gesture";
        case RunMode::HandFollow: return "hand_follow";
        case RunMode::GestureAction: return "gesture_action";
        case RunMode::Coquette: return "coquette";
    }
    return "unknown";
}

void printUsage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "Debug modes (also via ./start.sh):\n"
        << "  --all              full pipeline (default)\n"
        << "  --locate_face      face/neck track only\n"
        << "  --loacate_face     alias of --locate_face\n"
        << "  --gesture          gesture preview (no robot cmd)\n"
        << "  --gesture_action   gesture 0-4 + /joy_msg actions\n"
        << "  --actions          with --gesture: enable action commands\n"
        << "  --hand_follow      chassis hand follow only\n"
        << "  --coquette         gesture 1 coquette sway only\n"
        << "  --no-joy           skip 5s joy gate\n"
        << "  --no-gui           no OpenCV window (default)\n"
        << "  --gui              show OpenCV preview window\n"
        << "  --help, -h         this help\n\n"
        << "Examples:\n"
        << "  ./start.sh --locate_face --no-joy --no-gui\n";
}

static bool matchModeFlag(const char* arg, RunMode mode, AppConfig& cfg) {
    if (!arg) return false;
    auto set = [&](RunMode m) {
        cfg.mode = m;
        return true;
    };
    if (std::strcmp(arg, "--all") == 0) return set(RunMode::All);
    if (std::strcmp(arg, "--locate_face") == 0 || std::strcmp(arg, "--loacate_face") == 0)
        return set(RunMode::LocateFace);
    if (std::strcmp(arg, "--gesture") == 0) return set(RunMode::GestureOnly);
    if (std::strcmp(arg, "--hand_follow") == 0 || std::strcmp(arg, "--hand_tracking") == 0)
        return set(RunMode::HandFollow);
    if (std::strcmp(arg, "--gesture_action") == 0 || std::strcmp(arg, "--gesture_actions") == 0)
        return set(RunMode::GestureAction);
    if (std::strcmp(arg, "--coquette") == 0) return set(RunMode::Coquette);
    return false;
}

bool parseAppConfig(int argc, char** argv, AppConfig& cfg, std::string& err) {
    cfg = AppConfig{};
    bool mode_set = false;

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (std::strcmp(arg, "--no-joy") == 0) {
            cfg.no_joy = true;
            continue;
        }
        if (std::strcmp(arg, "--no-gui") == 0) {
            cfg.no_gui = true;
            continue;
        }
        if (std::strcmp(arg, "--gui") == 0) {
            cfg.no_gui = false;
            continue;
        }
        if (std::strcmp(arg, "--actions") == 0 || std::strcmp(arg, "--enable-actions") == 0) {
            cfg.enable_gesture_actions = true;
            continue;
        }
        if (matchModeFlag(arg, cfg.mode, cfg)) {
            mode_set = true;
            continue;
        }
        if (arg[0] == '-' && arg[1] == '-') {
            err = std::string("unknown option: ") + arg;
            return false;
        }
    }

    if (!mode_set) {
        cfg.mode = RunMode::All;
    }
    return true;
}

bool initDisplay(AppConfig& cfg) {
    if (cfg.no_gui) return false;

    const char* disp = std::getenv("DISPLAY");
    if (disp == nullptr || disp[0] == '\0') {
        ROS_WARN(
            "DISPLAY not set; running headless. Use: export DISPLAY=:0 or --no-gui");
        cfg.no_gui = true;
        return false;
    }

    try {
        cv::namedWindow("vision", cv::WINDOW_NORMAL);
        cv::destroyWindow("vision");
        return true;
    } catch (const cv::Exception& e) {
        ROS_WARN(
            "OpenCV GUI unavailable (%s); using --no-gui. SSH: pass --no-gui",
            e.what());
        cfg.no_gui = true;
        return false;
    }
}
