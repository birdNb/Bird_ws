#pragma once

#include <string>

/** 功能断点调试模式（./start.sh --locate_face 等） */
enum class RunMode {
    All,              // 完整流程（默认）
    LocateFace,       // 仅人脸/脖子跟踪
    GestureOnly,      // 仅手势识别预览，不发指令
    HandFollow,       // 仅五指底盘跟随
    GestureAction,    // 手势 + 动作库(0~4)，不启人脸跟踪
    Coquette,         // 仅测手势1撒娇扭腰（稳定2s后触发）
};

struct AppConfig {
    RunMode mode = RunMode::All;
    bool no_joy = false;   // 跳过手柄 5s 仲裁
    bool no_gui = true;  // 默认无窗口，与 zed --no-gui 一致；用 --gui 开启
    /** --actions：在 --gesture 预览下也下发 /joy_msg 动作（等同 --gesture_action） */
    bool enable_gesture_actions = false;
};

/** 解析 argv；未知参数返回 false 并写入 err */
bool parseAppConfig(int argc, char** argv, AppConfig& cfg, std::string& err);

/** DISPLAY 未设置或 OpenCV 无法创建窗口时自动 no_gui */
bool initDisplay(AppConfig& cfg);

const char* runModeName(RunMode mode);
void printUsage(const char* prog);
