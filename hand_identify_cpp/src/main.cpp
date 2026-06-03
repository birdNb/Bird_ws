#include "Camera.h"
#include "Common.h"
#include "Controller.h"
#include "DebugMode.h"
#include "FaceTracker.h"
#include "GestureDetector.h"
#include "HandTracker.h"
#include "JoyMonitor.h"
#include "TermDisplay.h"

#include <cstdio>

static const char* faceStateTag(const FaceTelemetry& t) {
    static char buf[48];
    switch (t.state) {
        case FaceTrackState::Tracking:
            return "TRACKING";
        case FaceTrackState::Homing:
            std::snprintf(buf, sizeof(buf), "HOMING %.1fs", t.lost_sec);
            return buf;
        case FaceTrackState::NoFace:
        default:
            if (t.lost_sec > 0.05f) {
                std::snprintf(buf, sizeof(buf), "NO_FACE %.1fs", t.lost_sec);
                return buf;
            }
            return "NO_FACE";
    }
}

static void pollFaceTerminalStatus(
    FaceTracker& face_tracker,
    FpsCounter& fps_counter,
    FaceDetectRate& detect_rate,
    TermStatusLine& term_line,
    long long& last_ros_log_ms) {
    fps_counter.tick();
    const FaceTelemetry telem = face_tracker.getTelemetry();
    detect_rate.tick(telem.has_face);

    const std::string line = TermStatusLine::formatFaceTrack(
        telem.has_face,
        faceStateTag(telem),
        fps_counter.fps(),
        detect_rate.percent(),
        detect_rate.hits(),
        detect_rate.windowSize(),
        telem.yaw_deg,
        telem.pitch_deg,
        telem.dx_norm,
        telem.dy_norm);
    term_line.print(line);

    const long long now = getCurrentTimeMs();
    if (now - last_ros_log_ms >= 1000) {
        ROS_INFO(
            "[track] face=%s  FPS=%.1f  det=%.0f%%(%d/%d)  yaw=%+.1f  pitch=%+.1f  dx=%+.2f dy=%+.2f",
            telem.has_face ? "Y" : "N",
            fps_counter.fps(),
            detect_rate.percent(),
            detect_rate.hits(),
            detect_rate.windowSize(),
            telem.yaw_deg,
            telem.pitch_deg,
            telem.dx_norm,
            telem.dy_norm);
        last_ros_log_ms = now;
    }
}

static void drawHud(cv::Mat& frame, const std::string& line, cv::Scalar color, int y = 30) {
    cv::putText(frame, line, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv::LINE_AA);
}

static int updateGestureHold(int gesture, bool has_hand, bool in_range,
                             int& candidate, long long& since_ms) {
    const long long now = getCurrentTimeMs();
    if (!has_hand || !in_range || gesture < GESTURE_1 || gesture > GESTURE_4) {
        candidate = GESTURE_NONE;
        since_ms = 0;
        return GESTURE_NONE;
    }
    if (gesture != candidate) {
        candidate = gesture;
        since_ms = now;
        return GESTURE_NONE;
    }
    if (since_ms <= 0) since_ms = now;
    if (now - since_ms >= GESTURE_HOLD_MS) return gesture;
    return GESTURE_NONE;
}

static bool joyBlocks(const AppConfig& cfg, JoyMonitor& joy) {
    return !cfg.no_joy && !joy.allowProgramControl();
}

int main(int argc, char** argv) {
    AppConfig cfg;
    std::string parse_err;
    if (!parseAppConfig(argc, argv, cfg, parse_err)) {
        std::cerr << parse_err << "\n";
        printUsage(argv[0]);
        return 1;
    }

    ros::init(argc, argv, "hand_identify_cpp");
    ros::NodeHandle nh;

    Camera cam;
    GestureDetector gesture_detector;
    FaceTracker face_tracker(nh);
    HandTracker hand_tracker(nh);
    JoyMonitor joy_monitor(nh);
    Controller controller(nh, face_tracker, hand_tracker);

    cv::Mat frame;
    int hold_candidate = GESTURE_NONE;
    long long hold_since_ms = 0;

    ROS_INFO("hand_identify_cpp started | mode=%s", runModeName(cfg.mode));
    if (cfg.no_joy) ROS_INFO("debug: joy arbitration disabled (--no-joy)");
    if (cfg.mode == RunMode::LocateFace) {
        ROS_INFO("face track only -> %s (head_yaw/pitch)", ABSOLUTE_TOPIC);
    } else if (cfg.mode == RunMode::GestureOnly) {
        ROS_INFO("gesture preview only (no robot commands)");
    } else if (cfg.mode == RunMode::HandFollow) {
        ROS_INFO("hand follow only -> %s", CMD_VEL_TOPIC);
    } else if (cfg.mode == RunMode::GestureAction) {
        ROS_INFO("gesture + actions (face track off)");
    } else if (cfg.mode == RunMode::Coquette) {
        ROS_INFO("coquette only: gesture 1 hold %dms", GESTURE_HOLD_MS);
    } else {
        ROS_INFO("joy priority: %s idle %ds | ESC quit", JOY_TOPIC, JOY_IDLE_MS / 1000);
    }

    const bool use_gui = initDisplay(cfg);
    const bool face_term_status = (cfg.mode == RunMode::LocateFace);
    TermStatusLine term_line;
    FpsCounter fps_counter;
    FaceDetectRate detect_rate;
    long long last_face_ros_log_ms = 0;
    if (face_term_status) {
        ROS_INFO("terminal status: live line (det%%/FPS/yaw/pitch), throttled ROS log 1Hz");
    }

    if (use_gui) {
        cv::namedWindow("vision", cv::WINDOW_NORMAL);
        ROS_INFO("GUI on DISPLAY=%s", std::getenv("DISPLAY") ? std::getenv("DISPLAY") : "");
    } else {
        ROS_INFO("headless (--no-gui or no DISPLAY)");
    }

    while (ros::ok() && cam.read(frame)) {
        ros::spinOnce();

        const std::string mode_tag = std::string("[") + runModeName(cfg.mode) + "]";

        if (joyBlocks(cfg, joy_monitor)) {
            controller.stopAll();
            face_tracker.setEnabled(false);
            long long rem = joy_monitor.idleRemainingMs();
            drawHud(frame, mode_tag + (joy_monitor.isActiveNow() ? " JOY active"
                                                                 : " joy wait " + std::to_string((rem + 999) / 1000) + "s"),
                    cv::Scalar(0, 0, 255));
            if (use_gui) {
                cv::imshow("vision", frame);
                if ((cv::waitKey(1) & 0xFF) == 27) break;
            }
            continue;
        }

        HandDetectResult hand;
        bool high_conf = false;
        int gesture = GESTURE_NONE;

        const bool need_gesture =
            cfg.mode == RunMode::All || cfg.mode == RunMode::GestureOnly
            || cfg.mode == RunMode::HandFollow || cfg.mode == RunMode::GestureAction
            || cfg.mode == RunMode::Coquette;

        if (need_gesture) {
            high_conf = gesture_detector.detectMaxHand(frame, hand);
            gesture = hand.gesture_id;
        }

        drawHud(frame, mode_tag + " running", cv::Scalar(0, 255, 0));
        if (need_gesture) {
            drawHud(frame,
                    "gesture:" + std::to_string(gesture) + " conf:"
                        + std::to_string(static_cast<int>(hand.confidence * 100)) + "%",
                    cv::Scalar(255, 255, 255), 70);
        }

        switch (cfg.mode) {
            case RunMode::LocateFace:
                face_tracker.setEnabled(true);
                face_tracker.trackAndControlNeck(frame);
                hand_tracker.stopChassis();
                controller.abortActions();
                if (face_term_status) {
                    pollFaceTerminalStatus(
                        face_tracker, fps_counter, detect_rate, term_line, last_face_ros_log_ms);
                }
                if (use_gui) {
                    const FaceTelemetry telem = face_tracker.getTelemetry();
                    drawHud(frame, faceStateTag(telem), cv::Scalar(0, 255, 255), 110);
                    drawHud(
                        frame,
                        "yaw " + std::to_string(static_cast<int>(telem.yaw_deg)) + " pitch "
                            + std::to_string(static_cast<int>(telem.pitch_deg)),
                        cv::Scalar(0, 255, 255),
                        150);
                }
                break;

            case RunMode::GestureOnly:
                face_tracker.setEnabled(false);
                face_tracker.stopNeck();
                hand_tracker.stopChassis();
                controller.abortActions();
                if (!hand.has_hand) {
                    drawHud(frame, "wait gesture", cv::Scalar(128, 128, 128), 110);
                }
                break;

            case RunMode::HandFollow:
                face_tracker.setEnabled(false);
                face_tracker.stopNeck();
                controller.abortActions();
                if (hand.has_hand) {
                    hand_tracker.followMaxHand(frame, hand);
                    drawHud(frame, "HAND follow", cv::Scalar(255, 0, 255), 110);
                } else {
                    hand_tracker.stopChassis();
                    drawHud(frame, "wait palm", cv::Scalar(128, 128, 128), 110);
                }
                break;

            case RunMode::GestureAction:
                face_tracker.setEnabled(false);
                face_tracker.stopNeck();
                hand_tracker.stopChassis();
                if (high_conf && gesture >= GESTURE_0 && gesture <= GESTURE_4) {
                    if (gesture == GESTURE_0) {
                        controller.abortActions();
                    } else {
                        const int confirmed = updateGestureHold(
                            gesture, hand.has_hand, hand.in_range, hold_candidate, hold_since_ms);
                        if (confirmed >= GESTURE_1 && !controller.isActionBusy()) {
                            controller.onConfirmedGesture(confirmed);
                        }
                    }
                }
                break;

            case RunMode::Coquette:
                face_tracker.setEnabled(false);
                face_tracker.stopNeck();
                hand_tracker.stopChassis();
                if (high_conf && gesture == GESTURE_1) {
                    const int confirmed = updateGestureHold(
                        gesture, hand.has_hand, hand.in_range, hold_candidate, hold_since_ms);
                    if (confirmed == GESTURE_1 && !controller.isActionBusy()) {
                        controller.onConfirmedGesture(GESTURE_1);
                    }
                    drawHud(frame, "G1 coquette...", cv::Scalar(0, 165, 255), 110);
                } else {
                    controller.abortActions();
                }
                break;

            case RunMode::All:
            default: {
                const bool enable_face = (gesture >= GESTURE_0 && gesture <= GESTURE_4);
                face_tracker.setEnabled(enable_face);

                if (!high_conf) {
                    face_tracker.trackAndControlNeck(frame);
                    hand_tracker.stopChassis();
                    controller.abortActions();
                } else if (gesture == GESTURE_5) {
                    face_tracker.stopNeck();
                    hand_tracker.followMaxHand(frame, hand);
                } else if (enable_face) {
                    face_tracker.trackAndControlNeck(frame);
                    hand_tracker.stopChassis();
                    if (gesture == GESTURE_0) {
                        controller.abortActions();
                        drawHud(frame, "G0 estop", cv::Scalar(0, 0, 255), 110);
                    } else {
                        const int confirmed = updateGestureHold(
                            gesture, hand.has_hand, hand.in_range, hold_candidate, hold_since_ms);
                        if (confirmed >= GESTURE_1 && !controller.isActionBusy()) {
                            controller.onConfirmedGesture(confirmed);
                        }
                    }
                } else {
                    controller.stopAll();
                }
                break;
            }
        }

        if (use_gui) {
            cv::imshow("vision", frame);
            if ((cv::waitKey(1) & 0xFF) == 27) break;
        }
    }

    controller.stopAll();
    face_tracker.shutdown();
    cam.release();
    if (face_term_status) {
        std::fprintf(stderr, "\n");
    }
    if (use_gui) cv::destroyAllWindows();
    ROS_INFO("hand_identify_cpp exited");
    return 0;
}
