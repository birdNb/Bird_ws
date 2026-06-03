#include "Camera.h"
#include "Common.h"
#include "Controller.h"
#include "DebugMode.h"
#include "FaceTracker.h"
#include "GestureDecision.h"
#include "GestureDetector.h"
#include "HandFollowController.h"
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

static void drawHud(
    cv::Mat& frame,
    const std::string& line,
    cv::Scalar color,
    int y = 30,
    double scale = 0.8,
    int thickness = 2) {
    cv::putText(
        frame,
        line,
        cv::Point(10, y),
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv::LINE_AA);
}

static void drawHandLandmarks(cv::Mat& frame, const std::vector<cv::Point>& lm, cv::Scalar line_col,
                              cv::Scalar dot_col) {
    if (static_cast<int>(lm.size()) != HAND_LANDMARK_COUNT) {
        return;
    }
    static const int kConnections[][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4},       {0, 5},  {5, 6},  {6, 7},  {7, 8},
        {0, 9}, {9, 10}, {10, 11}, {11, 12}, {0, 13}, {13, 14}, {14, 15}, {15, 16},
        {0, 17}, {17, 18}, {18, 19}, {19, 20}, {5, 9}, {9, 13}, {13, 17},
    };
    for (const auto& conn : kConnections) {
        cv::line(frame, lm[conn[0]], lm[conn[1]], line_col, 2, cv::LINE_AA);
    }
    for (const auto& p : lm) {
        cv::circle(frame, p, 4, dot_col, -1, cv::LINE_AA);
    }
}

static void drawGestureOverlay(cv::Mat& frame, const HandDetectResult& hand) {
    const cv::Scalar col =
        (hand.gesture_id >= 0 && hand.gesture_id <= 5) ? gestureColorBgr(hand.gesture_id)
                                                         : cv::Scalar(128, 128, 128);

    if (hand.has_landmarks) {
        drawHandLandmarks(frame, hand.landmarks, cv::Scalar(121, 22, 76), cv::Scalar(250, 44, 250));
    }

    if (hand.hand_rect.area() > 0) {
        cv::rectangle(frame, hand.hand_rect, col, 2);
    }

    if (!hand.has_hand) {
        drawHud(frame, "No hand", cv::Scalar(0, 255, 255), 130, 0.75, 2);
        return;
    }
    if (gestureRequiresDistanceGate(hand.gesture_id) && !hand.in_range) {
        drawHud(frame, "G5: move to follow range", cv::Scalar(0, 255, 255), 130, 0.75, 2);
        return;
    }
    if (hand.gesture_id >= 0 && hand.gesture_id <= 5) {
        std::string gtext = "Gesture: " + std::to_string(hand.gesture_id);
        if (hand.raw_gesture_id >= 0 && hand.raw_gesture_id != hand.gesture_id) {
            gtext += " (raw " + std::to_string(hand.raw_gesture_id) + ")";
        }
        drawHud(frame, gtext, gestureColorBgr(hand.gesture_id), 130, 1.0, 3);
        if (gestureRequiresDistanceGate(hand.gesture_id)) {
            drawHud(
                frame,
                "dist " + std::to_string(static_cast<int>(hand.distance_m * 100) / 100.0f) + "m",
                cv::Scalar(255, 255, 255),
                170,
                0.65,
                1);
        }
    }
}

struct GestureSenseCache {
    HandDetectResult last{};
    long long last_ms = 0;
    int tick = 0;

    void sense(
        GestureDetector& detector,
        const cv::Mat& frame,
        bool need_gesture,
        bool companion_mode,
        HandDetectResult& out) {
        if (!need_gesture) {
            out = HandDetectResult{};
            return;
        }
        ++tick;
        const long long now = getCurrentTimeMs();
        const bool run_detect = shouldRunGestureDetectThisFrame(tick, true, companion_mode);
        if (run_detect) {
            detector.detectMaxHand(frame, out);
            last = out;
            last_ms = now;
            return;
        }
        if (last_ms > 0 && (now - last_ms) <= HAND_CACHE_MAX_AGE_MS) {
            out = last;
            return;
        }
        out = HandDetectResult{};
    }
};

static void pollGestureTerminalStatus(
    const HandDetectResult& hand,
    FpsCounter& fps_counter,
    GestureStableRate& stable_rate,
    TermStatusLine& term_line,
    long long& last_term_ms,
    long long& last_ros_log_ms) {
    fps_counter.tick();
    const bool stable =
        hand.has_hand && hand.gesture_id >= 0
        && (hand.raw_gesture_id < 0 || hand.raw_gesture_id == hand.gesture_id)
        && (!gestureRequiresDistanceGate(hand.gesture_id) || hand.in_range);
    stable_rate.tick(stable);

    const long long now = getCurrentTimeMs();
    if (now - last_term_ms >= GESTURE_LOG_INTERVAL_MS) {
        const std::string line = TermStatusLine::formatGesturePreview(
            hand.has_hand,
            hand.gesture_id,
            hand.raw_gesture_id,
            hand.in_range,
            hand.distance_m,
            fps_counter.fps(),
            stable_rate.percent(),
            stable_rate.hits(),
            stable_rate.windowSize());
        term_line.print(line);
        last_term_ms = now;
    }

    if (now - last_ros_log_ms >= 1000) {
        if (hand.has_hand && hand.gesture_id >= 0) {
            ROS_INFO(
                "[gesture] G%d raw=%d range=%s dist=%.2fm stable=%.0f%% fps=%.1f",
                hand.gesture_id,
                hand.raw_gesture_id,
                hand.in_range ? "ok" : "out",
                hand.distance_m,
                stable_rate.percent(),
                fps_counter.fps());
        } else {
            ROS_INFO("[gesture] no hand  FPS=%.1f", fps_counter.fps());
        }
        last_ros_log_ms = now;
    }
}

static bool handLostTooLong(bool has_hand, long long now, long long& lost_since_ms) {
    if (has_hand) {
        lost_since_ms = 0;
        return false;
    }
    if (lost_since_ms <= 0) {
        lost_since_ms = now;
    }
    return (now - lost_since_ms) >= HAND_LOST_GRACE_MS;
}

static int updateGestureHold(int gesture, bool has_hand, bool /*in_range*/,
                             int& candidate, long long& since_ms) {
    const long long now = getCurrentTimeMs();
    static long long lost_since_ms = 0;

    if (handLostTooLong(has_hand, now, lost_since_ms)) {
        candidate = GESTURE_NONE;
        since_ms = 0;
        return GESTURE_NONE;
    }
    if (gesture < GESTURE_1 || gesture > GESTURE_4) {
        candidate = GESTURE_NONE;
        since_ms = 0;
        return GESTURE_NONE;
    }
    if (gesture != candidate) {
        candidate = gesture;
        since_ms = now;
        return GESTURE_NONE;
    }
    if (since_ms <= 0) {
        since_ms = now;
    }
    if (now - since_ms >= GESTURE_HOLD_MS) {
        return gesture;
    }
    return GESTURE_NONE;
}

static bool joyBlocks(const AppConfig& cfg, JoyMonitor& joy) {
    return !cfg.no_joy && !joy.allowProgramControl();
}

static bool gestureActionsEnabled(const AppConfig& cfg) {
    return cfg.mode == RunMode::GestureAction
           || (cfg.mode == RunMode::GestureOnly && cfg.enable_gesture_actions);
}

static bool gestureWithFaceTrack(RunMode mode) {
    return mode == RunMode::GestureAction || mode == RunMode::GestureOnly;
}

static void runCompanionFaceTrack(
    FaceTracker& face_tracker,
    cv::Mat& frame,
    bool use_gui,
    int sched_tick,
    bool companion_mode) {
    face_tracker.setEnabled(true);
    const bool run_detect =
        !companion_mode || shouldRunFaceDetectThisFrame(sched_tick, companion_mode);
    face_tracker.trackAndControlNeck(frame, run_detect);

    if (!use_gui) {
        return;
    }
    const FaceTelemetry telem = face_tracker.getTelemetry();
    if (telem.has_face) {
        drawHud(
            frame,
            "FACE yaw " + std::to_string(static_cast<int>(telem.yaw_deg)) + " pitch "
                + std::to_string(static_cast<int>(telem.pitch_deg)),
            cv::Scalar(0, 255, 128),
            110);
    } else {
        drawHud(frame, "FACE search", cv::Scalar(0, 200, 255), 110);
    }
}

static void processGestureActions(
    Controller& controller,
    const GestureDecision& decision,
    const HandDetectResult& hand,
    int& hold_candidate,
    long long& hold_since_ms) {
    static int last_fired_confirmed = GESTURE_NONE;
    const int gesture = decision.gesture;

    if (!decision.action_ready || gesture < GESTURE_0 || gesture > GESTURE_4) {
        last_fired_confirmed = GESTURE_NONE;
        return;
    }

    if (gesture == GESTURE_0) {
        controller.abortActions();
        last_fired_confirmed = GESTURE_NONE;
        ROS_INFO_THROTTLE(1.0, "[action] G0 estop -> abort");
        return;
    }

    const int confirmed = updateGestureHold(
        gesture, hand.has_hand, hand.in_range, hold_candidate, hold_since_ms);

    if (gesture != confirmed) {
        last_fired_confirmed = GESTURE_NONE;
    }

    if (confirmed >= GESTURE_1) {
        if (confirmed != last_fired_confirmed && !controller.isActionBusy()) {
            if (controller.onConfirmedGesture(confirmed)) {
                last_fired_confirmed = confirmed;
            }
        }
    }
}

static void drawG5HoldHud(cv::Mat& frame, const HandFollowController& follow) {
    if (!follow.holdPending()) {
        return;
    }
    drawHud(
        frame,
        "G5 hold " + std::to_string(follow.holdProgressPct()) + "% / "
            + std::to_string(GESTURE_FOLLOW_HOLD_MS / 1000) + "s",
        gestureColorBgr(GESTURE_5),
        240,
        0.75,
        2);
}

static void drawHandFollowHud(
    cv::Mat& frame,
    const HandFollowController& follow,
    const HandTracker& tracker,
    const JoyMonitor& joy,
    const AppConfig& cfg,
    int y_base = 110) {
    if (!follow.g5Confirmed()) {
        return;
    }
    drawHud(
        frame,
        "htrack " + follow.statusMode(),
        cv::Scalar(0, 255, 128),
        y_base,
        0.65,
        2);
    if (!cfg.no_joy && joy.blocksHandTracking()) {
        drawHud(
            frame,
            "joy " + std::to_string((joy.idleRemainingMs() + 999) / 1000) + "s",
            cv::Scalar(0, 165, 255),
            y_base + 32,
            0.6,
            2);
    } else {
        drawHud(
            frame,
            "x=" + std::to_string(tracker.lastLinearX())
                + " rz=" + std::to_string(tracker.lastAngularZ()),
            cv::Scalar(255, 255, 255),
            y_base + 32,
            0.55,
            2);
    }
}

static void drawGestureHoldHud(
    cv::Mat& frame,
    int gesture,
    int hold_candidate,
    long long hold_since_ms) {
    if (gesture < GESTURE_1 || gesture > GESTURE_4 || gesture != hold_candidate) {
        return;
    }
    const long long now = getCurrentTimeMs();
    if (hold_since_ms <= 0) {
        return;
    }
    const int pct = std::min(
        100,
        static_cast<int>(100 * (now - hold_since_ms) / std::max(1, GESTURE_HOLD_MS)));
    drawHud(
        frame,
        "hold G" + std::to_string(gesture) + " " + std::to_string(pct) + "%",
        gestureColorBgr(gesture),
        210,
        0.75,
        2);
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
    HandFollowController hand_follow;

    cv::Mat frame;
    int hold_candidate = GESTURE_NONE;
    long long hold_since_ms = 0;

    ROS_INFO("hand_identify_cpp started | mode=%s", runModeName(cfg.mode));
    if (cfg.no_joy) ROS_INFO("debug: joy arbitration disabled (--no-joy)");
    if (cfg.mode == RunMode::LocateFace) {
        ROS_INFO("face track only -> %s (head_yaw/pitch)", ABSOLUTE_TOPIC);
    } else if (cfg.mode == RunMode::GestureOnly) {
        if (cfg.enable_gesture_actions) {
            ROS_INFO(
                "gesture preview + actions -> %s (hold %dms for G1-4)",
                JOY_MSG_TOPIC,
                GESTURE_HOLD_MS);
        } else {
            ROS_INFO("gesture preview only (add --actions to send /joy_msg)");
        }
    } else if (cfg.mode == RunMode::HandFollow) {
        ROS_INFO(
            "[htrack] distance_hold -> %s | G5即跟手 Z=%.2fm |X|=%.1f |Z|=%.1f",
            CMD_VEL_TOPIC,
            TARGET_DISTANCE_M,
            LINEAR_X_MAG,
            ANGULAR_Z_MAG);
    } else if (cfg.mode == RunMode::GestureAction) {
        ROS_INFO(
            "gesture + face + actions | G5 hold %.0fs then distance_hold Z=%.2fm",
            GESTURE_FOLLOW_HOLD_MS / 1000.0f,
            TARGET_DISTANCE_M);
    } else if (cfg.mode == RunMode::Coquette) {
        ROS_INFO("coquette only: gesture 1 hold %dms", GESTURE_HOLD_MS);
    } else {
        ROS_INFO("joy priority: %s idle %ds | ESC quit", JOY_TOPIC, JOY_IDLE_MS / 1000);
    }

    const bool use_gui = initDisplay(cfg);
    const bool face_term_status = (cfg.mode == RunMode::LocateFace);
    const bool gesture_term_status =
        cfg.mode == RunMode::GestureOnly || cfg.mode == RunMode::GestureAction;
    const bool companion_face = gestureWithFaceTrack(cfg.mode);
    TermStatusLine term_line;
    FpsCounter fps_counter;
    FaceDetectRate detect_rate;
    GestureStableRate gesture_stable_rate;
    long long last_face_ros_log_ms = 0;
    long long last_gesture_term_ms = 0;
    long long last_gesture_ros_log_ms = 0;
    if (face_term_status) {
        ROS_INFO("terminal status: live line (det%%/FPS/yaw/pitch), throttled ROS log 1Hz");
    }
    if (gesture_term_status) {
        ROS_INFO(
            "terminal status: colored gesture G0-5, refresh %.0fms, loop %d fps",
            GESTURE_LOG_INTERVAL_MS,
            MAIN_LOOP_FPS);
    }

    if (use_gui) {
        cv::namedWindow("vision", cv::WINDOW_NORMAL);
        cv::setWindowProperty("vision", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("vision", DISPLAY_W, DISPLAY_H);
        ROS_INFO(
            "GUI %dx%d (16:9 letterbox) DISPLAY=%s",
            DISPLAY_W,
            DISPLAY_H,
            std::getenv("DISPLAY") ? std::getenv("DISPLAY") : "");
    } else {
        ROS_INFO("headless (--no-gui or no DISPLAY)");
    }

    ros::Rate loop_rate(MAIN_LOOP_FPS);
    GestureSenseCache gesture_cache;
    int sched_tick = 0;
    while (ros::ok() && cam.read(frame)) {
        ++sched_tick;
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
        const bool need_gesture =
            cfg.mode == RunMode::All || cfg.mode == RunMode::GestureOnly
            || cfg.mode == RunMode::HandFollow || cfg.mode == RunMode::GestureAction
            || cfg.mode == RunMode::Coquette;

        gesture_cache.sense(
            gesture_detector, frame, need_gesture, companion_face, hand);
        const GestureDecision decision = evaluateGestureDecision(hand);
        const int gesture = decision.gesture;

        drawHud(frame, mode_tag + " running", cv::Scalar(0, 255, 0));
        if (need_gesture && hand.has_hand && hand.gesture_id >= 0) {
            drawHud(
                frame,
                "G" + std::to_string(gesture) + " conf:"
                    + std::to_string(static_cast<int>(hand.confidence * 100)) + "%",
                gestureColorBgr(gesture),
                70);
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
                if (companion_face) {
                    runCompanionFaceTrack(
                        face_tracker, frame, use_gui, sched_tick, companion_face);
                } else {
                    face_tracker.setEnabled(false);
                    face_tracker.stopNeck();
                }
                hand_tracker.stopChassis();
                if (!gestureActionsEnabled(cfg)) {
                    controller.abortActions();
                }
                drawGestureOverlay(frame, hand);
                if (gesture_term_status) {
                    pollGestureTerminalStatus(
                        hand,
                        fps_counter,
                        gesture_stable_rate,
                        term_line,
                        last_gesture_term_ms,
                        last_gesture_ros_log_ms);
                }
                if (gestureActionsEnabled(cfg)) {
                    drawGestureHoldHud(frame, gesture, hold_candidate, hold_since_ms);
                    processGestureActions(
                        controller, decision, hand, hold_candidate, hold_since_ms);
                }
                break;

            case RunMode::HandFollow:
                controller.abortActions();
                drawGestureOverlay(frame, hand);
                hand_follow.update(
                    frame, hand_tracker, face_tracker, joy_monitor, cfg, decision, hand, false);
                drawG5HoldHud(frame, hand_follow);
                drawHandFollowHud(frame, hand_follow, hand_tracker, joy_monitor, cfg);
                break;

            case RunMode::GestureAction: {
                if (hand_follow.shouldPauseCompanionFace(companion_face)) {
                    HandFollowController::pauseCompanionFace(face_tracker);
                } else {
                    runCompanionFaceTrack(
                        face_tracker, frame, use_gui, sched_tick, companion_face);
                }
                drawGestureOverlay(frame, hand);
                if (gesture_term_status) {
                    pollGestureTerminalStatus(
                        hand,
                        fps_counter,
                        gesture_stable_rate,
                        term_line,
                        last_gesture_term_ms,
                        last_gesture_ros_log_ms);
                }
                hand_follow.update(
                    frame, hand_tracker, face_tracker, joy_monitor, cfg, decision, hand,
                    companion_face);
                if (!hand_follow.g5Confirmed()) {
                    drawGestureHoldHud(frame, gesture, hold_candidate, hold_since_ms);
                    processGestureActions(
                        controller, decision, hand, hold_candidate, hold_since_ms);
                } else {
                    controller.abortActions();
                }
                drawG5HoldHud(frame, hand_follow);
                drawHandFollowHud(frame, hand_follow, hand_tracker, joy_monitor, cfg, 240);
                break;
            }

            case RunMode::Coquette:
                face_tracker.setEnabled(false);
                face_tracker.stopNeck();
                hand_tracker.stopChassis();
                if (decision.action_ready && gesture == GESTURE_1) {
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
                if (hand_follow.shouldPauseCompanionFace(true)) {
                    HandFollowController::pauseCompanionFace(face_tracker);
                } else if (decision.companion_face_phase) {
                    face_tracker.setEnabled(true);
                    face_tracker.trackAndControlNeck(
                        frame, shouldRunFaceDetectThisFrame(sched_tick, true));
                }
                hand_follow.update(
                    frame, hand_tracker, face_tracker, joy_monitor, cfg, decision, hand, true);
                if (hand_follow.g5Confirmed()) {
                    controller.abortActions();
                    drawHandFollowHud(frame, hand_follow, hand_tracker, joy_monitor, cfg, 110);
                } else if (decision.companion_face_phase) {
                    hand_tracker.stopChassis();
                    drawG5HoldHud(frame, hand_follow);
                    if (gesture == GESTURE_0) {
                        controller.abortActions();
                        hand_follow.reset(hand_tracker, &face_tracker);
                        drawHud(frame, "G0 estop", cv::Scalar(0, 0, 255), 110);
                    } else if (!hand_follow.holdPending()) {
                        const int confirmed = updateGestureHold(
                            gesture, hand.has_hand, hand.in_range, hold_candidate, hold_since_ms);
                        if (confirmed >= GESTURE_1 && !controller.isActionBusy()) {
                            controller.onConfirmedGesture(confirmed);
                        }
                    }
                } else {
                    hand_tracker.stopChassis();
                    controller.abortActions();
                }
                break;
            }
        }

        if (use_gui) {
            cv::imshow("vision", frame);
            if ((cv::waitKey(1) & 0xFF) == 27) break;
        }
        loop_rate.sleep();
    }

    controller.stopAll();
    face_tracker.shutdown();
    cam.release();
    if (face_term_status || gesture_term_status) {
        std::fprintf(stderr, "\n");
    }
    if (use_gui) cv::destroyAllWindows();
    ROS_INFO("hand_identify_cpp exited");
    return 0;
}
