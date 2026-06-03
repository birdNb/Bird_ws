#include "Camera.h"
#include "Common.h"
#include "Controller.h"
#include "FaceTracker.h"
#include "GestureDetector.h"
#include "HandTracker.h"
#include "JoyMonitor.h"

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

int main(int argc, char** argv) {
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

    ROS_INFO("hand_identify_cpp 启动");
    ROS_INFO("手柄优先: %s 无输入 %d s 后启用视觉", JOY_TOPIC, JOY_IDLE_MS / 1000);
    ROS_INFO("手势0~4: 脖子跟随+动作(2s稳定, 2s冷却); 手势5: 五指底盘跟随");
    ROS_INFO("低置信度: 仅人脸跟踪; 多手取面积最大; ESC 退出");

    cv::namedWindow("视觉控制", cv::WINDOW_NORMAL);

    while (ros::ok() && cam.read(frame)) {
        ros::spinOnce();

        if (!joy_monitor.allowProgramControl()) {
            controller.stopAll();
            face_tracker.setEnabled(false);
            long long rem = joy_monitor.idleRemainingMs();
            drawHud(frame, joy_monitor.isActiveNow() ? "手柄控制中" : "等待手柄空闲 " + std::to_string((rem + 999) / 1000) + "s",
                    cv::Scalar(0, 0, 255));
            cv::imshow("视觉控制", frame);
            if ((cv::waitKey(1) & 0xFF) == 27) break;
            continue;
        }

        HandDetectResult hand;
        const bool high_conf = gesture_detector.detectMaxHand(frame, hand);
        const int gesture = hand.gesture_id;

        const bool enable_face_track =
            (gesture >= GESTURE_0 && gesture <= GESTURE_4);
        face_tracker.setEnabled(enable_face_track);

        drawHud(frame, "视觉控制中", cv::Scalar(0, 255, 0));
        drawHud(frame,
                "手势:" + std::to_string(gesture) + " 置信:" + std::to_string(static_cast<int>(hand.confidence * 100)) + "%",
                cv::Scalar(255, 255, 255), 70);

        if (!high_conf) {
            face_tracker.trackAndControlNeck(frame);
            hand_tracker.stopChassis();
            controller.abortActions();
        } else if (gesture == GESTURE_5) {
            face_tracker.stopNeck();
            hand_tracker.followMaxHand(frame, hand);
        } else if (enable_face_track) {
            face_tracker.trackAndControlNeck(frame);
            hand_tracker.stopChassis();
            if (gesture == GESTURE_0) {
                controller.abortActions();
                drawHud(frame, "手势0 急停", cv::Scalar(0, 0, 255), 110);
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

        cv::imshow("视觉控制", frame);
        if ((cv::waitKey(1) & 0xFF) == 27) break;
    }

    controller.stopAll();
    face_tracker.shutdown();
    cam.release();
    cv::destroyAllWindows();
    ROS_INFO("hand_identify_cpp 已退出");
    return 0;
}
