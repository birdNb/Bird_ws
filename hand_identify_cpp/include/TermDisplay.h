#pragma once

#include "Common.h"

#include <cstdio>
#include <string>

/** 终端单行刷新状态（对齐 emit_status_line + colorama） */
class TermStatusLine {
public:
    void print(const std::string& text, int width = 96);

    static std::string formatFaceTrack(
        bool has_face,
        const char* state_tag,
        float fps,
        float detect_rate_pct,
        int detect_hits,
        int detect_window,
        float yaw_deg,
        float pitch_deg,
        float dx_norm,
        float dy_norm);

    static std::string formatGesturePreview(
        bool has_hand,
        int gesture,
        int raw_gesture,
        bool in_range,
        float distance_m,
        float fps,
        float stable_pct,
        int stable_hits,
        int stable_window);
};

class FpsCounter {
public:
    void tick();
    float fps() const { return fps_; }

private:
    int frames_ = 0;
    long long t0_ms_ = 0;
    float fps_ = 0.0f;
};

class FaceDetectRate {
public:
    void tick(bool detected);
    float percent() const;
    int hits() const { return hits_; }
    int windowSize() const { return filled_; }

private:
    static constexpr int kWindow = 30;
    int hits_ = 0;
    int total_ = 0;
    int buf_[kWindow] = {};
    int idx_ = 0;
    int filled_ = 0;
};

class GestureStableRate {
public:
    void tick(bool stable);
    float percent() const;
    int hits() const { return hits_; }
    int windowSize() const { return filled_; }

private:
    static constexpr int kWindow = 30;
    int hits_ = 0;
    int buf_[kWindow] = {};
    int idx_ = 0;
    int filled_ = 0;
};
