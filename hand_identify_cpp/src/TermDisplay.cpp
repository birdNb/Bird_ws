#include "TermDisplay.h"

#include "Common.h"

#include <algorithm>
#include <cstdio>

void TermStatusLine::print(const std::string& text, int width) {
    const int pad = std::max(0, width - static_cast<int>(text.size()));
    std::fprintf(stderr, "\r%s%*s", text.c_str(), pad, "");
    std::fflush(stderr);
}

std::string TermStatusLine::formatFaceTrack(
    bool has_face,
    const char* state_tag,
    float fps,
    float detect_rate_pct,
    int detect_hits,
    int detect_window,
    float yaw_deg,
    float pitch_deg,
    float dx_norm,
    float dy_norm) {
    char buf[256];
    std::snprintf(
        buf,
        sizeof(buf),
        "[face] %s  face=%c  det=%3.0f%%(%d/%d)  FPS=%5.1f  "
        "yaw=%+6.1f  pitch=%+6.1f  dx=%+.2f dy=%+.2f",
        state_tag,
        has_face ? 'Y' : 'N',
        detect_rate_pct,
        detect_hits,
        detect_window,
        fps,
        yaw_deg,
        pitch_deg,
        dx_norm,
        dy_norm);
    return std::string(buf);
}

void FpsCounter::tick() {
    const long long now = getCurrentTimeMs();
    if (t0_ms_ <= 0) {
        t0_ms_ = now;
        frames_ = 0;
        return;
    }
    ++frames_;
    if (frames_ >= 10) {
        const float dt = (now - t0_ms_) / 1000.0f;
        if (dt > 1e-3f) {
            fps_ = frames_ / dt;
        }
        frames_ = 0;
        t0_ms_ = now;
    }
}

void FaceDetectRate::tick(bool detected) {
    if (filled_ < kWindow) {
        ++filled_;
    }
    if (buf_[idx_] != 0) {
        --hits_;
    }
    buf_[idx_] = detected ? 1 : 0;
    if (detected) {
        ++hits_;
    }
    idx_ = (idx_ + 1) % kWindow;
    total_ = filled_;
}

float FaceDetectRate::percent() const {
    if (total_ <= 0) return 0.0f;
    return 100.0f * static_cast<float>(hits_) / static_cast<float>(total_);
}
