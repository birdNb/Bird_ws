#pragma once

#include "Common.h"

#ifdef HAVE_SIM2REAL_MSG
#include <sim2real_msg/Joy.h>

/** 扳机松开=1、按键松开=0，避免全 0 消息被实机当成误触发 */
inline void fillJoyNeutral(sim2real_msg::Joy& msg) {
    msg = sim2real_msg::Joy();
    msg.lt = JOY_TRIGGER_REST;
    msg.rt = JOY_TRIGGER_REST;
}

inline float joyKeyValue(const std::string& token, bool pressed) {
    if (token == "lt" || token == "rt") {
        return pressed ? (JOY_TRIGGER_REST - JOY_TRIGGER_ACTIVE_MARGIN)
                         : JOY_TRIGGER_REST;
    }
    return pressed ? 1.0f : 0.0f;
}

inline void applyJoyToken(sim2real_msg::Joy& msg, const std::string& token, bool pressed) {
    const float v = joyKeyValue(token, pressed);
    if (token == "a") msg.a = v;
    else if (token == "b") msg.b = v;
    else if (token == "x") msg.x = v;
    else if (token == "y") msg.y = v;
    else if (token == "rt") msg.rt = v;
    else if (token == "lt") msg.lt = v;
}

inline sim2real_msg::Joy makeJoyCombo(const std::string& combo, bool pressed) {
    sim2real_msg::Joy msg;
    fillJoyNeutral(msg);
    size_t start = 0;
    while (start < combo.size()) {
        const size_t plus = combo.find('+', start);
        const std::string token = combo.substr(
            start, plus == std::string::npos ? std::string::npos : plus - start);
        if (!token.empty()) {
            applyJoyToken(msg, token, pressed);
        }
        if (plus == std::string::npos) {
            break;
        }
        start = plus + 1;
    }
    return msg;
}
#endif
