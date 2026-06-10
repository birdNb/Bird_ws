#pragma once

#include "Common.h"

/** 每帧更新的运行上下文，集中判定能否下发机器人侧效应 */
struct RobotOutputContext {
    bool no_joy = false;
    bool joy_blocking = false;
    bool g5_follow_active = false;
    long long ms_since_last_joy = 999999;
};

class RobotOutputGate {
public:
    void update(const RobotOutputContext& ctx) { ctx_ = ctx; }

    /** 脸跟踪脖子发布（G5 跟手时关闭） */
    bool allowFaceNeck() const { return !ctx_.g5_follow_active; }

    /** /joy_msg、/action_config、撒娇扭腰等 */
    bool allowGestureSideEffects() const {
        if (ctx_.g5_follow_active) {
            return false;
        }
        if (ctx_.no_joy) {
            return true;
        }
        if (ctx_.joy_blocking) {
            return false;
        }
        return ctx_.ms_since_last_joy >= JOY_SIDE_EFFECT_COOLDOWN_MS;
    }

    bool allowJoyMsgPublish() const { return allowGestureSideEffects(); }

    bool allowPolicyChange() const { return allowGestureSideEffects(); }

    bool allowChassisCmdVel() const {
        if (ctx_.no_joy) {
            return true;
        }
        return !ctx_.joy_blocking;
    }

private:
    RobotOutputContext ctx_;
};
