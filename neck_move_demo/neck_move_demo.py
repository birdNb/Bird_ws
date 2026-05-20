#!/usr/bin/env python3
# ==============================================
# 脖子运动 Demo（sim2real 控制接口）
# 流程： UP -> CENTER -> DOWN -> CENTER -> LEFT -> CENTER -> RIGHT -> CENTER -> 循环
# 控制方式：发布 sensor_msgs/JointState 到绝对位置话题（默认 /pi_plus_absolute）
# 关节：head_yaw_joint (左右), head_pitch_joint (上下)
#
# 调参：修改文件顶部 "可调参数" 区域，无需改主循环逻辑
# 退出：Ctrl+C，会自动发送回中指令后退出
# ==============================================

import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# ===================== 可调参数 =====================
# ROS 话题（绝对位置控制，sim2real 标准接口）
ABSOLUTE_TOPIC = "/pi_plus_absolute"

# 头部关节名（必须与机器人 pd.yaml / mtr_state 中一致）
HEAD_YAW_JOINT = "head_yaw_joint"      # 左右
HEAD_PITCH_JOINT = "head_pitch_joint"  # 上下

# ----- 运动幅度（单位：度 deg；脚本内部自动转弧度） -----
# pd.yaml 软件限位：yaw / pitch 均为 ±1.58 rad ≈ ±90.5°
# pitch 实际不对称（低头比抬头空间大）：抬头 -45°，低头 +75°
PITCH_UP_DEG = -45.0     # 抬头幅度（度）
PITCH_DOWN_DEG = 75.0    # 低头幅度（度）
YAW_LEFT_DEG = 120.0      # 向左转幅度（度）
YAW_RIGHT_DEG = -120.0    # 向右转幅度（度）

# 内部使用的弧度值（不要直接改这几个）
PITCH_UP = math.radians(PITCH_UP_DEG)
PITCH_DOWN = math.radians(PITCH_DOWN_DEG)
YAW_LEFT = math.radians(YAW_LEFT_DEG)
YAW_RIGHT = math.radians(YAW_RIGHT_DEG)

# ----- 时间参数（秒） -----
# 每段平滑运动的时长（从当前位置 -> 目标位置，正弦速度曲线）
# 全自由度范围下行程更大（90°），运动放慢以便观察干涉点
MOVE_TO_EXTREME_SEC = 2.0   # 从中位移动到极限位置的时长
MOVE_TO_CENTER_SEC = 1.5    # 从极限位置回到中位的时长（行程相同，时间略短）
DWELL_SECONDS = 0.5         # 到达目标后保持时长（便于观察是否有干涉/堵转）
INIT_CENTER_SECONDS = 1.2   # 启动时先回中的时长

# ----- 控制频率 -----
# 官方文档推荐 50~200Hz；这里特意降到 20Hz 用于验证抖动是否由本地 publish
# 抖动 / 频繁刷新激发 -> 若 20Hz 明显变好，说明高频抖来自频繁更新本身
PUBLISH_RATE_HZ = 50

# ----- 速度前馈 -----
# True: 计算目标速度并填入 JointState.velocity 字段。
#       sim2real 把 velocity 当 target_dq 使用 -> 做 velocity feedforward，
#       消除跟踪滞后引起的高频震荡。
# False: 只发位置，控制器靠 PD 自己跟踪（容易抖）
ENABLE_VEL_FEEDFORWARD = True

# ----- 轨迹平滑曲线 -----
# "quintic":    5 阶多项式 (10t^3 - 15t^4 + 6t^5)  起止位置/速度/加速度均为0
#                -> 无加速度阶跃，运动最平顺，推荐
# "cosine":     余弦平滑    0.5*(1-cos(pi*t/T))     起止位置/速度=0 但加速度有跳变
#                -> 较好，可能在起止时刻激发高频抖动
TRAJECTORY_PROFILE = "quintic"

# ----- 等待连接（秒） -----
WAIT_SUBSCRIBER_TIMEOUT = 3.0

# ----- FSM 状态守门 -----
# sim2real master 把当前 FSM 状态用 std_msgs/Int32 发布到 /fsm_state
# 枚举值(见 fsm.h FsmNodeType):
#   0=INIT, 1=ERROR, 2~4=CANDIDATE_*, 5=EXEC_DEFAULT, 6=EXEC_CUSTOM,
#   7=EXEC_REMOTE, 8=PROTECTION_SHUTDOWN, 9~12=校零, 13~14=示教,
#   15~16=DEVELOP
# 只有 EXEC_DEFAULT(5) 状态下，DefaultControllerInterface 才被实例化，
# /pi_plus_absolute 才会被订阅并下发到电机 PD。
FSM_STATE_TOPIC = "/fsm_state"
FSM_EXEC_DEFAULT = 5
# 是否启用 FSM 状态守门(未进 ExecDefault 不动作；运动中状态变化立即停)
FSM_GATE_ENABLED = True
# 启动时等待进入 ExecDefault 的最长时间(秒)；超时仍打印提示并继续等
FSM_WAIT_TIMEOUT = 30.0
# =====================================================


class FsmStateMonitor:
    """订阅 /fsm_state 并维护最新值，线程安全。"""

    def __init__(self, topic: str = FSM_STATE_TOPIC):
        self._lock = threading.Lock()
        self._state = None  # None = 还没收到过
        self._sub = rospy.Subscriber(topic, Int32, self._cb, queue_size=10)

    def _cb(self, msg: Int32) -> None:
        with self._lock:
            self._state = int(msg.data)

    @property
    def state(self):
        with self._lock:
            return self._state

    @staticmethod
    def state_name(v) -> str:
        return {
            0: "INIT", 1: "ERROR",
            2: "CANDIDATE_DEFAULT", 3: "CANDIDATE_CUSTOM",
            4: "CANDIDATE_REMOTE",
            5: "EXEC_DEFAULT", 6: "EXEC_CUSTOM", 7: "EXEC_REMOTE",
            8: "PROTECTION_SHUTDOWN",
            9: "CANDIDATE_CALIBRATION", 10: "EXEC_CALIBRATING",
            11: "EXEC_CALIB_OK", 12: "EXEC_CALIB_FAILED",
            13: "CANDIDATE_TEACHING", 14: "EXEC_TEACHING",
            15: "CANDIDATE_DEVELOP", 16: "EXEC_DEVELOP",
        }.get(v, f"UNKNOWN({v})")

    def wait_for_exec_default(self, timeout: float = FSM_WAIT_TIMEOUT) -> bool:
        """阻塞等待直到 state == 5 (EXEC_DEFAULT)。

        每秒打印一次当前状态提示。timeout 内未达成返回 False 但仍继续等。
        """
        deadline = time.time() + timeout
        last_log_t = 0.0
        warned_timeout = False
        while not rospy.is_shutdown():
            s = self.state
            if s == FSM_EXEC_DEFAULT:
                return True
            now = time.time()
            if now - last_log_t >= 1.0:
                if s is None:
                    rospy.logwarn(
                        "[FSM] 还未收到 %s, 请确认 sim2real_master_node 已启动",
                        FSM_STATE_TOPIC,
                    )
                else:
                    rospy.logwarn(
                        "[FSM] 当前状态 %s(%d) != EXEC_DEFAULT(5)；"
                        "请按手柄 Start 键进入 default 模式",
                        self.state_name(s), s,
                    )
                last_log_t = now
            if not warned_timeout and now > deadline:
                rospy.logerr(
                    "[FSM] 等待 %.0fs 仍未进入 EXEC_DEFAULT，继续等待中...",
                    timeout,
                )
                warned_timeout = True
            time.sleep(0.1)
        return False


def make_msg(yaw: float, pitch: float,
             yaw_dot: float = 0.0, pitch_dot: float = 0.0) -> JointState:
    """构造一条 JointState 控制消息（只控两个头部关节，其余电机不受影响）。

    若 ENABLE_VEL_FEEDFORWARD=True，则把目标速度填入 velocity 字段，
    sim2real master 会把它作为 target_dq 用于 velocity feedforward。
    """
    msg = JointState()
    msg.name = [HEAD_YAW_JOINT, HEAD_PITCH_JOINT]
    msg.position = [yaw, pitch]
    if ENABLE_VEL_FEEDFORWARD:
        msg.velocity = [yaw_dot, pitch_dot]
    else:
        msg.velocity = []
    msg.effort = []
    return msg


def publish_hold(pub: rospy.Publisher, rate: rospy.Rate,
                 yaw: float, pitch: float, duration: float) -> None:
    """在目标位置上保持 duration 秒，按 PUBLISH_RATE_HZ 持续发布同一个目标。

    sim2real 控制器需要持续接收目标位置而不是只发一次，否则可能被判定为信号丢失
    或被其他节点覆盖。
    """
    msg = make_msg(yaw, pitch)
    end_t = rospy.Time.now() + rospy.Duration.from_sec(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end_t:
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


def _profile(tau: float, T: float):
    """轨迹归一化曲线：返回 (s, ds_dt)，s/ds_dt 都对应 [0, 1] 行程。

    quintic（minimum-jerk）:
        s   = 10*τ^3 - 15*τ^4 + 6*τ^5
        s'  = (30*τ^2 - 60*τ^3 + 30*τ^4) / T
            = 30*τ^2*(1-τ)^2 / T
        起止位置/速度/加速度 = 0  -> 无加速度阶跃，平顺度最优
    cosine（余弦插值）:
        s   = 0.5*(1 - cos(π*τ))
        s'  = 0.5*(π/T)*sin(π*τ)
        起止位置/速度 = 0, 但加速度起止 = 0.5*ω^2 ≠ 0（阶跃）
    """
    if TRAJECTORY_PROFILE == "cosine":
        omega = math.pi / T
        s = 0.5 * (1.0 - math.cos(math.pi * tau))
        ds_dt = 0.5 * omega * math.sin(math.pi * tau)
        return s, ds_dt
    # 默认: quintic
    t2 = tau * tau
    t3 = t2 * tau
    t4 = t3 * tau
    t5 = t4 * tau
    s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
    ds_dt = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / T
    return s, ds_dt


def move_smooth(pub: rospy.Publisher, rate: rospy.Rate,
                start_yaw: float, start_pitch: float,
                end_yaw: float, end_pitch: float,
                duration: float) -> None:
    """从 start 平滑插值到 end，使用 TRAJECTORY_PROFILE 指定的曲线。

    - 同时把解析位置和解析速度都发出去，sim2real master 会把 velocity
      作为 target_dq 用于 velocity feedforward，消除跟踪滞后引起的高频抖动
    - 默认 quintic 五阶多项式，起止加速度=0（比 cosine 更平顺）

    若 duration <= 0 或 start == end，则等效为 hold 0 秒（直接返回）。
    """
    if duration <= 1e-6:
        return
    dy = end_yaw - start_yaw
    dp = end_pitch - start_pitch
    t0 = rospy.Time.now()
    end_t = t0 + rospy.Duration.from_sec(duration)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now >= end_t:
            break
        elapsed = (now - t0).to_sec()
        tau = elapsed / duration  # 归一化时间 [0, 1]
        s, ds = _profile(tau, duration)
        yaw = start_yaw + dy * s
        pitch = start_pitch + dp * s
        yaw_dot = dy * ds
        pitch_dot = dp * ds
        msg = make_msg(yaw, pitch, yaw_dot, pitch_dot)
        msg.header.stamp = now
        pub.publish(msg)
        rate.sleep()
    # 显式把最终精确位置再发一次（速度=0），避免循环最后一拍提前结束导致小偏差
    msg = make_msg(end_yaw, end_pitch, 0.0, 0.0)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)


def main() -> None:
    rospy.init_node("neck_move_demo", anonymous=False)
    pub = rospy.Publisher(ABSOLUTE_TOPIC, JointState, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE_HZ)

    # 退出时尽量回中（即使 Ctrl+C / 节点崩溃也补几次回中指令）
    def on_shutdown() -> None:
        rospy.logwarn("[neck_move_demo] shutting down -> 回中")
        msg = make_msg(0.0, 0.0)
        for _ in range(20):  # ~0.4s 持续发送
            msg.header.stamp = rospy.Time.now()
            try:
                pub.publish(msg)
            except Exception:
                break
            time.sleep(0.02)

    rospy.on_shutdown(on_shutdown)

    # 等待至少一个订阅者连上（master_node），避免最初几条消息直接掉
    t0 = time.time()
    while (pub.get_num_connections() == 0
           and time.time() - t0 < WAIT_SUBSCRIBER_TIMEOUT
           and not rospy.is_shutdown()):
        time.sleep(0.1)

    if pub.get_num_connections() == 0:
        rospy.logwarn(
            "[neck_move_demo] %s 上还没有订阅者（master_node 可能未启动），"
            "仍然继续发布，但电机不会响应。", ABSOLUTE_TOPIC,
        )
    else:
        rospy.loginfo(
            "[neck_move_demo] %d 个订阅者已连接到 %s",
            pub.get_num_connections(), ABSOLUTE_TOPIC,
        )

    rospy.loginfo(
        "幅度 yaw=[%+.1f°, %+.1f°] pitch=[%+.1f°, %+.1f°]  "
        "move(to_ext=%.2fs, to_ctr=%.2fs)  dwell=%.2fs  rate=%dHz",
        YAW_RIGHT_DEG, YAW_LEFT_DEG, PITCH_UP_DEG, PITCH_DOWN_DEG,
        MOVE_TO_EXTREME_SEC, MOVE_TO_CENTER_SEC, DWELL_SECONDS,
        PUBLISH_RATE_HZ,
    )
    rospy.loginfo("Ctrl+C 退出（自动回中）")

    # ===== FSM 状态守门员 =====
    # 启动 /fsm_state 监听，未进 EXEC_DEFAULT(5) 不下发任何运动指令，
    # 避免在 INIT(发 PD=0 -> 抖动)/PROTECTION_SHUTDOWN/校零等状态下乱发命令
    fsm = FsmStateMonitor() if FSM_GATE_ENABLED else None
    if fsm is not None:
        rospy.loginfo("FSM 守门已启用，等待 EXEC_DEFAULT(5)...")
        fsm.wait_for_exec_default(FSM_WAIT_TIMEOUT)
        rospy.loginfo(
            "[FSM] 进入 EXEC_DEFAULT(%d)，开始运动",
            FSM_EXEC_DEFAULT,
        )

    # 启动先平稳回中（这里 start==end，等价于 hold）
    rospy.loginfo("-> INIT CENTER")
    publish_hold(pub, rate, 0.0, 0.0, INIT_CENTER_SECONDS)

    # 跟踪"当前已经到达"的目标位置（用于下一段插值的起点）
    cur_yaw, cur_pitch = 0.0, 0.0

    # 序列项: (描述, 目标yaw, 目标pitch, 该段移动时长)
    loop_idx = 0
    while not rospy.is_shutdown():
        loop_idx += 1
        sequence = [
            ("UP    ", 0.0,        PITCH_UP,   MOVE_TO_EXTREME_SEC),
            ("CENTER", 0.0,        0.0,        MOVE_TO_CENTER_SEC),
            ("DOWN  ", 0.0,        PITCH_DOWN, MOVE_TO_EXTREME_SEC),
            ("CENTER", 0.0,        0.0,        MOVE_TO_CENTER_SEC),
            ("LEFT  ", YAW_LEFT,   0.0,        MOVE_TO_EXTREME_SEC),
            ("CENTER", 0.0,        0.0,        MOVE_TO_CENTER_SEC),
            ("RIGHT ", YAW_RIGHT,  0.0,        MOVE_TO_EXTREME_SEC),
            ("CENTER", 0.0,        0.0,        MOVE_TO_CENTER_SEC),
        ]
        rospy.loginfo("===== loop #%d =====", loop_idx)
        for desc, tgt_yaw, tgt_pitch, move_dur in sequence:
            if rospy.is_shutdown():
                break
            # 每段动作前再确认一下 FSM 状态(防中途切换到非 ExecDefault)
            if fsm is not None and fsm.state != FSM_EXEC_DEFAULT:
                rospy.logwarn(
                    "[FSM] 状态变化为 %s(%s)，运动暂停",
                    FsmStateMonitor.state_name(fsm.state), fsm.state,
                )
                fsm.wait_for_exec_default(FSM_WAIT_TIMEOUT)
                rospy.loginfo("[FSM] 恢复到 EXEC_DEFAULT，继续运动")
                # 状态恢复后重新回中再继续序列，避免位置突变
                publish_hold(pub, rate, 0.0, 0.0, INIT_CENTER_SECONDS)
                cur_yaw, cur_pitch = 0.0, 0.0
            rospy.loginfo(
                "  -> %s  yaw=%+6.1f°  pitch=%+6.1f°  move=%.2fs",
                desc, math.degrees(tgt_yaw), math.degrees(tgt_pitch),
                move_dur,
            )
            move_smooth(
                pub, rate, cur_yaw, cur_pitch, tgt_yaw, tgt_pitch, move_dur,
            )
            # 到位后做一个短暂停留，便于观察极限位置（设 0 可关闭）
            if DWELL_SECONDS > 0.0:
                publish_hold(pub, rate, tgt_yaw, tgt_pitch, DWELL_SECONDS)
            cur_yaw, cur_pitch = tgt_yaw, tgt_pitch


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
