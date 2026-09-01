#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BFM 安全版 joy_mapper：手柄回中不向 /cmd_vel 发零速，避免盖掉小程序三轴方向。
由 ble/install.sh → ensure_bfm_joy_mapper.sh 安装/热替换，勿再手动启动。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from hightorque_msgs.msg import HightorqueJoy
from rclpy.node import Node
from sensor_msgs.msg import Joy

CMD_VEL_EPS = 0.02


class BeitongLayout:
    axis_l_h = 0
    axis_l_v = 1
    axis_lt = 5
    axis_r_h = 3
    axis_r_v = 2
    axis_rt = 4
    axis_dpad_h = 6
    axis_dpad_v = 7
    btn_a = 0
    btn_b = 1
    btn_x = 3
    btn_y = 4
    btn_lb = 6
    btn_rb = 7
    btn_back = 10
    btn_start = 11
    btn_center = 12
    btn_l = 13
    btn_r = 14
    trigger_rest_high = True


def _axis(msg: Joy, index: int, scale: float = 1.0) -> float:
    if index >= len(msg.axes):
        return 0.0
    return float(msg.axes[index]) * scale


def _button(msg: Joy, index: int) -> float:
    if index >= len(msg.buttons):
        return 0.0
    return float(msg.buttons[index])


class JoyMapperBfmFix(Node):
    def __init__(self) -> None:
        super().__init__("joy_mapper_node")
        self._layout = BeitongLayout()
        self.create_subscription(Joy, "joy", self._on_joy, 10)
        self._walk_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # launch remap: joy_msg → hightorque_joy；热替换无 remap 时直接发 hightorque_joy
        self._mode_pub = self.create_publisher(HightorqueJoy, "joy_msg", 10)
        self._mode_pub_ht = self.create_publisher(HightorqueJoy, "hightorque_joy", 10)
        self.get_logger().info(
            "joy_mapper_bfm_fix：回中不发零速，保留 BLE BFM 三轴 /cmd_vel"
        )

    def _normalize_trigger(self, raw: float) -> float:
        if raw < -0.5:
            return raw
        if self._layout.trigger_rest_high:
            if raw > 0.75:
                return 0.0
            if raw < 0.5:
                return -1.0
            return 0.0
        if raw > 0.5:
            return -1.0
        return 0.0

    def _on_joy(self, msg: Joy) -> None:
        lay = self._layout
        lt_raw = _axis(msg, lay.axis_lt)
        rt_raw = _axis(msg, lay.axis_rt)
        lt_pressed = self._normalize_trigger(lt_raw) < -0.5
        rt_pressed = self._normalize_trigger(rt_raw) < -0.5
        speed_boost = _button(msg, lay.btn_rb) > 0.5 and not lt_pressed and not rt_pressed
        speed_scale = 2.0 if speed_boost else 1.0

        twist = Twist()
        twist.angular.z = _axis(msg, lay.axis_r_h, 1.57 * speed_scale)
        twist.linear.x = _axis(msg, lay.axis_l_v, 1.5 * speed_scale)
        twist.linear.y = _axis(msg, lay.axis_l_h, 0.7 * speed_scale)
        if (
            abs(twist.linear.x) >= CMD_VEL_EPS
            or abs(twist.linear.y) >= CMD_VEL_EPS
            or abs(twist.angular.z) >= CMD_VEL_EPS
        ):
            self._walk_pub.publish(twist)

        joy = HightorqueJoy()
        joy.l_horizontal = _axis(msg, lay.axis_l_h)
        joy.l_vertical = _axis(msg, lay.axis_l_v)
        joy.lt = self._normalize_trigger(lt_raw)
        joy.r_horizontal = _axis(msg, lay.axis_r_h)
        joy.r_vertical = _axis(msg, lay.axis_r_v)
        joy.rt = self._normalize_trigger(rt_raw)
        joy.dpad_horizontal = _axis(msg, lay.axis_dpad_h)
        joy.dpad_vertical = _axis(msg, lay.axis_dpad_v)
        joy.a = _button(msg, lay.btn_a)
        joy.b = _button(msg, lay.btn_b)
        joy.x = _button(msg, lay.btn_x)
        joy.y = _button(msg, lay.btn_y)
        joy.lb = _button(msg, lay.btn_lb)
        joy.rb = _button(msg, lay.btn_rb)
        joy.back = _button(msg, lay.btn_back)
        joy.start = _button(msg, lay.btn_start)
        joy.center = _button(msg, lay.btn_center)
        joy.l = _button(msg, lay.btn_l)
        joy.r = _button(msg, lay.btn_r)
        self._mode_pub.publish(joy)
        self._mode_pub_ht.publish(joy)


def main() -> None:
    rclpy.init()
    node = JoyMapperBfmFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
