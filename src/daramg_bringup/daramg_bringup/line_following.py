#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist


class DriveFollowLine(Node):
    """
    /line_detector/line_found (Bool)
    /line_detector/line_offset (Float32, -1.0~+1.0)
    를 받아 단순 P제어로 라인을 따라 전진한다.
    """
    def __init__(self):
        super().__init__('drive_follow_line')

        # ---- Parameters ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.20)  # m/s
        self.declare_parameter('k_ang', 0.9)           # yaw P게인

        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, QoSProfile(depth=10))

        self.sub_found  = self.create_subscription(Bool,    '/line_detector/line_found',  self.found_cb,  10)
        self.sub_offset = self.create_subscription(Float32, '/line_detector/line_offset', self.offset_cb, 10)

        self.line_found = False
        self.offset     = 0.0

        # 20 Hz 제어
        self.timer = self.create_timer(0.05, self.control)
        self.get_logger().info("[line_following] Following line with simple P control.")

    def found_cb(self, msg: Bool):
        self.line_found = bool(msg.data)

    def offset_cb(self, msg: Float32):
        self.offset = float(msg.data)  # -1.0(좌) ~ +1.0(우)

    def control(self):
        twist = Twist()
        v = float(self.get_parameter('forward_speed').get_parameter_value().double_value)
        k = float(self.get_parameter('k_ang').get_parameter_value().double_value)

        if self.line_found:
            twist.linear.x  = v
            twist.angular.z = -k * self.offset  # 좌(-)면 +z 회전 → 중앙 복귀
        else:
            # 라인이 안 보이면 정지 (원하면 아래 한 줄을 주석 해제해서 천천히 직진)
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            # twist.linear.x = 0.05  # <- 라인 유실 시 천천히 직진하려면 이 줄 사용

        self.pub_cmd.publish(twist)


def main():
    rclpy.init()
    node = DriveFollowLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 정지
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
