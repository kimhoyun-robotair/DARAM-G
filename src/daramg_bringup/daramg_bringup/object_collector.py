#!/usr/bin/env python3

# Non-ROS Library
import numpy as np

# ROS2 Library
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import Detection, DetectionArray


class ObjectCollector(Node):
    def __init__(self):
        super().__init__("object_collector")

        # 파라미터 선언
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('depth_stop_threshold', 0.1)  # 로봇이 안전하게 객체 앞에 멈추는 거리
        self.declare_parameter('fov_deg', 60.0)               # 카메라 시야각
        self.declare_parameter('Kp_ang', 0.002)               # 각속도 제어 이득
        self.declare_parameter('Kp_lin', 0.5)                 # 선속도 제어 이득

        # 파라미터 읽어오기
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.depth_stop_threshold = float(self.get_parameter('depth_stop_threshold').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.Kp_ang = float(self.get_parameter('Kp_ang').value)
        self.Kp_lin = float(self.get_parameter('Kp_lin').value)

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.det_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detection_callback,
            10
        )

        # 내부 변수들 초기화
        self.target_detected = False # target이 잡혔는지 트리거
        self.latest_detection_time = None # 마지막으로 객체가 탐지된 시간
        self.cmd_vel = Twist() # 로봇 구동에 필요한 속도 출력

        # subscription 콜백과 별개로 제어 콜백 함수 구현
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("Object Collector Node Configured!")


    def detection_callback(self, msg: DetectionArray):
        """YOLO 3D 탐지 결과 콜백"""
        if not msg.detections:
            self.get_logger().warn("No Object Detected")
            self.stop_robot()
            self.target_detected = False
            return

        # 최신 탐지 시각 갱신
        self.latest_detection_time = self.get_clock().now()
        self.target_detected = True

        # 가장 가까운 객체 선정
        valid_detections = [det for det in msg.detections if det.depth > 0.0]
        if not valid_detections:
            self.get_logger().warn("No valid depth detected")
            self.stop_robot()
            return

        nearest_det = min(valid_detections, key=lambda d: d.depth)
        target_depth = nearest_det.depth

        # 해당 객체의 유효성 검사
        if target_depth <= 0 or np.isnan(target_depth) or np.isinf(target_depth):
            self.get_logger().warn("Invalid depth value detected.")
            return

        # 시야 중심으로부터 오프셋 계산하기
        cx = nearest_det.bbox.center.position.x
        offset_x = cx - (self.image_width / 2.0)
        angle_error = (offset_x / self.image_width) * self.fov_deg

        # PID 기반 제어 구현 -> 일단 P 제어기만
        ang_z = -self.Kp_ang * angle_error
        lin_x = self.Kp_lin * target_depth if target_depth > 0 else 0.0

        # 안전 정지 및 감속 조건
        stop_margin = 0.05
        if 0 < target_depth < self.depth_stop_threshold:
            self.get_logger().info(f"Target within stopping distance: {target_depth:.2f} m → STOP")
            self.stop_robot()
            return
        elif target_depth <= self.depth_stop_threshold + stop_margin:
            lin_x *= 0.5

        # 최대 속도 제한
        ang_z = np.clip(ang_z, -0.5, 0.5)
        lin_x = np.clip(lin_x, 0.0, 0.3)

        # 속도 값 publish
        self.cmd_vel.linear.x = lin_x
        self.cmd_vel.angular.z = ang_z
        self.cmd_pub.publish(self.cmd_vel)

        self.get_logger().debug(
            f"Target Depth: {target_depth:.2f} m | Lin_x: {lin_x:.2f} | Ang_z: {ang_z:.2f}"
        )


    def stop_robot(self):
        # 로봇 정지시키는 함수
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        self.cmd_vel = stop_cmd


    def control_loop(self):
        # 탐지 갱신이 없을 때 로봇 자동 정지
        if self.latest_detection_time is None:
            self.stop_robot()
            return

        # 2초 이상 탐지 안될 경우 정지
        if (self.get_clock().now() - self.latest_detection_time).nanoseconds > 2e9:
            self.get_logger().warn("No recent detection — stopping robot.")
            self.stop_robot()
            self.target_detected = False
            return

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
