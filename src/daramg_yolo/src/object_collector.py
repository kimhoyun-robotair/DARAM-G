#!/usr/bin/env python3

# Non-ROS Library
import cv2
import numpy as np
import random

# ROS2 Library
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from detection_msgs.msg import BoundingBox, BoundingBoxes

class ObjectCollector(Node):
    def __init__(self):
        super().__init__("object_collector")
        
        # 파라미터 선언
        self.declare_parameter('image_width', 1080)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('depth_stop_threshold', 0.1) # 로봇이 멈추는 거리
        self.declare_parameter('fov_deg', 60.0) # 카메라의 시야각도
        self.declare_parameter('Kp_ang', 0.002) # 각속도 제어 이득
        self.declare_parameter('Kp_lin', 0.5) # 선형 속도 제어 이득

        # 파라미터 값들 가져오기
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.depth_stop_threshold = float(self.get_parameter('depth_stop_threshold').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.Kp_ang = float(self.get_parameter('Kp_ang').value)
        self.Kp_lin = float(self.get_parameter('Kp_lin').value)

        # publisher and subscirber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bbox_sub = self.create_subscription(BoundingBoxes, 'yolo_bboxes', self.bbox_callback, 10)

        self.target_detected = False # 객체가 탐지되었는지 여부
        self.prev_error_x = 0.0 # 이전 위치 오차 값
        self.cmd_vel = Twist() # 속도 토픽
        self.latest_detection_time = None # 미탐지 시간 계산을 위한 변수 선언

        self.timer = self.create_timer(0.2, self.controll_loop) # 타이머 선언으로 제어 루프 시작하기

        self.get_logger().info("Object Collector Node Configured!")
    
    def bbox_callback(self, msg:BoundingBoxes):
        if not msg.bounding_boxes:
            self.get_logger().warn("No Object Detected")
            self.stop_robot()
            self.target_detected = False
            return
        
        # 가장 최근에 탐지된 시간 기록
        self.latest_detection_time = self.get_clock().now()
        self.target_detected = True

        # 실제 하드웨어에서 사용할 코드에서는 사용해보기
        #valid_boxes = [b for b in msg.bounding_boxes if b.yoloclass in ['bottle', 'cup']]
        #if not valid_boxes:
        #    self.stop_robot()
        #    return
        #nearest_bbox = min(valid_boxes, key=lambda b: b.depth if b.depth > 0 else float('inf'))

        # 가장 가까운 객체를 선택하기
        nearest_bbox = min(msg.bounding_boxes, key=lambda b: b.depth if b.depth > 0 else float('inf'))
        target_depth = nearest_bbox.depth

        # depth값이 비정상적 수치를 기록할 경우 무시
        if target_depth <=0 or np.isnan(target_depth) or np.isinf(target_depth):
            self.get_logger().warn("Invalid Depth Value Deteced.")
            return

        # 가장 가까운 객체까지의 오프셋 계산 -> 각도 및 선속도 제어를 위한 값 산출
        cx = nearest_bbox.center_x
        offset_x = cx - (self.image_width / 2.0)
        angle_error = (offset_x / self.image_width)*self.fov_deg
        ang_z = -self.Kp_ang * angle_error
        lin_x = self.Kp_lin * target_depth if target_depth > 0 else 0.0

        # 임계 거리 이하로 접근하게 될 경우 로봇의 이동 중지
        # margin을 추가해서 좀 더 안정적인 제어
        self.stop_margin = 0.05
        if 0 < target_depth < self.depth_stop_threshold:
            self.get_logger().info(f"Target within stopping distance: {target_depth:.2f}m, Stop!")
            self.stop_robot()
            return
        elif target_depth <= self.depth_stop_threshold + self.stop_margin:
            lin_x *= 0.5
        
        # 속도 제한 걸기
        ang_z = max(-0.5, min(0.5, ang_z))
        lin_x = min(0.3, max(0.0, lin_x))

        # 속도 값 topic으로 publish
        self.cmd_vel.linear.x = lin_x
        self.cmd_vel.angular.z = ang_z

        self.get_logger().debug(f"Target Depth: {target_depth:.2f}m, Lin_x: {lin_x:.2f}, Ang_z: {ang_z:.2f}")

    def stop_robot(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel = stop_cmd
        self.cmd_pub.publish(stop_cmd)

    def controll_loop(self):
        # latest_detection_time이 아직 초기화되지 않은 경우 예외 처리
        if self.latest_detection_time is None:
            self.stop_robot()
            return

        if (self.get_clock().now() - self.latest_detection_time).nanoseconds > 2e9:
            self.get_logger().warn("No recent detection, stopping robot.")
            self.stop_robot()
            self.cmd_vel = Twist()
            self.target_detected = False
            return

        self.cmd_pub.publish(self.cmd_vel)

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