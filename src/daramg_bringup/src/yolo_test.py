#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist
from utility import compute_cmd_vel_from_YOLO, YoloConfig, DetectionLite

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine')

        # Create a subscriber to the detection topic
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',  # Replace with your topic name
            self.detection_callback,
            5  # Queue size
        )

        # Create a publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.yolo_config = YoloConfig(image_width=640, image_height=480,
                                      fov_deg=60.0, kp_ang=0.002, kp_lin=0.5,
                                      depth_stop_threshold=0.1, stop_margin=0.05,
                                      lin_max=0.3, ang_max=0.5)

    def detection_callback(self, msg):
        """Callback function to process detections and update state."""
        self.lin_x, self.ang_z, self.yolo_info = compute_cmd_vel_from_YOLO(detections=msg, 
                                                                           config=self.yolo_config)
        self.twist = Twist()
        self.twist.linear.x = self.lin_x
        self.twist.angular.z = self.ang_z
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachine()
    
    try:
        rclpy.spin(node)  # Run the display loop
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()