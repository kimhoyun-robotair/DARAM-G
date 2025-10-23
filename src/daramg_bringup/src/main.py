#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import DetectionArray
from detection_msgs.msg import LineDetection, ColorBoundingBoxes
from std_msgs.msg import Bool
from utility import (compute_cmd_vel_from_line, compute_cmd_vel_from_YOLO, 
                    YoloConfig, LineConfig, moving_forward, turning_standby, stop_robot)
from enum import Enum, auto

class State(Enum):
    SEARCH = auto()
    GRIP = auto()
    RETURN = auto()
    UNGRIP = auto()

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__("finite_state_machine")

        # subscriber
        self.yolo_detect_sub = self.create_subscription(
            DetectionArray,
            "/yolo/detections_3d",
            self.yolo_cb,
            10
        )

        self.line_detect_sub = self.create_subscription(
            LineDetection,
            '/lane_detector/line_detection',
            self.line_cb,
            10
        )

        self.color_detect_sub = self.create_subscription(
            ColorBoundingBoxes,
            '/color_bboxes',
            self.color_cb,
            10
        )

        # publisher
        self.cmdvel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.process_state)
        self.trash_count : int = 0

        # publisher topic declare
        self.twist = Twist()
    
        # msg variables for callback function
        self.yolo_info = None
        self.line_info = None
        self.color_info = None

        # bool var for judging target detect or not
        self.target_detect = False
        self.line_detect = False
        self.color_detect = False

        # state variable
        self.state = State.SEARCH
        self.using_yolo = True
        self.using_line = False
        self.using_color = False

        # config for YOLO
        self.yolo_config = YoloConfig(image_width=640, image_height=480,
                                      fov_deg=60.0, kp_ang=0.002, kp_lin=0.5,
                                      depth_stop_threshold=0.1, stop_margin=0.05,
                                      lin_max=0.3, ang_max=0.5)
        
        # config for Line Detection
        self.line_config = LineConfig(deadband_px=20, v_turn=0.05, v_straight=0.1,
                                      w_turn=0.15, lin_max=0.3, ang_max=0.5)
        
        # config for Color Detection
        pass


    def yolo_cb(self, msg : DetectionArray):
        self.yolo_info = msg
        if (msg is not None) and (hasattr(msg.detections, "class_id")):
            self.target_detect = True

    def line_cb(self, msg : LineDetection):
        self.line_info = msg
        if (msg is not None) and (hasattr(msg, "found")):
            self.line_detect = True
    
    def color_cb(self, msg : ColorBoundingBoxes):
        self.color_info = msg
        if (msg is not None) and (hasattr(msg.color_bounding_boxes, "string")):
            self.line_detect = True        

    def process_state(self):
        while rclpy.ok():
            if self.trash_count == 10:
                break

            
def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__=="__main__":
    main()