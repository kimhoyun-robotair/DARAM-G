#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine')

        # Create a subscriber to the detection topic
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',  # Replace with your topic name
            self.detection_callback,
            10  # Queue size
        )

        # Create a publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize state
        self.state = 'SEARCHING'  # Possible states: SEARCHING, APPROACHING, STOPPED

    def detection_callback(self, msg):
        """Callback function to process detections and update state."""
        if msg.detections:
            self.get_logger().info('Object detected, switching to APPROACHING state.')
            self.state = 'APPROACHING'
            self.approach_object()
        else:
            self.get_logger().info('No object detected, switching to SEARCHING state.')
            self.state = 'SEARCHING'
            self.search_for_object()

    def approach_object(self):
        """Logic to approach the detected object."""
        twist = Twist()
        twist.linear.x = 0.2  # Move forward
        twist.angular.z = 0.0  # No rotation
        self.publisher.publish(twist)

    def search_for_object(self):
        """Logic to search for an object."""
        twist = Twist()
        twist.linear.x = 0.0  # Stop moving forward
        twist.angular.z = 0.5  # Rotate in place
        self.publisher.publish(twist)