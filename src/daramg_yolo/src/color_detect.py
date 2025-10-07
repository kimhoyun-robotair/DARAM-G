#!/usr/bin/env python3

# Non-ROS Library
import cv2, numpy as np
import threading

# ROS Library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # publisher and subscriber
        