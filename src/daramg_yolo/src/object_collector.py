#!/usr/bin/env python3

# Non-ROS Library
import cv2
import numpy as np
import random

# ROS2 Library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from detection_msgs.msg import BoundingBox, BoundingBoxes

class ObjectCollector(Node):
    def __init__(self):
        super().__init__("object_collector")

        