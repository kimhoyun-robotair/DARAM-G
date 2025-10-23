#!/usr/bin/env python3
import ast
import math
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from detection_msgs.msg import LineDetection


def parse_norm_points(s: str) -> List[Tuple[float, float]]:
    """
    Parse normalized polygon points from parameter string.
    Example format:
      "0.1,0.9; 0.9,0.9; 0.7,0.6; 0.3,0.6"
    or JSON-like:
      "[[0.1,0.9],[0.9,0.9],[0.7,0.6],[0.3,0.6]]"
    Returns list of (x_norm, y_norm)
    """
    s = s.strip()
    if not s:
        return []
    try:
        # Try JSON-like
        pts = ast.literal_eval(s)
        if isinstance(pts, (list, tuple)) and len(pts) > 0:
            out = []
            for p in pts:
                if isinstance(p, (list, tuple)) and len(p) == 2:
                    out.append((float(p[0]), float(p[1])))
            if out:
                return out
    except Exception:
        pass

    # Fallback: "x,y; x,y; ..."
    parts = [p.strip() for p in s.split(";") if p.strip()]
    out = []
    for p in parts:
        xy = [q.strip() for q in p.split(",")]
        if len(xy) == 2:
            out.append((float(xy[0]), float(xy[1])))
    return out


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__("lane_detector")

        # ---- Parameters ----
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("invert_lightness", True)
        self.declare_parameter("threshold", 200)
        self.declare_parameter("min_area", 500)
        self.declare_parameter(
            "mask_points_norm",
            "[[0.1,0.95],[0.9,0.95],[0.7,0.6],[0.3,0.6]]"  # lower trapezoid
        )
        self.declare_parameter("use_imshow", False)

        # Dynamic parameter update
        self.add_on_set_parameters_callback(self._on_param_update)

        # Read initial params
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.invert_lightness = self.get_parameter("invert_lightness").get_parameter_value().bool_value
        self.thresh = int(self.get_parameter("threshold").get_parameter_value().integer_value)
        self.min_area = float(self.get_parameter("min_area").get_parameter_value().double_value
                              if self.get_parameter("min_area").type_ == Parameter.Type.DOUBLE
                              else self.get_parameter("min_area").get_parameter_value().integer_value)
        self.mask_points_norm = parse_norm_points(
            self.get_parameter("mask_points_norm").get_parameter_value().string_value
        )
        self.use_imshow = self.get_parameter("use_imshow").get_parameter_value().bool_value

        # Bridge & pubs/subs
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.pub_light = self.create_publisher(Image, "/lane_detector/debug/lightness", 10)
        self.pub_light_inv = self.create_publisher(Image, "/lane_detector/debug/lightness_inverted", 10)
        self.pub_masked = self.create_publisher(Image, "/lane_detector/debug/masked", 10)
        self.pub_binary = self.create_publisher(Image, "/lane_detector/debug/binary", 10)
        self.pub_contours = self.create_publisher(Image, "/lane_detector/debug/contours", 10)

        self.pub_centroid = self.create_publisher(Point, "/lane_detector/centroid", 10)
        self.pub_line = self.create_publisher(LineDetection, "/lane_detector/line_detection", 10)


        self.get_logger().info(
            f"lane_detector started. Subscribing: {self.image_topic} | "
            f"invert_lightness={self.invert_lightness} | threshold={self.thresh} | min_area={self.min_area}"
        )

    # ---------------- Parameter callback ----------------
    def _on_param_update(self, params: List[Parameter]):
        for p in params:
            if p.name == "invert_lightness":
                self.invert_lightness = p.value
            elif p.name == "threshold":
                self.thresh = int(p.value)
            elif p.name == "min_area":
                self.min_area = float(p.value)
            elif p.name == "mask_points_norm":
                self.mask_points_norm = parse_norm_points(p.value)
            elif p.name == "use_imshow":
                self.use_imshow = p.value
        return rclpy.parameter.SetParametersResult(successful=True)

    # ----------------- Core pipeline --------------------
    def image_cb(self, msg: Image):
        # 0) Read image (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        # 1) Convert RGB->HLS and extract Lightness
        # Note: incoming is BGR; convert BGR->HLS (OpenCV's HLS = (H, L, S))
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        light = hls[:, :, 1]  # Lightness channel

        self.pub_light.publish(self.bridge.cv2_to_imgmsg(light, encoding="mono8"))

        # 2) (Optional) invert lightness to detect dark line on light background
        if self.invert_lightness:
            light_proc = cv2.bitwise_not(light)
            self.pub_light_inv.publish(self.bridge.cv2_to_imgmsg(light_proc, encoding="mono8"))
        else:
            light_proc = light

        # 3) Polygon mask to filter environment disturbances
        mask = np.zeros((h, w), dtype=np.uint8)
        if self.mask_points_norm:
            pts = np.array(
                [[int(xn * w), int(yn * h)] for (xn, yn) in self.mask_points_norm],
                dtype=np.int32
            )
        else:
            # full frame if not provided
            pts = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.int32)
        cv2.fillPoly(mask, [pts], 255)

        masked_light = cv2.bitwise_and(light_proc, light_proc, mask=mask)
        self.pub_masked.publish(self.bridge.cv2_to_imgmsg(masked_light, encoding="mono8"))

        # 4) High-pass binary threshold on lightness => light objects=255, others=0
        #    If you're detecting a dark line, set invert_lightness=True so dark becomes bright.
        _, binary = cv2.threshold(masked_light, self.thresh, 255, cv2.THRESH_BINARY)
        # Optionally clean small noise
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        self.pub_binary.publish(self.bridge.cv2_to_imgmsg(binary, encoding="mono8"))

        # 5) Find all white contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"[lane_detector] contours_found={len(contours)}")

        # 6) Pick biggest contour & centroid
        overlay = frame.copy()
        selected_centroid = None
        selected_area = 0.0

        if contours:
            # Filter by min_area
            big = [c for c in contours if cv2.contourArea(c) >= self.min_area]
            if big:
                c = max(big, key=cv2.contourArea)
                area = float(cv2.contourArea(c))
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    selected_centroid = (cx, cy)
                    selected_area = area

                    # Draw
                    cv2.drawContours(overlay, [c], -1, (0, 255, 0), 2)
                    cv2.circle(overlay, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.putText(overlay, f"centroid=({cx},{cy}) area={int(area)}",
                                (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 255), 1)
                    self.get_logger().info(
                        f"[lane_detector] SELECTED contour_area={int(area)} centroid=({cx},{cy})"
                    )
                else:
                    self.get_logger().warn("[lane_detector] moments.m00 == 0; centroid undefined")
            else:
                self.get_logger().info(
                    f"[lane_detector] no contour >= min_area({self.min_area}); nothing selected"
                )
        else:
            self.get_logger().info("[lane_detector] no contours found")

        # Publish overlay image with contours/centroid
        self.pub_contours.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

        # Publish centroid (in pixel coordinates); z carries area
        pt = Point()
        if selected_centroid is not None:
            pt.x = float(selected_centroid[0])
            pt.y = float(selected_centroid[1])
            pt.z = float(selected_area)
        else:
            pt.x = -1.0
            pt.y = -1.0
            pt.z = 0.0
        self.pub_centroid.publish(pt)

        # Optional on-screen debug
        if self.use_imshow:
            cv2.imshow("lightness", light)
            if self.invert_lightness:
                cv2.imshow("lightness_inverted", light_proc)
            cv2.imshow("masked", masked_light)
            cv2.imshow("binary", binary)
            cv2.imshow("contours", overlay)
            cv2.waitKey(1)

        ld = LineDetection()
        ld.header = msg.header  # 입력 이미지의 타임스탬프/프레임 이용
        ld.width = int(w)

        if selected_centroid is not None:
            ld.found = True
            ld.cx = float(selected_centroid[0])
        else:
            ld.found = False
            ld.cx = -1.0

        self.pub_line.publish(ld)


def main():
    rclpy.init()
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.use_imshow:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
