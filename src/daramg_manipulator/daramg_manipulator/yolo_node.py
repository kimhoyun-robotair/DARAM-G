#!/usr/bin/env python3
import os
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from daramg_msgs.msg import Detection, YoloDetection
import cv2

class YoloObbNode(Node):
    def __init__(self):
        super().__init__('yolo_node_vehicle')

        # Parameter
        self.declare_parameter('model_path', os.path.join(os.environ['HOME']+
            '/DARAM-G/src/daramg_manipulator/src/best.pt'))
        self.declare_parameter('conf', 0.70)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('draw_score', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').value)
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.draw_score = bool(self.get_parameter('draw_score').value)

        # YOLO Model
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        self.bridge = CvBridge()

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Pub, Sub
        self.sub = self.create_subscription(Image, image_topic, self.cb_image, 10)
        self.pub_infer = self.create_publisher(YoloDetection, '/yolo_detection', qos_sensor)
        self.pub_img  = self.create_publisher(Image, '/yolo_image', qos_sensor)

        self.get_logger().info(f'Configuration Completed')

    def cb_image(self, msg: Image):
        # Convert ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        # Run inference
        try:
            results = self.model(frame, conf=self.conf)
        except Exception as e:
            self.get_logger().warn(f'YOLO inference failed: {e}')
            return

        # Prepare inference message
        out = YoloDetection()
        out.header = msg.header

        # Copy frame for OpenCV drawing
        vis = frame.copy()

        any_drawn = False
        for r in results:
            boxes = getattr(r, 'boxes', None)
            if boxes is None or len(boxes) == 0:
                continue

            for b in boxes:
                try:
                    # axis-aligned box: x1,y1,x2,y2  (float)
                    x1, y1, x2, y2 = b.xyxy[0].to('cpu').detach().numpy().tolist()

                    # ----- for message: float64 coords (8 values)
                    pts_float64 = np.array([[x1, y1],
                                            [x2, y1],
                                            [x2, y2],
                                            [x1, y2]], dtype=np.float64)
                    coords_list = [float(v) for v in pts_float64.reshape(-1)]  # <-- 핵심

                    # ----- for drawing: int coords
                    pts_int = pts_float64.astype(np.int32)

                    cls_id = int(b.cls.item()) if hasattr(b.cls, 'item') else int(b.cls)
                    cls_name = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)
                    score = float(b.conf.item()) if hasattr(b, 'conf') else None

                    # publish message element (float64[])
                    ir = Detection()
                    ir.yolo_class_name = cls_name
                    ir.coordinates = copy.copy(coords_list)  # list of python floats
                    out.yolo_detection.append(ir)

                    # draw polygon (rectangle as polyline)
                    cv2.polylines(vis, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)

                    # center dot
                    cx = int((x1 + x2) / 2.0)
                    cy = int((y1 + y2) / 2.0)
                    cv2.circle(vis, (cx, cy), 2, (0, 255, 0), -1)

                    # label text (class [score]) near top-left
                    lbl = cls_name if (score is None or not self.draw_score) else f'{cls_name} {score:.2f}'
                    cv2.putText(vis, lbl, (int(x1), max(0, int(y1) - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1, cv2.LINE_AA)

                    any_drawn = True
                except Exception as e:
                    self.get_logger().warn(f'box parse/draw error: {e}')
                    continue

        # Publish inference results
        self.pub_infer.publish(out)

        # Publish OpenCV visualization (always publish; same stamp/frame as input)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            img_msg.header = msg.header
            self.pub_img.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'cv2 publish failed: {e}')

        if not any_drawn:
            if (self.get_clock().now().nanoseconds // 1_000_000_000) % 2 == 0:
                self.get_logger().info('no_results')

def main():
    rclpy.init()
    node = YoloObbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
