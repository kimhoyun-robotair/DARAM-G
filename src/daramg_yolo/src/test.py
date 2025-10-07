#!/usr/bin/env python3

# Non-ROS Library
import os, threading, time
import numpy as np
from ultralytics import YOLO
import cv2, torch

# ROS Library
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBox, BoundingBoxes


class YoloDetection(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        # =================== Device 설정 ===================
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {device}")

        # =================== 파라미터 선언 ===================
        self.declare_parameter('model_path', os.path.join(os.environ['HOME'], 'DARAM-G/src/daramg_yolo/src/coke.pt'))
        self.declare_parameter('confidence', 0.85)
        self.declare_parameter('image_topic_name', "/camera/image")
        self.declare_parameter('compressed_image_topic_name', '/camera/compressedImage')
        self.declare_parameter('depth_image_topic_name', '/camera/depth_image')
        self.declare_parameter('use_image_raw_or_compressed', 'image_raw')  # image_raw or compressed
        self.declare_parameter('yolo_detect_topic', '/yolo_detect_image')
        self.declare_parameter('target_classes', [])
        self.declare_parameter('enable_fps_log', True)  # 🔸 FPS 로깅 활성화 여부

        # =================== 파라미터 로드 ===================
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value
        self.compressed_image_topic_name = self.get_parameter('compressed_image_topic_name').get_parameter_value().string_value
        self.depth_image_topic_name = self.get_parameter('depth_image_topic_name').get_parameter_value().string_value
        self.use_image_raw_or_compressed = self.get_parameter('use_image_raw_or_compressed').get_parameter_value().string_value
        self.yolo_detect_topic = self.get_parameter('yolo_detect_topic').get_parameter_value().string_value
        self.target_classes = list(self.get_parameter('target_classes').get_parameter_value().string_array_value)
        self.enable_fps_log = bool(self.get_parameter('enable_fps_log').value)

        # =================== YOLO 모델 로드 ===================
        try:
            self.yolo_model = YOLO(self.model_path)
            self.yolo_model.to(device)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO Model: {e}")
            raise FileNotFoundError

        # =================== ROS 인터페이스 설정 ===================
        self.cv_bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.latest_depth = None

        # FPS 계산용 변수
        self.frame_count = 0
        self.last_log_time = time.perf_counter()
        self.avg_fps = 0.0

        # Subscriber 설정
        if self.use_image_raw_or_compressed == "image_raw":
            self.image_subscription = self.create_subscription(Image, self.image_topic_name, self.image_cb, 5)
        elif self.use_image_raw_or_compressed == "compressed":
            self.compimage_subscription = self.create_subscription(CompressedImage, self.compressed_image_topic_name, self.compimage_cb, 5)

        self.depth_subscription = self.create_subscription(Image, self.depth_image_topic_name, self.depth_cb, 5)

        # Publisher 설정
        self.yoloimage_publisher = self.create_publisher(CompressedImage, self.yolo_detect_topic, 5)
        self.boundingboxes_publisher = self.create_publisher(BoundingBoxes, '/yolo_bboxes', 5)

        self.get_logger().info("Yolo Detection Configuration Completed")

    # =================== Depth Callback ===================
    def depth_cb(self, msg: Image):
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.latest_depth = depth_image
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    # =================== Image Callback ===================
    def image_cb(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(frame, msg.header)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    # =================== Compressed Image Callback ===================
    def compimage_cb(self, msg):
        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(frame, msg.header)
        except Exception as e:
            self.get_logger().error(f"Compressed image conversion failed: {e}")

    # =================== YOLO Inference Core ===================
    def process_image(self, frame, header):
        start_time = time.perf_counter()  # 🔸 시작 시간 기록

        try:
            yolo_results = self.yolo_model.predict(source=frame, conf=self.confidence, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO Inference Failed: {e}")
            return

        bboxes = BoundingBoxes()
        bboxes.header = header
        bboxes.image_header = header
        copy_image = frame.copy()

        # Depth 데이터 안전하게 복사
        with self.depth_lock:
            depth_copy = None if self.latest_depth is None else self.latest_depth.copy()

        for r in yolo_results:
            boxes = getattr(r, 'boxes', None)
            if boxes is None or len(boxes) == 0:
                continue

            for b in boxes:
                try:
                    x1, y1, x2, y2 = map(int, b.xyxy[0].to('cpu').detach().numpy().tolist())
                    cx, cy = int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)

                    # Depth 값 추출 (copy로 보호된 버전)
                    depth_value = -1.0
                    if depth_copy is not None and 0 <= cx < depth_copy.shape[1] and 0 <= cy < depth_copy.shape[0]:
                        depth_value = depth_copy[cy, cx]
                        if np.isnan(depth_value) or np.isinf(depth_value):
                            depth_value = -1.0

                    class_id = int(b.cls.item()) if hasattr(b.cls, 'item') else int(b.cls)
                    class_name = self.yolo_model.names[class_id] if hasattr(self.yolo_model, 'names') else str(class_id)
                    score = float(b.conf.item()) if hasattr(b, 'conf') else 0.0

                    bbox = BoundingBox()
                    bbox.yoloclass = class_name
                    bbox.probability = score
                    bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = x1, y1, x2, y2
                    bbox.center_x, bbox.center_y = cx, cy
                    bbox.depth = float(depth_value)
                    bboxes.bounding_boxes.append(bbox)
                except Exception as e:
                    self.get_logger().error(f"Error processing bbox: {e}")

        # Publish
        try:
            compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(copy_image, dst_format='jpg')
            compressed_msg.header = header
            self.yoloimage_publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Compressed publish failed: {e}")

        self.boundingboxes_publisher.publish(bboxes)

        # 🔸 FPS 계산
        end_time = time.perf_counter()
        elapsed = end_time - start_time
        self.frame_count += 1

        if self.enable_fps_log:
            if elapsed > 0:
                current_fps = 1.0 / elapsed
                self.avg_fps = (self.avg_fps * 0.9) + (current_fps * 0.1)  # EMA(지수이동평균)
            
            if time.perf_counter() - self.last_log_time >= 2.0:  # 2초마다 출력
                self.get_logger().info(f"[YOLO] Current FPS: {current_fps:.2f} | Avg FPS: {self.avg_fps:.2f}")
                self.last_log_time = time.perf_counter()


# =================== Main Entry ===================
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetection()

    # 🔸 멀티스레드 / 단일스레드 비교를 위해 주석 토글 가능
    # executor = rclpy.executors.SingleThreadedExecutor()  # 기본 spin()과 동일
    executor = MultiThreadedExecutor(num_threads=4)       # 멀티스레드 모드

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
