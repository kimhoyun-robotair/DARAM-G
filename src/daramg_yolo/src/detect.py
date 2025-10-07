#!/usr/bin/env python3

# Non-ROS Library
import os, threading, numpy as np
from ultralytics import YOLO
import cv2, torch

# ROS Library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBox, BoundingBoxes

class YoloDetection(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        # CUDA or CPU
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {device}")

        # 파라미터 선언
        self.declare_parameter('model_path', os.path.join(os.environ['HOME']+'/DARAM-G/src/daramg_yolo/src/coke.pt')) # YOLO 가중치 모델 경로
        self.declare_parameter('confidence', 0.85) # YOLO detection confidence threshold
        self.declare_parameter('image_topic_name', "/camera/image") # camera 토픽 이름
        self.declare_parameter('compressed_image_topic_name', '/camera/compressedImage') # 압축된 이미지 토픽 이름
        self.declare_parameter('depth_image_topic_name', '/camera/depth_image') # depth 이미지 토픽 이름
        
        self.declare_parameter('use_image_raw_or_compressed', 'image_raw') # 압축 이미지를 쓸건지 raw 이미지를 쓸건지 정하는 파라미터 (image_raw or compressed)

        self.declare_parameter('yolo_detect_topic', '/yolo_detect_image') # yolo detection 이후 publish 할 이미지 토픽 이름
        self.declare_parameter('target_classes', []) # 나중에 객체가 탐지되었는지, 안되었는지 검토하기 위한 파라미터 선언

        # 파라미터 값 읽어오기
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value
        self.compressed_image_topic_name = self.get_parameter('compressed_image_topic_name').get_parameter_value().string_value
        self.depth_image_topic_name = self.get_parameter('depth_image_topic_name').get_parameter_value().string_value

        self.use_image_raw_or_compressed = self.get_parameter('use_image_raw_or_compressed').get_parameter_value().string_value

        self.yolo_detect_topic = self.get_parameter('yolo_detect_topic').get_parameter_value().string_value
        self.target_classes = list(self.get_parameter('target_classes').get_parameter_value().string_array_value)

        # YOLO 모델 불러오기
        try:
            self.yolo_model = YOLO(self.model_path)
            self.yolo_model.to(device)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO Model: {e}")
            raise FileNotFoundError
        
        # cv2 <-> ROS2
        self.cv_bridge = CvBridge()

        # 멀티 스레딩 사용시 shared variable에 대한 lock
        self.depth_lock = threading.Lock()

        # ROS2 Publisher, Subscriber
        # 압축된 이미지를 사용할지, 아니면 raw 이미지를 사용할지 결정해서 조건문으로 분기
        if self.use_image_raw_or_compressed == "image_raw":
            self.image_subscription = self.create_subscription(Image, self.image_topic_name, self.image_cb, 5)
        elif self.use_image_raw_or_compressed == "compressed":
            self.compimage_subscription = self.create_subscription(CompressedImage, self.compressed_image_topic_name, self.compimage_cb, 5)
        self.depth_subscription = self.create_subscription(Image, self.depth_image_topic_name, self.depth_cb, 5)

        self.yoloimage_publisher = self.create_publisher(CompressedImage, self.yolo_detect_topic, 5)
        # self.yoloimage_publisher = self.create_publisher(Image, self.yolo_detect_topic, 5)
        self.boundingboxes_publisher = self.create_publisher(BoundingBoxes, '/yolo_bboxes', 5)

        # 가장 최근에 사용할 depth 이미지
        self.latest_depth = None

        self.get_logger().info("Yolo Detection Configuration Completed")

    def process_image(self, frame, header):
        # 이미지 처리 함수
        try:
            yolo_results = self.yolo_model.predict(source=frame, conf=self.confidence, verbose=False) # yolo 모델을 통한 객체 인식 진행
        except Exception as e:
            self.get_logger().error(f"YOLO Inference Failed:{e}")
            return
        
        # YOLO 탐지 결과를 저장하고 publish 하기 위한 bbox 집합 인터페이스(메시지)를 지정
        bboxes = BoundingBoxes()
        bboxes.header = header
        bboxes.image_header = header

        copy_image = frame.copy() # yolo 탐지 결과를 표시하기 위해서 영상 이미지를 복사함

        with self.depth_lock:
            # depth_lock을 획득한 상태에서만 latest_depth에 접근 가능하도록 스레드 처리
            depth_copy = None if self.latest_depth is None else self.latest_depth.copy()

        for r in yolo_results:
            boxes = getattr(r, 'boxes', None) # yolo 탐지 결과 안에 box 속성이 있는지 확인
                                              # getattr(object, attribute, default) -> object에 attribute(속성)이 있는지 확인 -> default는 없을 경우 결과로 raise하는 값
                                              # 이 코드는 result 안에 box가 있으면 그대로 진행하고, 없으면 None을 띄우도록 작성
            if boxes is None or len(boxes) == 0: # boxes가 값이 None이라면 그냥 빠르게 pass
                continue

            for b in boxes: # boxes 안에 들어있는 box들에 대해서
                try:
                    x1, y1, x2, y2 = b.xyxy[0].to('cpu').detach().numpy().tolist()
                    # YOLO 탐지 결과로 나오는 xyxy[0] GPU 텐서를 CPU 메모리에 복사하고, numpy -> list로 차례대로 변환
                    # GPU 상 YOLO 탐지 결과를 파이썬 리스트로 변환
                    
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2]) # 정수 변환

                    # BBox의 중심점 계산
                    cx = int((x1+x2)/2.0)
                    cy = int((y1+y2)/2.0)

                    # depth 계산
                    depth_value = -1.0 # 초기화를 통해서 런타임 오류 발생 방지
                    if depth_copy is not None:
                        if 0 <= cx < depth_copy.shape[1] and 0 <= cy < depth_copy.shape[0]:
                            depth_value = depth_copy[cy, cx]
                            if np.isnan(depth_value) or np.isinf(depth_value):
                                depth_value = -1.0
                        else:
                            depth_value = -1.0
                    
                    # yolo가 탐지한 객체에 대한 class id, class name 그리고 확률을 지정
                    class_id = int(b.cls.item()) if hasattr(b.cls, 'item') else int(b.cls)
                    class_name = self.yolo_model.names[class_id] if hasattr(self.yolo_model, 'names') else str(class_id)
                    score = float(b.conf.item()) if hasattr(b, 'conf') else 0.0
                    # hasattr 함수는 지정된 객체가 그 속성을 가지고 있는지 판독하는 파이썬 내장함수

                    bbox = BoundingBox()
                    bbox.yoloclass = class_name
                    bbox.probability = score
                    bbox.xmin = x1
                    bbox.ymin = y1
                    bbox.xmax = x2
                    bbox.ymax = y2
                    bbox.center_x = cx
                    bbox.center_y = cy
                    bbox.depth = float(depth_value) if self.latest_depth is not None else -1.0

                    bboxes.bounding_boxes.append(bbox)

                    # 구한 좌표를 기반으로 bbox 그리기
                    cv2.rectangle(copy_image,
                                  (x1, y1), (x2, y2),
                                  (0, 255, 0), 2)
                    # 탐지된 객체 중심 표시
                    cv2.circle(copy_image, (cx, cy), 2, (0, 255, 0), -1)
                    # 객체 이름과 신뢰도(확률) 표시
                    label = f"{class_name}: {score:.2f}, {depth_value:.2f}m"
                    cv2.putText(copy_image, label, (x1, max(0, y1-5)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing bounding box: {e}")
                    continue
        # cv2의 imshow 함수를 사용해서 yolo 탐지 결과 시각화
        cv2.imshow("YOLO Detection Result", copy_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
        
        # 대역폭 절감을 위해서 압축된 이미지로 publish (raw 이미지는 선택)
        try:
            compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(copy_image, dst_format='jpg')
            # compressed_msg = self.cv_bridge.cv2_to_imgmsg(copy_image, encoding='bgr8')
            compressed_msg.header = header
            self.yoloimage_publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Compressed publish failed: {e}")
        
        self.boundingboxes_publisher.publish(bboxes)

    def estimate_depth(self, bbox):
        # bbox 영역의 depth 값을 중앙값으로 계산해서 더 정확한 depth 값 산출
        if self.latest_depth is None:
            return None
        x1, y1, x2, y2 = map(int, [bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax])
        roi = self.latest_depth[y1:y2, x1:x2]
        valid_depth = roi[np.isfinite(roi)] & (roi > 0)
        if valid_depth.size == 0:
            return None
        return float(np.median(valid_depth))
    
    def depth_cb(self, msg:Image):
        # depth 이미지 콜백
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.latest_depth = depth_image # lock 내부에서 depth 이미지 업데이트
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

    def image_cb(self, msg):
        # raw image 콜백
        # cv_bridge를 활용해서 ROS2의 image 토픽에서 opencv의 이미지 데이터로 변환
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(frame, msg.header)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

    def compimage_cb(self, msg):
        # 압축된 이미지 콜백
        # cv_bridge를 활용해서 ROS2의 compressed image 토픽에서 opencv의 이미지 데이터로 변환
        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(frame, msg.header)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()