#!/usr/bin/env python3

# Non-ROS Library
import threading, cv2, numpy as np

# ROS Library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from detection_msgs.msg import ColorBoundingBox, ColorBoundingBoxes

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # 파라미터 선언
        self.declare_parameter('image_topic_name', "/camera/image") # camera 토픽 이름
        self.declare_parameter('compressed_image_topic_name', '/camera/compressedImage') # 압축된 이미지 토픽 이름
        self.declare_parameter('depth_image_topic_name', '/camera/depth_image') # depth 이미지 토픽 이름
        self.declare_parameter('use_image_raw_or_compressed', 'image_raw') # 압축 이미지를 쓸건지 raw 이미지를 쓸건지 정하는 파라미터 (image_raw or compressed)

        # 파라미터 값 읽어오기
        self.image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value
        self.compressed_image_topic_name = self.get_parameter('compressed_image_topic_name').get_parameter_value().string_value
        self.depth_image_topic_name = self.get_parameter('depth_image_topic_name').get_parameter_value().string_value

        self.use_image_raw_or_compressed = self.get_parameter('use_image_raw_or_compressed').get_parameter_value().string_value

        # cv2 <-> ROS2
        self.cv_bridge = CvBridge()

        # ROS2 Publisher, Subscriber
        # 압축된 이미지를 사용할지, 아니면 raw 이미지를 사용할지 결정해서 조건문으로 분기
        if self.use_image_raw_or_compressed == "image_raw":
            self.image_subscription = self.create_subscription(Image, self.image_topic_name, self.image_cb, 5)
        elif self.use_image_raw_or_compressed == "compressed":
            self.compimage_subscription = self.create_subscription(CompressedImage, self.compressed_image_topic_name, self.compimage_cb, 5)
        self.depth_subscription = self.create_subscription(Image, self.depth_image_topic_name, self.depth_cb, 5)

        self.color_boundingboxes_publisher = self.create_publisher(ColorBoundingBoxes, '/color_bboxes', 5)

        # 가장 최근에 사용할 depth 및 RGB 이미지
        self.latest_depth = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.depth_lock = threading.Lock()


        # HSV를 통해서 이미지를 인식하기 위해서, 범위를 지정
        self.color_ranges = {
            'red1':  (np.array([0, 80, 60]),   np.array([10, 255, 255])),
            'red2':  (np.array([170, 80, 60]), np.array([180, 255, 255])),
            'yellow':(np.array([10, 30, 40]),  np.array([50, 255, 255])),
            'blue':  (np.array([90, 80, 50]),  np.array([130, 255, 255]))
        }

        # 탐지 대상의 색상에 맞춘 BBox용 색상들
        self.bbox_colors = {
            'red':    (0, 0, 255),
            'yellow': (0, 255, 255),
            'blue':   (255, 0, 0)
        }

        # 이미지 콜백과 별개로 이미지 처리 스레드를 백그라운드에서 돌리기
        # 콜백과 분리함으로서 효율성 확보
        self.processing_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.processing_thread.start()

        self.get_logger().info("Color Detection Configuration Completed")

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
            with self.frame_lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

    def compimage_cb(self, msg):
        # 압축된 이미지 콜백
        # cv_bridge를 활용해서 ROS2의 compressed image 토픽에서 opencv의 이미지 데이터로 변환
        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return
        
    def draw_bboxes(self, frame, mask, color, label):
        # 마스크 기반으로 외곽선 찾아 BBox 그리기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 작은 노이즈 무시
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
                cv2.putText(frame, label, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return frame

    def estimate_depth(self, bbox):
        # 전체 BBOX 영역에서 중앙값을 사용해서 depth를 추정하기
        if self.latest_depth is None:
            return None
        try:
            x1, y1, x2, y2 = map(int, [bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax]) # 정수로 변환
            # ROI 영역 조정하기
            w = x2 - x1
            h = y2 - y1
            x1 += int(w*0.3)
            x2 -= int(w*0.3)
            y1 += int(h*0.3) 
            y2 -= int(h*0.3)
            # 스레드 shared variable에 대한 lock 획득
            with self.depth_lock:
                depth_copy = self.latest_depth.copy()
            roi = depth_copy[max(0, y1):min(y2, depth_copy.shape[0]),
                            max(0, x1):min(x2, depth_copy.shape[1])]
            # ROI 영역에서 depth 값 추출
            valid_depth = roi[np.isfinite(roi) & (roi > 0)]
            if valid_depth.size == 0:
                return None
            return float(np.median(valid_depth))
        except Exception as e:
            self.get_logger().warn(f"Depth estimation failed: {e}")
            return None
        
    def process_loop(self):
        rate = self.create_rate(30) # 30Hz로 loop 생성
        while rclpy.ok():
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is None:
                rate.sleep()
                continue

            # 이미지를 BGR -> HSV로 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # ColorBoundingBoxes 메시지 선언
            color_boxes_msg = ColorBoundingBoxes()

            # 색상별 탐지 시작
            for color_name in ['red', 'yellow', 'blue']:
                mask = None
                if color_name == 'red':
                    mask1 = cv2.inRange(hsv, *self.color_ranges['red1'])
                    mask2 = cv2.inRange(hsv, *self.color_ranges['red2'])
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    mask = cv2.inRange(hsv, *self.color_ranges[color_name])

                # 마스크를 기반으로 BBox 그리기
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    if cv2.contourArea(cnt) < 500: # 너무 작은 값은 노이즈로 가정해서 버리기
                        continue

                    x, y, w, h = cv2.boundingRect(cnt)
                    cx, cy = int(x + w/2), int(y + h/2) # 중심점 구하기

                    depth_value = self.estimate_depth(ColorBoundingBox(xmin=x, ymin=y, xmax=(x+w), 
                                                                       ymax=(y+h), center_x=cx, center_y=cy)) # Depth 추정하기
                    if depth_value is None:
                        continue

                    # ColorBoundingBox 생성
                    box = ColorBoundingBox()
                    box.colorclass = color_name
                    box.xmin = x
                    box.ymin = y
                    box.xmax = (x+w)
                    box.ymax = (y+h)
                    box.center_x = cx
                    box.center_y = cy
                    box.depth = float(depth_value)
                    color_boxes_msg.color_bounding_boxes.append(box)

                    # 디버그용 시각화
                    cv2.rectangle(frame, (x, y), (x + w, y + h), self.bbox_colors[color_name], 2)
                    cv2.circle(frame, (cx, cy), 2, (0,255, 0), -1)
                    cv2.putText(frame, f"{color_name} {depth_value:.2f}m", (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.bbox_colors[color_name], 2)

            # 퍼블리시
            self.color_boundingboxes_publisher.publish(color_boxes_msg)

            # 시각화
            cv2.imshow("Color Detection Result", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

            rate.sleep()
                

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
        