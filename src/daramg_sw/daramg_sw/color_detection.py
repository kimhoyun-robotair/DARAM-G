import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # ✅ 카메라 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',   # 실제 토픽 이름 확인 필요
            self.image_callback,
            1
        )

        # ✅ 결과 이미지 퍼블리셔
        self.image_pub = self.create_publisher(Image, '/image_with_rect', 10)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # ✅ HSV 범위 정의
        # 빨강은 Hue가 양 끝단(0~10, 170~180)에 분포하므로 2개 구간으로 나눔
        self.color_ranges = {
            'red1':  (np.array([0, 80, 60]),   np.array([10, 255, 255])),
            'red2':  (np.array([170, 80, 60]), np.array([180, 255, 255])),
            'yellow':(np.array([10, 30, 40]),  np.array([50, 255, 255])),
            'blue':  (np.array([90, 80, 50]),  np.array([130, 255, 255]))
        }

        # 각 색상별 박스 컬러 (BGR)
        self.bbox_colors = {
            'red':    (0, 0, 255),
            'yellow': (0, 255, 255),
            'blue':   (255, 0, 0)
        }

        # ✅ 백그라운드 스레드 시작
        self.processing_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.processing_thread.start()

        self.get_logger().info("ColorDetector node started (red, yellow, blue).")

    def image_callback(self, msg):
        """ROS 이미지 → OpenCV 이미지로 변환"""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.frame_lock:
            self.latest_frame = frame.copy()

    def process_loop(self):
        rate = self.create_rate(30)  # 30Hz 처리 루프
        while rclpy.ok():
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is None:
                rate.sleep()
                continue

            # ✅ BGR → HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # ✅ 색상별 탐지
            detected_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

            # --- 빨강 ---
            mask_red1 = cv2.inRange(hsv, *self.color_ranges['red1'])
            mask_red2 = cv2.inRange(hsv, *self.color_ranges['red2'])
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            frame = self.draw_bboxes(frame, mask_red, self.bbox_colors['red'], 'Red')

            # --- 노랑 ---
            mask_yellow = cv2.inRange(hsv, *self.color_ranges['yellow'])
            frame = self.draw_bboxes(frame, mask_yellow, self.bbox_colors['yellow'], 'Yellow')

            # --- 파랑 ---
            mask_blue = cv2.inRange(hsv, *self.color_ranges['blue'])
            frame = self.draw_bboxes(frame, mask_blue, self.bbox_colors['blue'], 'Blue')

            # ✅ 결과 퍼블리시
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_img)

            rate.sleep()

    def draw_bboxes(self, frame, mask, color, label):
        """마스크 기반으로 외곽선 찾아 BBox 그리기"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 작은 노이즈 무시
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
                cv2.putText(frame, label, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
