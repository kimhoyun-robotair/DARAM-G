import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',   # 실제 토픽 이름 확인 필요
            self.image_callback,
            1
        )

        # 결과 이미지 퍼블리셔
        self.image_pub = self.create_publisher(Image, '/image_with_rect', 10)

        # HSV 범위 (예: 검은색)
        self.lower = np.array([0, 0, 0])
        self.upper = np.array([180, 255, 50])

        self.bridge = CvBridge()

        # 최신 프레임 저장
        self.latest_frame = None
        self.frame_lock = threading.Lock()

    def image_callback(self, msg):
        """ROS 이미지 → OpenCV 이미지"""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.frame_lock:
            self.latest_frame = frame.copy()

    def process_image(self):
        while rclpy.ok():
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is None:
                continue  # 아직 프레임 없음

            # HSV 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 특정 색상 범위 마스크
            mask = cv2.inRange(hsv, self.lower, self.upper)

            # 외곽선 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 500:  # 작은 노이즈 제거
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

            # 새로운 이미지 토픽으로 publish
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_img)

            # 시각화 (옵션)
            cv2.imshow("mask image", mask)
            cv2.imshow("detected objects", frame)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        node.process_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
