#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2, threading, numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # subscription for Image
        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            1
        )

        '''
        # subscription for CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw",
            self.image_callback,
            1
        )
        '''
        # cv2 <-> ROS bridge
        self.bridge = CvBridge()
        # variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Flag to control the display loop
        self.running = True

        # start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning"""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        # msg -> Image
        # msg -> CompressedImage
        """Callback Function to receive and store the latest frame"""
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_image(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800, 600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:
                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        rows, cols = img.shape[:2]

        # 1. Convert to HLS color space to extract lightness channel
        H,L,S = self.convert2hls(img)

        # 2. Invert lightness channel if we follow a dark line on a light background
        # L = 255 - L # Invert lightness channel

        # 3. apply a polygon mask to filter out simulation's bright sky
        L_masked, mask = self.apply_polygon_mask(L)

        # 4. For light line on dark background in simulation:
        lightnessMask = self.threshold_binary(L_masked, (50, 255))

        # For light line on dark background in real life environment:
        #lightnessMask = self.threshold_binary(L_masked, (180, 255))
        stackedMask = np.dstack((lightnessMask, lightnessMask, lightnessMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        # 5. return value of findContours depends on OpenCV version
        (contours,hierarchy) = cv2.findContours(lightnessMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # overlay mask on lightness image to show masked area on the small picture
        lightnessMask = cv2.addWeighted(mask,0.2,lightnessMask,0.8,0)

        # 6. Find the biggest contour (if detected) and calculate its centroid
        if len(contours) > 0:
            
            biggest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(biggest_contour)

            # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Show contour and centroid
            cv2.drawContours(contourMask, biggest_contour, -1, (0,255,0), 10)
            cv2.circle(contourMask, (cx, cy), 20, (0, 0, 255), -1)

            # Show crosshair and difference from middle point
            cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
            cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)
            cv2.line(crosshairMask,(int(cols/2),0),(int(cols/2),rows),(255,0,0),10)

        # Return processed frames
        return L_masked, contourMask, crosshairMask

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B
    
    # Convert to HLS color space
    def convert2hls(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        H = hls[:, :, 0]
        L = hls[:, :, 1]
        S = hls[:, :, 2]

        return H, L, S
    
    # apply a trapezoid polygon mask, size is hardcoded for 640 x 480px
    def apply_polygon_mask(self, img):
        # np.zeros_like(var)
        # var 변수의 크기만큼 0으로 찬 zero-matrix를 반환한다.
        mask = np.zeros_like(img)
        ignore_mask_color = 255
        imshape = img.shape

        vertices = np.array([[(0, imshape[0]), (500, 200), (800, 200), (imshape[1], imshape[0])]], dtype=np.int32)
        # cv2.fillPoly(img, pts, color)
        # img 내부의 pts 영역만큼 color 색으로 칠해서 반환
        # pts는 np를 활용한 다각형 배열
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        # cv2.bitwise_and(img1, img2, mask)
        # img1, img2간 서로 겹치는 영역 출력
        # img1과 mask만 입력하면 img중 mask에 해당되는 부분만 출력
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image, mask
    
    # apply threshold and result a binary image
    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255
    
    # add small images to the top row of the main image
    def add_small_pictures(self, img, samll_images, size=(160, 120)):
        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in samll_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack([small, small, small])
            img[y_offset:y_offset+size[1], x_offset:x_offset+size[0]] = small
            x_offset += size[0] + x_base_offset

        return img

    def stops(self):
        """Stop the node and the spin thread"""
        self.running = False
        self.spin_thread.join()

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()