#!/usr/bin/env python3

import math, time
import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge

from daramg_msgs.msg import YoloDetection

from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber
import tf2_ros, tf2_geometry_msgs
from nav2_msgs.action import NavigateToPose

class YoloDepthToNavNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_to_nav')

        # 각종 파라미터들
        self.declare_parameter('detection_topic', '/yolo_detection')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        self.declare_parameter('target_class', 'whiteCross')
        self.declare_parameter('patch_size', 5)
        self.declare_parameter('min_z', 0.05)
        self.declare_parameter('max_z', 5.0)
        self.declare_parameter('publish_all', True)

        self.declare_parameter('approach_offset', 0.45)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')

        # 같은 프레임에서 너무 많은 목표를 한번에 사용하지 않도록 조절하는 파라미터
        self.declare_parameter('goal_cooldown_sec', 2.0)

        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.patch_size = int(self.get_parameter('patch_size').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.publish_all = bool(self.get_parameter('publish_all').value)

        self.approach = float(self.get_parameter('approach_offset').value)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.goal_cooldown = float(self.get_parameter('goal_cooldown_sec').value)

        if self.patch_size % 2 == 0:
            self.patch_size += 1

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # subscriber + YOLO와 Depth Frame 조절하기
        self.sub_det   = MFSubscriber(self, YoloDetection, detection_topic, qos_profile=qos_sensor)
        self.sub_depth = MFSubscriber(self, Image, depth_topic, qos_profile=qos_sensor)
        self.sub_info  = MFSubscriber(self, CameraInfo, camera_info_topic, qos_profile=qos_sensor)

        self.ts = ApproximateTimeSynchronizer([self.sub_det, self.sub_depth, self.sub_info],
                                              queue_size=10, slop=1.5)
        self.ts.registerCallback(self.synced_cb)

        # Publisher (디버깅용)
        self.pub_point_map = self.create_publisher(PointStamped, '/yolo_target_point_map', 10)
        self.pub_range = self.create_publisher(Float64, '/yolo_target_range', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/yolo_nav_goal', 10)  # Nav2는 안 봄, 디버그용

        # map <-> camera frame 변환을 위해 사용
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 이동
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.last_goal_ts = 0.0

        self.bridge = CvBridge()
        self.get_logger().info('YoloDepthToNavNode (action-based) started.')

    def synced_cb(self, det_msg: YoloDetection, depth_msg: Image, info_msg: CameraInfo):
        # Intrinsics
        fx = info_msg.k[0]; fy = info_msg.k[4]
        cx = info_msg.k[2]; cy = info_msg.k[5]

        # depth → numpy(m)
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth_m = depth.astype(np.float32) * 0.001
            elif depth.dtype in (np.float32, np.float64):
                depth_m = depth.astype(np.float32)
            else:
                self.get_logger().warn(f'Unsupported depth dtype: {depth.dtype}')
                return
        except Exception as e:
            self.get_logger().warn(f'depth cv_bridge fail: {e}')
            return

        H, W = depth_m.shape[:2]
        half = self.patch_size // 2

        # 대상 클래스만
        dets = [d for d in det_msg.yolo_detection
                if d.yolo_class_name == self.target_class and len(d.coordinates) == 8]
        if not dets:
            return

        published = 0
        for d in dets:
            pts = np.array(d.coordinates, dtype=np.float64).reshape(4, 2)
            u = int(round(pts[:, 0].mean()))
            v = int(round(pts[:, 1].mean()))
            if not (0 <= u < W and 0 <= v < H):
                continue

            # depth median
            u0, u1 = max(0, u - half), min(W, u + half + 1)
            v0, v1 = max(0, v - half), min(H, v + half + 1)
            patch = depth_m[v0:v1, u0:u1]
            valid = patch[(patch > self.min_z) & (patch < self.max_z) & np.isfinite(patch)]
            if valid.size == 0:
                z = float(depth_m[v, u])
                if not (self.min_z < z < self.max_z) or not np.isfinite(z):
                    continue
            else:
                z = float(np.median(valid))

            # 카메라 좌표계
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            # 카메라 → map
            ps_cam = PointStamped()
            ps_cam.header = depth_msg.header
            ps_cam.point.x, ps_cam.point.y, ps_cam.point.z = float(x), float(y), float(z)
            try:
                tf_cam_to_map = self.tf_buffer.lookup_transform(
                    self.global_frame, ps_cam.header.frame_id, Time())
                ps_map = tf2_geometry_msgs.do_transform_point(ps_cam, tf_cam_to_map)
            except Exception as e:
                self.get_logger().warn(f'TF to {self.global_frame} failed: {e}')
                continue

            # 디버그 퍼블리시
            ps_out = PointStamped()
            ps_out.header.frame_id = self.global_frame
            ps_out.header.stamp = depth_msg.header.stamp
            ps_out.point = ps_map.point
            self.pub_point_map.publish(ps_out)
            self.pub_range.publish(Float64(data=float(math.sqrt(
                ps_map.point.x**2 + ps_map.point.y**2 + ps_map.point.z**2))))

            # base_link in map (yaw 계산)
            try:
                tf_base_map = self.tf_buffer.lookup_transform(
                    self.global_frame, self.base_frame, Time())
                bx = tf_base_map.transform.translation.x
                by = tf_base_map.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f'TF {self.global_frame}->{self.base_frame} failed: {e}')
                bx = by = None

            goal = PoseStamped()
            goal.header.frame_id = self.global_frame
            goal.header.stamp = depth_msg.header.stamp

            if bx is not None:
                dx = ps_map.point.x - bx
                dy = ps_map.point.y - by
                theta = math.atan2(dy, dx)
                gx = ps_map.point.x - self.approach * math.cos(theta)
                gy = ps_map.point.y - self.approach * math.sin(theta)
                goal.pose.position.x = float(gx)
                goal.pose.position.y = float(gy)
                goal.pose.orientation.z = math.sin(theta/2.0)
                goal.pose.orientation.w = math.cos(theta/2.0)
            else:
                goal.pose.position.x = float(ps_map.point.x)
                goal.pose.position.y = float(ps_map.point.y)
                goal.pose.orientation.w = 1.0

            # 디버그 토픽(단순 시각화용)
            self.pub_goal.publish(goal)

            # action으로 전송하기
            now = time.time()
            if now - self.last_goal_ts < self.goal_cooldown:
                # 너무 자주 쏘지 않음
                continue

            if not self.nav_client.server_is_ready():
                self.get_logger().info('Waiting for Nav2 action server...')
                if not self.nav_client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().warn('Nav2 action server not ready.')
                    continue

            # 진행 중인 goal 있으면 취소
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal

            send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
            send_future.add_done_callback(self._goal_response_cb)

            self.last_goal_ts = now
            self.get_logger().info(
                f"[{d.yolo_class_name}] sent Nav2 goal: ({goal.pose.position.x:.2f},{goal.pose.position.y:.2f}), "
                f"offset={self.approach:.2f}m")
            published += 1
            if not self.publish_all:
                break

        if published == 0:
            self.get_logger().debug('no valid target after filters')

    # --- Action callbacks ---
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self.current_goal_handle = None
            return
        self.get_logger().info('Nav2 goal accepted.')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f'Nav2 feedback: {fb.distance_remaining:.2f} m')

    def _result_cb(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Nav2 result: {result}')
        except Exception as e:
            self.get_logger().warn(f'Nav2 result error: {e}')
        finally:
            self.current_goal_handle = None

def main():
    rclpy.init()
    node = YoloDepthToNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
