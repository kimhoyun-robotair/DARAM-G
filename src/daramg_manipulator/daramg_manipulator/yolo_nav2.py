#!/usr/bin/env python3
import math, time
import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.node import Node

# Lifecycle
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from action_msgs.msg import GoalStatus

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge

from daramg_msgs.msg import YoloDetection

from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber
import tf2_ros, tf2_geometry_msgs
from nav2_msgs.action import NavigateToPose

class YoloDepthToNavLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_depth_to_nav')

        # 내부 상태 플래그
        self._active = False
        # YOLO coordinate
        self.first_only = None
        self._first_goal_sent = None

        # 파라미터는 configure 단계에서 선언/획득
        self.target_class = None
        self.patch_size = None
        self.min_z = None
        self.max_z = None
        self.publish_all = None
        self.approach = None
        self.base_frame = None
        self.global_frame = None
        self.goal_cooldown = None

        self.detection_topic = None
        self.depth_topic = None
        self.camera_info_topic = None

        # ROS entity 핸들
        self.sub_det = None
        self.sub_depth = None
        self.sub_info = None
        self.ts = None

        self.pub_point_map = None
        self.pub_range = None
        self.pub_goal = None

        self.nav_client = None
        self.current_goal_handle = None
        self.last_goal_ts = 0.0

        # lifecycle publisher (활성화/비활성화 가능)
        self.nav_completed_pub = None

        # TF
        self.tf_buffer = None
        self.tf_listener = None

        self.bridge = None

        self.get_logger().info('YoloDepthToNavLifecycle constructed.')

    # ---------------- Lifecycle Callbacks ----------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure()')
        try:
            # 파라미터 선언
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
            self.declare_parameter('goal_cooldown_sec', 2.0)
            self.declare_parameter('first_only', True)

            # 파라미터 값 읽기
            self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
            self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
            self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

            self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
            self.patch_size = int(self.get_parameter('patch_size').value)
            self.min_z = float(self.get_parameter('min_z').value)
            self.max_z = float(self.get_parameter('max_z').value)
            self.publish_all = bool(self.get_parameter('publish_all').value)

            self.approach = float(self.get_parameter('approach_offset').value)
            self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
            self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
            self.goal_cooldown = float(self.get_parameter('goal_cooldown_sec').value)
            self.first_only = bool(self.get_parameter('first_only').value)

            if self.patch_size % 2 == 0:
                self.patch_size += 1

            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )

            # publisher (디버깅용) — 일반 퍼블리셔는 lifecycle 제약 X
            self.pub_point_map = self.create_lifecycle_publisher(PointStamped, '/yolo_target_point_map', 10)
            self.pub_range = self.create_lifecycle_publisher(Float64, '/yolo_target_range', 10)
            self.pub_goal = self.create_lifecycle_publisher(PoseStamped, '/yolo_nav_goal', 10)

            # lifecycle publisher (완료 신호)
            self.nav_completed_pub = self.create_lifecycle_publisher(Bool, 'yolo_nav2_completed', 10)

            # message_filters 구독자 및 동기화기
            self.sub_det   = MFSubscriber(self, YoloDetection, self.detection_topic, qos_profile=qos_sensor)
            self.sub_depth = MFSubscriber(self, Image, self.depth_topic, qos_profile=qos_sensor)
            self.sub_info  = MFSubscriber(self, CameraInfo, self.camera_info_topic, qos_profile=qos_sensor)

            self.ts = ApproximateTimeSynchronizer([self.sub_det, self.sub_depth, self.sub_info],
                                                  queue_size=10, slop=1.5)
            self.ts.registerCallback(self.synced_cb)

            # TF
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # Nav2 action client
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.current_goal_handle = None
            self.last_goal_ts = 0.0
            self._first_goal_sent = False

            self.bridge = CvBridge()

            self.get_logger().info('Configured.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate()')
        try:
            self._first_goal_sent = False
            self._active = True
            self.get_logger().info('Activated.')
            return super().on_activate(state)
        except Exception as e:
            self.get_logger().error(f'on_activate error: {e}')
            return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate()')
        try:
            self._active = False
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.current_goal_handle = None

            self.get_logger().info('Deactivated.')
            return super().on_deactivate(state)
        except Exception as e:
            self.get_logger().error(f'on_deactivate error: {e}')
            return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup()')
        try:
            self._active = False

            # 진행 중인 goal 취소
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.current_goal_handle = None

            # 동기화기/구독자 참조 해제 (rclpy 자원은 노드 종료 시 정리됨)
            self.ts = None
            self.sub_det = None
            self.sub_depth = None
            self.sub_info = None

            self.tf_listener = None
            self.tf_buffer = None

            self.nav_client = None
            self.bridge = None

            # 퍼블리셔도 참조만 해제 (lifecycle publisher는 deactivate된 상태)
            self.pub_point_map = None
            self.pub_range = None
            self.pub_goal = None
            self.nav_completed_pub = None

            self.get_logger().info('Cleaned up.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_cleanup error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown()')
        try:
            self._active = False
            if self.current_goal_handle is not None:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.current_goal_handle = None
            self.get_logger().info('Shut down.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_shutdown error: {e}')
            return TransitionCallbackReturn.FAILURE

    # ---------------- Core Logic ----------------

    def synced_cb(self, det_msg: YoloDetection, depth_msg: Image, info_msg: CameraInfo):
        # 비활성 상태면 콜백 무시
        if not self._active:
            return
        if self.first_only and self._first_goal_sent:
            return
        # Intrinsics
        fx = info_msg.k[0]; fy = info_msg.k[4]
        cx = info_msg.k[2]; cy = info_msg.k[5]

        # depth → numpy(m)
        try:
            if self.bridge is None:
                return
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
            if self.pub_point_map:
                ps_out = PointStamped()
                ps_out.header.frame_id = self.global_frame
                ps_out.header.stamp = depth_msg.header.stamp
                ps_out.point = ps_map.point
                self.pub_point_map.publish(ps_out)

            if self.pub_range:
                rng = float(math.sqrt(
                    ps_map.point.x**2 + ps_map.point.y**2 + ps_map.point.z**2))
                self.pub_range.publish(Float64(data=rng))

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

            # 디버그 goal
            if self.pub_goal:
                self.pub_goal.publish(goal)

            # action 전송(쿨다운)
            now = time.time()
            if now - self.last_goal_ts < self.goal_cooldown:
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

            if self.first_only:
                self._first_goal_sent = True

            send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
            send_future.add_done_callback(self._goal_response_cb)

            self.last_goal_ts = now
            self.get_logger().info(
                f"[{d.yolo_class_name}] sent Nav2 goal: "
                f"({goal.pose.position.x:.2f},{goal.pose.position.y:.2f}), "
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
            if self.first_only:
                self._first_goal_sent = False
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
            result_wrapper = future.result()  # has .status and .result
            status = result_wrapper.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                # 활성화 상태에서만 publish
                if self._active and self.nav_completed_pub is not None:
                    self.nav_completed_pub.publish(Bool(data=True))
                self.get_logger().info('Nav2 result: SUCCEEDED -> published yolo_nav2_completed=True')
            else:
                self.get_logger().info(f'Nav2 result status: {status}')
        except Exception as e:
            self.get_logger().warn(f'Nav2 result error: {e}')
        finally:
            self.current_goal_handle = None

def main():
    rclpy.init()
    node = YoloDepthToNavLifecycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
