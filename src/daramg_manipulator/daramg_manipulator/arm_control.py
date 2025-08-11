#!/usr/bin/env python3
# file: arm_traj_client_lifecycle.py
import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


class ArmTrajectoryClient(LifecycleNode):
    def __init__(self):
        super().__init__('arm_trajectory_client')

        # 결과 알림 퍼블리셔 (latched처럼 마지막 값 유지되도록)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._done_pub = self.create_publisher(Bool, 'robotarm_completed', qos)

        self._action_name = '/arm_controller/follow_joint_trajectory'
        self._client: ActionClient = None
        self._send_timer = None
        self._goal_sent = False

        # 목표 트래젝터리 (헤더 타임스탬프는 비움 = 즉시 시작)
        jt = JointTrajectory()
        jt.joint_names = ['arm_base_yaw', 'arm_shoulder_pitch', 'arm_elbow_pitch', 'arm_wrist_pitch']

        p1 = JointTrajectoryPoint()
        p1.positions = [0.00, 0.14, 0.022, 0.00]
        p1.time_from_start = Duration(sec=1)

        p2 = JointTrajectoryPoint()
        p2.positions = [0.00, -0.50, 0.70, 0.00]
        p2.time_from_start = Duration(sec=8)

        jt.points = [p1, p2]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(sec=5)

        tol_vals = [0.05, 0.05, 0.05, 0.05]
        for name, tol in zip(jt.joint_names, tol_vals):
            t = JointTolerance()
            t.name = name
            t.position = float(tol)
            t.velocity = 0.0
            t.acceleration = 0.0
            goal.goal_tolerance.append(t)

        self._goal = goal

    # ── Lifecycle callbacks ─────────────────────────────────────────────────────
    def on_configure(self, _state):
        self.get_logger().info('Configuring...')
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._goal_sent = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state):
        self.get_logger().info('Activating... (will send trajectory when server is ready)')
        # 주기적으로 서버 준비 확인 후 goal 전송 (블로킹 회피)
        self._send_timer = self.create_timer(0.2, self._try_send_goal_once)
        return super().on_activate(_state)

    def on_deactivate(self, _state):
        self.get_logger().info('Deactivating...')
        if self._send_timer is not None:
            self._send_timer.cancel()
            self._send_timer = None
        return super().on_activate(_state)

    def on_cleanup(self, _state):
        self.get_logger().info('Cleaning up...')
        self._client = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state):
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    # ── Internal helpers ────────────────────────────────────────────────────────
    def _try_send_goal_once(self):
        if self._goal_sent:
            return
        if not self._client:
            return
        # 즉시 리턴 방식으로 서버 준비 확인
        if not self._client.wait_for_server(timeout_sec=0.0):
            return

        self.get_logger().info('Sending trajectory goal...')
        self._goal_sent = True
        if self._send_timer is not None:
            self._send_timer.cancel()
            self._send_timer = None

        send_future = self._client.send_goal_async(self._goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by the server')
            # 실패도 알리고 싶으면 False를 퍼블리시
            self._publish_done(False)
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        ok = (result.error_code == 0)
        self.get_logger().info(f'Action finished: error_code={result.error_code} '
                               f'error_string="{getattr(result, "error_string", "")}"')
        self._publish_done(ok)

    def _feedback_cb(self, feedback_msg):
        # 필요시 디버그 출력
        fb = feedback_msg.feedback
        # self.get_logger().debug(f'actual: {list(fb.actual.positions) if fb.actual.positions else []}')
        pass

    def _publish_done(self, success: bool):
        msg = Bool()
        msg.data = success  # 요청대로 성공 시 True. (원하면 항상 True로 바꿔도 됨)
        self._done_pub.publish(msg)
        self.get_logger().info(f'Published /robotarm_completed: {msg.data}')


def main():
    rclpy.init()
    node = ArmTrajectoryClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
