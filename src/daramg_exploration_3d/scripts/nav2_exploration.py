#!/usr/bin/env python3
# file: frontier_nav2_commander.py

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from frontier_msgs.msg import FrontierInfoArray          # 커스텀 배열 msg
from nav2_msgs.action import NavigateToPose              # Nav2 액션
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String                          # nav2_status 알림용

class Nav2Commander(Node):

    def __init__(self):
        super().__init__('nav2_commander')

        # ❶ FrontierInfoArray 구독
        self.create_subscription(
            FrontierInfoArray,
            'frontier_info',
            self.frontier_cb,
            10)

        # ❷ Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ❸ 상태 퍼블리셔 (FrontierDetector 타이머 제어용)
        self.status_pub = self.create_publisher(String, 'nav2_status', 10)

        self.get_logger().info('Nav2 Commander ready.')

    # ─────────────────────────────────────────────────────────────
    def frontier_cb(self, msg: FrontierInfoArray):
        if not msg.frontiers:
            self.get_logger().warn('No frontiers received!')
            return

        # ❶ 100 ≤ cells ≤ 10 000 인 프런티어만 추려 놓기
        valid = [f for f in msg.frontiers if 5 <= f.cells <= 10000]

        # ❷ 유효 후보가 없으면(모두 너무 작거나 너무 큼) fallback
        if not valid:
            self.get_logger().warn(
                'No frontier in desired cell-range; falling back to overall max.')
            valid = msg.frontiers               # 전부 사용하도록 리셋

        # ❸ 그중 셀 수가 가장 큰 프런티어 선택
        best = max(valid, key=lambda f: f.cells)

        # ───── 디버그 출력 ─────
        self.get_logger().info(
            f'Selected frontier #{best.index}: '
            f'cells={best.cells}, edge≈{best.edge_estimate:.2f} m, '
            f'centroid=({best.centroid.x:.2f},{best.centroid.y:.2f},{best.centroid.z:.2f})'
        )

        # ───── Nav2 Goal 생성 & 전송 ─────
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = msg.header.frame_id
        goal_pose.header.stamp    = self.get_clock().now().to_msg()
        goal_pose.pose.position   = best.centroid
        goal_pose.pose.orientation.w = 1.0         # yaw = 0
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        self.status_pub.publish(String(data='MOVING'))
        self.nav_client.send_goal_async(goal_msg).add_done_callback(
            self._goal_response_cb)


    # ─────────────────────────────────────────────────────────────
    def _goal_response_cb(self, future):
        """Goal 을 서버가 수락했을 때 호출"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            self.status_pub.publish(String(data='FAILED'))
            return

        self.get_logger().info('Goal accepted; waiting for result…')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    # ─────────────────────────────────────────────────────────────
    def _result_cb(self, future):
        """Nav2 가 목표를 종료했을 때 호출"""
        res_msg   = future.result().result      # NavigateToPose_Result
        status_msg = String()

        # ── 버전별 필드 호환 처리 ───────────────────────────
        if hasattr(res_msg, 'error_code'):              # Rolling 이후
            succeeded = (res_msg.error_code == 0)
        elif hasattr(res_msg, 'result'):                # Foxy-Humble 일부 브랜치
            succeeded = (res_msg.result == 0)
        elif hasattr(res_msg, 'success'):               # Iron 이후 bool
            succeeded = res_msg.success
        else:
            succeeded = False                           # 알 수 없는 형식 → 실패

        # ── 로깅 & 상태 알림 ───────────────────────────────
        if succeeded:
            self.get_logger().info('Arrived at goal.')
            status_msg.data = 'ARRIVED'
        else:
            self.get_logger().warn('Nav2 reported failure.')
            status_msg.data = 'FAILED'

        self.status_pub.publish(status_msg)

# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = Nav2Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
