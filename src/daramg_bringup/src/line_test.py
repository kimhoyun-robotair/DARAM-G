#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from detection_msgs.msg import LineDetection
from utility import compute_cmd_vel_from_line, LineConfig

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # create a subscriber to line detection topic
        self.subscription = self.create_subscription(
            LineDetection,
            '/line_detection',
            self.line_detection_callback,
            5
        )

        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.line_config = LineConfig(deadband_px=20, v_turn=0.05, v_straight=0.1,
                                      w_turn=0.15, lin_max=0.3, ang_max=0.5)
        
    def line_detection_callback(self, msg: LineDetection):
        """Callback function to process line detection and publish cmd_vel."""
        lin_x, ang_z, self.line_info = compute_cmd_vel_from_line(obs=msg,
                                                                 config=self.line_config)
        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z
        self.cmdvel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    
    try:
        rclpy.spin(node)  # Run the display loop
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()