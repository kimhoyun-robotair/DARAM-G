#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import os
import signal

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them as Twist messages.
W: Forward
S: Backward
A: Turn Left (Yaw +)
D: Turn Right (Yaw -)

Press SPACE to toggle between human control and autonomous exploration mode.
In autonomous mode, the node "ros2 run autonomous_exploration cartographer_control" is launched in a new terminal.
Press SPACE again to stop autonomous mode and return to manual control.
CTRL-C to quit.
"""

moveBindings = {
    'w': (1.0, 0.0),  # x+, forward
    's': (-1.0, 0.0), # x-, backward
    'a': (0.0, 1.0),  # yaw+
    'd': (0.0, -1.0), # yaw-
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

class TeleopTwistKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.25
        self.turn = 0.2
        self.x_val = 0.0
        self.yaw_val = 0.0
        self.control_mode = True  # True: human, False: autonomous
        self.autonomous_proc = None  # gnome-terminal subprocess 객체

    def enter_autonomous_mode(self):
        if self.autonomous_proc is None or self.autonomous_proc.poll() is not None:
            try:
                print("Launching autonomous_exploration rtabmap_control node in new gnome-terminal...")
                self.autonomous_proc = subprocess.Popen(
                    [
                        'gnome-terminal', '--', 'bash', '-c',
                        # exec로 bash를 ros2 run으로 대체, 세션 시작해서 PGID 관리
                        'exec ros2 run autonomous_exploration rtabmap_control'
                    ],
                    preexec_fn=os.setsid  # 새로운 세션/프로세스 그룹
                )
            except Exception as e:
                print(f"Failed to launch autonomous node: {e}")
        self.control_mode = False
        print("AUTONOMOUS mode activated. Manual control disabled.")

    def enter_human_mode(self):
        if self.autonomous_proc is not None and self.autonomous_proc.poll() is None:
            print("Terminating gnome-terminal subprocess for autonomous node...")
            try:
                # 전체 세션에 SIGTERM 전송 (터미널+bash+ros2 run+손자들)
                os.killpg(os.getpgid(self.autonomous_proc.pid), signal.SIGTERM)
                self.autonomous_proc.wait(timeout=5)
            except Exception as e:
                print("Terminal did not terminate gracefully, killing...", e)
                try:
                    os.killpg(os.getpgid(self.autonomous_proc.pid), signal.SIGKILL)
                except Exception as e2:
                    print("Force kill also failed:", e2)
            self.autonomous_proc = None
        self.control_mode = True
        self.x_val = 0.0
        self.yaw_val = 0.0
        print("HUMAN mode activated. Manual control enabled.")

    def keyboard_control(self, key):
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
        else:
            x = 0.0
            th = 0.0
            if key == '\x03':
                raise KeyboardInterrupt

        self.x_val += x * self.speed
        self.yaw_val += th * self.turn

        twist = Twist()
        twist.linear.x = self.x_val
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.yaw_val
        self.pub.publish(twist)
        print(f"[HUMAN] X: {twist.linear.x:.2f}   Yaw: {twist.angular.z:.2f}")

    def publish_zero_twist(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def shutdown_autonomous_proc(self):
        if self.autonomous_proc is not None and self.autonomous_proc.poll() is None:
            print("Shutting down gnome-terminal subprocess for autonomous node...")
            try:
                os.killpg(os.getpgid(self.autonomous_proc.pid), signal.SIGTERM)
                self.autonomous_proc.wait(timeout=5)
            except Exception as e:
                print("Force killing terminal...", e)
                try:
                    os.killpg(os.getpgid(self.autonomous_proc.pid), signal.SIGKILL)
                except Exception as e2:
                    print("Force kill also failed:", e2)
            self.autonomous_proc = None

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = TeleopTwistKeyboardNode()

    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)
            if node.control_mode:
                if key == ' ':
                    node.enter_autonomous_mode()
                    continue
                node.keyboard_control(key)
            else:
                # Autonomous 모드: 키보드 입력 무시, 스페이스/ctrl-c만 인식
                if key == ' ':
                    node.enter_human_mode()
                    continue
                if key == '\x03':
                    break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        node.publish_zero_twist()
        node.shutdown_autonomous_proc()
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
