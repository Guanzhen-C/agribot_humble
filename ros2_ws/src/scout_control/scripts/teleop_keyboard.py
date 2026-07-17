#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LIN_VEL_LIMIT = 1.0
ANG_VEL_LIMIT = 2.0
LIN_VEL_STEP = 0.1
ANG_VEL_STEP = 0.1

INFO = """
----------------------------------------
Scout Keyboard Teleoperation Panel
----------------------------------------
                 W
             A   S   D
                 X
W/S : Increase/decrease linear velocity
D/A : Increase/decrease angular velocity
X   : Emergency brake
Press CTRL+C to quit
----------------------------------------
"""


def constrain(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def generate_cmd_vel(linear: float, angular: float) -> Twist:
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("scout_teleop_keyboard")
        self.cmd_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_current(self) -> None:
        self.publisher.publish(generate_cmd_vel(self.linear, self.angular))

    def run(self) -> None:
        print(INFO)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == "w":
                    self.linear = constrain(self.linear + LIN_VEL_STEP, -LIN_VEL_LIMIT, LIN_VEL_LIMIT)
                elif key == "s":
                    self.linear = constrain(self.linear - LIN_VEL_STEP, -LIN_VEL_LIMIT, LIN_VEL_LIMIT)
                elif key == "a":
                    self.angular = constrain(self.angular + ANG_VEL_STEP, -ANG_VEL_LIMIT, ANG_VEL_LIMIT)
                elif key == "d":
                    self.angular = constrain(self.angular - ANG_VEL_STEP, -ANG_VEL_LIMIT, ANG_VEL_LIMIT)
                elif key == "x":
                    self.linear = 0.0
                    self.angular = 0.0
                elif key == "\x03":
                    break
                else:
                    continue
                self.publish_current()
        finally:
            if rclpy.ok():
                self.linear = 0.0
                self.angular = 0.0
                self.publish_current()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main() -> None:
    if os.name == "nt":
        raise RuntimeError("teleop_keyboard.py only supports POSIX terminals")
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
