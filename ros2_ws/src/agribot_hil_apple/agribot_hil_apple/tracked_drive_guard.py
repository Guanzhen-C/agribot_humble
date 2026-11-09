"""Forward native navigation commands only after the physical CAN zero handshake."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


def drive_enabled(status, status_age, command_age):
    return (0 <= status_age < 2. and 0 <= command_age < .3 and
            status.get('motion_authorized') is True and
            status.get('truth_fresh') is True and
            status.get('drive_lease_fresh') is True and status.get('fault') == '')


class TrackedDriveGuard(Node):
    def __init__(self):
        super().__init__('hil_tracked_drive_guard')
        self.status = {}
        self.command = Twist()
        self.status_at = self.command_at = -math.inf
        self.enabled = None
        self.output = self.create_publisher(Twist, '/hil/drive/command', 1)
        self.create_subscription(Twist, '/hil/navigation/cmd_vel', self.receive_command, 1)
        self.create_subscription(String, '/hil/tracked_can/status', self.receive_status, 1)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_command(self, msg):
        if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.command, self.command_at = msg, time.monotonic()

    def receive_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
        except (ValueError, TypeError):
            return
        self.status, self.status_at = status, time.monotonic()

    def tick(self):
        now = time.monotonic()
        enabled = drive_enabled(self.status, now-self.status_at, now-self.command_at)
        # Zero goes through the same native driver, USB-CAN and 6750 as motion.
        # An expired receiver never resumes on nonzero commands alone.
        self.output.publish(self.command if enabled else Twist())
        if enabled != self.enabled:
            self.get_logger().info('Physical drive handshake: '+('ready' if enabled else 'neutral'))
            self.enabled = enabled


def main():
    rclpy.init()
    node = TrackedDriveGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
