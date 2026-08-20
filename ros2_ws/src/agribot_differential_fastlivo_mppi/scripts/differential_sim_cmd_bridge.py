#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class DifferentialSimCommandBridge(Node):
    def __init__(self):
        super().__init__("differential_sim_cmd_bridge")
        self.ready = False
        self.last_command_ns = None
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 0.5).value
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ready_subscription = self.create_subscription(
            Bool, "/fastlivo_rtk/ready", self.handle_ready, state_qos
        )
        self.command_subscription = self.create_subscription(
            Twist, "/nav2/cmd_vel", self.handle_command, 10
        )
        self.timer = self.create_timer(0.05, self.enforce_watchdog)

    def handle_ready(self, message):
        was_ready = self.ready
        self.ready = bool(message.data)
        if was_ready and not self.ready:
            self.publish_stop()

    def handle_command(self, message):
        self.last_command_ns = self.get_clock().now().nanoseconds
        if self.ready:
            self.publisher.publish(message)
        else:
            self.publish_stop()

    def enforce_watchdog(self):
        if self.last_command_ns is None:
            return
        age_sec = (
            self.get_clock().now().nanoseconds - self.last_command_ns
        ) / 1.0e9
        if not self.ready or age_sec > self.command_timeout_sec:
            self.publish_stop()
            self.last_command_ns = None

    def publish_stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialSimCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
