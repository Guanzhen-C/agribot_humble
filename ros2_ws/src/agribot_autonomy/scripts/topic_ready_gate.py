#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TopicReadyGate(Node):
    def __init__(self) -> None:
        super().__init__("topic_ready_gate")
        self.topic = self.declare_parameter("topic", "/amcl_pose").value
        self.message_type = self.declare_parameter("message_type", "pose").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 30.0).value)
        self.received = False
        self.timed_out = False

        if self.message_type == "odometry":
            self.create_subscription(Odometry, self.topic, self.handle_msg, 10)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self.topic, self.handle_msg, 10
            )

        self.create_timer(self.timeout_sec, self.handle_timeout)
        self.get_logger().info(
            f"Waiting for first {self.message_type} message on {self.topic}"
        )

    def handle_msg(self, _msg) -> None:
        if self.received:
            return
        self.received = True
        self.get_logger().info(f"Topic ready: {self.topic}")
        raise SystemExit(0)

    def handle_timeout(self) -> None:
        if self.received or self.timed_out:
            return
        self.timed_out = True
        self.get_logger().error(
            f"Timeout waiting for {self.topic}; navigation launch remains blocked"
        )


def main() -> None:
    rclpy.init()
    node = TopicReadyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(exc.code)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
