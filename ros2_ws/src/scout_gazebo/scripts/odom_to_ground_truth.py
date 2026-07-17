#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToGroundTruth(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_ground_truth")
        input_topic = self.declare_parameter("input_topic", "/odom").value
        output_topic = self.declare_parameter(
            "output_topic", "/base_pose_ground_truth"
        ).value
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.handle_odom, 10
        )

    def handle_odom(self, message: Odometry) -> None:
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = OdomToGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
