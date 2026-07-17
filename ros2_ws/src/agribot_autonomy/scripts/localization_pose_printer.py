#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocalizationPosePrinter(Node):
    def __init__(self) -> None:
        super().__init__("localization_pose_printer")
        self.topic = self.declare_parameter("topic", "/amcl_pose").value
        self.message_type = self.declare_parameter("message_type", "pose").value
        self.print_rate = float(self.declare_parameter("print_rate", 2.0).value)
        self.label = self.declare_parameter("label", "Localization").value
        self.latest = None

        if self.message_type == "odometry":
            self.create_subscription(Odometry, self.topic, self.handle_msg, 20)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.topic, self.handle_msg, 20)

        self.create_timer(1.0 / max(self.print_rate, 1e-3), self.handle_timer)

    def handle_msg(self, msg) -> None:
        self.latest = msg

    def handle_timer(self) -> None:
        if self.latest is None:
            return

        header = self.latest.header
        if hasattr(self.latest, "pose") and hasattr(self.latest.pose, "pose"):
            pose = self.latest.pose.pose
        else:
            return

        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.get_logger().info(
            f"{self.label} pose [{header.frame_id or 'unknown'}]: "
            f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
            f"z={pose.position.z:.3f} yaw={yaw:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = LocalizationPosePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
