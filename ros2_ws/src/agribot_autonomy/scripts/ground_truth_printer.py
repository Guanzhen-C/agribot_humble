#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return __import__("math").atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GroundTruthPrinter(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_printer")
        self.topic = self.declare_parameter("topic", "/base_pose_ground_truth").value
        self.print_rate = float(self.declare_parameter("print_rate", 2.0).value)
        self.latest = None

        self.create_subscription(Odometry, self.topic, self.handle_msg, 10)
        self.create_timer(1.0 / max(self.print_rate, 1e-3), self.handle_timer)

    def handle_msg(self, msg: Odometry) -> None:
        self.latest = msg

    def handle_timer(self) -> None:
        if self.latest is None:
            return

        pose = self.latest.pose.pose
        twist = self.latest.twist.twist
        yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.get_logger().info(
            f"Ground truth pose [{self.latest.header.frame_id or 'unknown'}]: "
            f"x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f} "
            f"yaw={yaw:.3f} vx={twist.linear.x:.3f} wz={twist.angular.z:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = GroundTruthPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
