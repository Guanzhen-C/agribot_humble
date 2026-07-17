#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdometryTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odometry_tf_broadcaster")
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/filtered").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.stamp_with_current_time = bool(
            self.declare_parameter("stamp_with_current_time", False).value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, 20)

    def handle_odom(self, msg: Odometry) -> None:
        stamp = self.get_clock().now().to_msg() if self.stamp_with_current_time else msg.header.stamp
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = OdometryTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
