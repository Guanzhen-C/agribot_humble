#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GroundTruthDisplayBridge(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_display_bridge")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.pose_topic = self.declare_parameter("pose_topic", "/amcl_pose").value
        self.stop_topic = self.declare_parameter("stop_topic", "/fastlio_pose").value
        self.stop_topic_type = self.declare_parameter("stop_topic_type", "pose").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.initial_pose_x = float(self.declare_parameter("initial_pose_x", 0.0).value)
        self.initial_pose_y = float(self.declare_parameter("initial_pose_y", 0.0).value)
        self.initial_pose_z = float(self.declare_parameter("initial_pose_z", 0.0).value)
        self.initial_pose_yaw = float(self.declare_parameter("initial_pose_yaw", 0.0).value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.create_subscription(Odometry, self.ground_truth_topic, self.handle_ground_truth, 10)
        if self.stop_topic_type == "odometry":
            self.create_subscription(Odometry, self.stop_topic, self.handle_stop, 10)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self.stop_topic, self.handle_stop, 10
            )
        self.create_timer(1.0 / max(self.publish_rate, 1e-3), self.handle_publish_timer)

        self.latest_pose = Pose()
        self.latest_pose.position.x = self.initial_pose_x
        self.latest_pose.position.y = self.initial_pose_y
        self.latest_pose.position.z = self.initial_pose_z
        quat = quaternion_from_yaw(self.initial_pose_yaw)
        self.latest_pose.orientation.x = quat[0]
        self.latest_pose.orientation.y = quat[1]
        self.latest_pose.orientation.z = quat[2]
        self.latest_pose.orientation.w = quat[3]
        self.stopped = False

        self.get_logger().info(
            "Publishing startup display pose at "
            f"({self.initial_pose_x:.2f}, {self.initial_pose_y:.2f}, "
            f"{self.initial_pose_z:.2f}, {self.initial_pose_yaw:.2f}) "
            f"until {self.ground_truth_topic} arrives"
        )

    def handle_ground_truth(self, msg: Odometry) -> None:
        if self.stopped:
            return
        self.latest_pose = msg.pose.pose

    def handle_stop(self, _msg: Odometry) -> None:
        if self.stopped:
            return
        self.stopped = True
        self.get_logger().info(
            f"Stopping local display bridge after remote odom appeared on {self.stop_topic}"
        )

    def handle_publish_timer(self) -> None:
        if self.stopped or self.latest_pose is None:
            return

        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.latest_pose.position.x
        tf_msg.transform.translation.y = self.latest_pose.position.y
        tf_msg.transform.translation.z = self.latest_pose.position.z
        tf_msg.transform.rotation = self.latest_pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.pose = self.latest_pose
        pose_msg.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02,
        ]
        self.pose_pub.publish(pose_msg)


def main() -> None:
    rclpy.init()
    node = GroundTruthDisplayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
