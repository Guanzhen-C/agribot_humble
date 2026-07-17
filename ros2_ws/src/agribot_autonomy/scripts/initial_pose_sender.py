#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class InitialPoseSender(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_sender")

        self.pose_x = float(self.declare_parameter("x", 0.0).value)
        self.pose_y = float(self.declare_parameter("y", 0.0).value)
        self.pose_z = float(self.declare_parameter("z", 0.0).value)
        self.pose_yaw = float(self.declare_parameter("yaw", 0.0).value)
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.topic = self.declare_parameter("topic", "/initialpose").value
        self.startup_delay = float(self.declare_parameter("startup_delay", 2.0).value)
        self.publish_count = int(self.declare_parameter("publish_count", 5).value)
        self.publish_interval = float(self.declare_parameter("publish_interval", 0.5).value)
        self.covariance_xy = float(self.declare_parameter("covariance_xy", 0.25).value)
        self.covariance_yaw = float(self.declare_parameter("covariance_yaw", 0.1).value)
        self.transient_local = bool(self.declare_parameter("transient_local", True).value)
        self.use_zero_stamp = bool(self.declare_parameter("use_zero_stamp", False).value)
        self.stamp_offset_sec = float(self.declare_parameter("stamp_offset_sec", -0.1).value)
        self.stamp_from_topic = self.declare_parameter("stamp_from_topic", "").value
        self.latest_topic_stamp = None

        durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
            if self.transient_local
            else DurabilityPolicy.VOLATILE
        )
        qos = QoSProfile(depth=1, durability=durability)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, qos)
        self.stamp_subscription = None
        if self.stamp_from_topic:
            self.stamp_subscription = self.create_subscription(
                Odometry,
                self.stamp_from_topic,
                self._stamp_callback,
                10,
            )
        self.publish_index = 0
        self.started = False
        self.deadline = self.get_clock().now() + Duration(seconds=self.startup_delay)
        self.timer = self.create_timer(self.publish_interval, self.tick)

    def _stamp_callback(self, msg: Odometry) -> None:
        self.latest_topic_stamp = msg.header.stamp

    def tick(self) -> None:
        if not self.started and self.get_clock().now() < self.deadline:
            return
        self.started = True
        if self.stamp_from_topic and self.latest_topic_stamp is None:
            return

        msg = PoseWithCovarianceStamped()
        if self.stamp_from_topic and self.latest_topic_stamp is not None:
            msg.header.stamp = self.latest_topic_stamp
        elif not self.use_zero_stamp:
            stamp_time = self.get_clock().now() + Duration(seconds=self.stamp_offset_sec)
            if stamp_time.nanoseconds < 0:
                stamp_time = self.get_clock().now()
            msg.header.stamp = stamp_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = self.pose_z
        msg.pose.pose.orientation.z = math.sin(self.pose_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.pose_yaw / 2.0)
        msg.pose.covariance[0] = self.covariance_xy
        msg.pose.covariance[7] = self.covariance_xy
        msg.pose.covariance[35] = self.covariance_yaw
        self.publisher.publish(msg)
        self.publish_index += 1

        if self.publish_index >= max(1, self.publish_count):
            self.get_logger().info(
                f"Published initial pose to {self.topic}: "
                f"({self.pose_x:.2f}, {self.pose_y:.2f}, {self.pose_z:.2f}, {self.pose_yaw:.2f})"
            )
            self.destroy_timer(self.timer)


def main() -> None:
    rclpy.init()
    node = InitialPoseSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
