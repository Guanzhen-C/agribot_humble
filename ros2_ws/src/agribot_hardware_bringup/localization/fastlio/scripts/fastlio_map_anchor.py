#!/usr/bin/env python3

import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_matrix(quaternion: List[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-12:
        raise ValueError("pose quaternion must not be zero")
    scale = 2.0 / norm
    matrix = np.identity(4)
    matrix[0, 0] = 1.0 - scale * (y * y + z * z)
    matrix[0, 1] = scale * (x * y - z * w)
    matrix[0, 2] = scale * (x * z + y * w)
    matrix[1, 0] = scale * (x * y + z * w)
    matrix[1, 1] = 1.0 - scale * (x * x + z * z)
    matrix[1, 2] = scale * (y * z - x * w)
    matrix[2, 0] = scale * (x * z - y * w)
    matrix[2, 1] = scale * (y * z + x * w)
    matrix[2, 2] = 1.0 - scale * (x * x + y * y)
    return matrix


def quaternion_from_matrix(matrix: np.ndarray) -> List[float]:
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return [
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ]
    axis = int(np.argmax(np.diag(rotation)))
    if axis == 0:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        return [
            0.25 * scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
        ]
    if axis == 1:
        scale = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        ) * 2.0
        return [
            (rotation[0, 1] + rotation[1, 0]) / scale,
            0.25 * scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
        ]
    scale = math.sqrt(
        1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
    ) * 2.0
    return [
        (rotation[0, 2] + rotation[2, 0]) / scale,
        (rotation[1, 2] + rotation[2, 1]) / scale,
        0.25 * scale,
        (rotation[1, 0] - rotation[0, 1]) / scale,
    ]


def pose_matrix(position, orientation) -> np.ndarray:
    transform = quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    transform[:3, 3] = [position.x, position.y, position.z]
    return transform


def configured_pose(values: List[float]) -> np.ndarray:
    if len(values) not in (3, 6):
        raise ValueError(
            "initial_pose must be [x, y, yaw] or [x, y, z, roll, pitch, yaw]"
        )
    if len(values) == 3:
        x, y, yaw = values
        z = roll = pitch = 0.0
    else:
        x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transform = np.identity(4)
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = [x, y, z]
    return transform


class FastLioMapAnchor(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_map_anchor")
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/fastlio/odometry"
        ).value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/localization_pose"
        ).value
        self.initial_pose_topic = self.declare_parameter(
            "initial_pose_topic", "/initialpose"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.allow_reinitialization = bool(
            self.declare_parameter("allow_reinitialization", True).value
        )
        self.stamp_with_current_time = bool(
            self.declare_parameter("stamp_with_current_time", False).value
        )
        initial_pose = [
            float(value)
            for value in self.declare_parameter(
                "initial_pose", [0.0, 0.0, 0.0]
            ).value
        ]

        self.pending_map_to_base: Optional[np.ndarray] = configured_pose(initial_pose)
        self.latest_odom_to_base: Optional[np.ndarray] = None
        self.map_to_odom: Optional[np.ndarray] = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 20
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.handle_odometry, 50
        )
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.handle_initial_pose,
            10,
        )
        self.get_logger().info(
            f"Anchoring {self.odom_topic} to {self.map_frame}; "
            "the static map is not used to correct FAST-LIO"
        )

    def handle_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        if self.map_to_odom is not None and not self.allow_reinitialization:
            return
        try:
            self.pending_map_to_base = pose_matrix(
                message.pose.pose.position, message.pose.pose.orientation
            )
        except ValueError as error:
            self.get_logger().warning(f"Rejected initial pose: {error}")
            return
        self.initialize_anchor()

    def handle_odometry(self, message: Odometry) -> None:
        try:
            self.latest_odom_to_base = pose_matrix(
                message.pose.pose.position, message.pose.pose.orientation
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignored invalid odometry: {error}")
            return
        self.initialize_anchor()
        if self.map_to_odom is None:
            return

        stamp = (
            self.get_clock().now().to_msg()
            if self.stamp_with_current_time
            else message.header.stamp
        )
        self.publish_transform(stamp)
        self.publish_pose(stamp, message)

    def initialize_anchor(self) -> None:
        if (
            self.pending_map_to_base is None
            or self.latest_odom_to_base is None
        ):
            return
        self.map_to_odom = (
            self.pending_map_to_base @ np.linalg.inv(self.latest_odom_to_base)
        )
        self.pending_map_to_base = None
        self.get_logger().info(
            "Initialized map anchor from the current FAST-LIO pose"
        )

    def publish_transform(self, stamp) -> None:
        quaternion = quaternion_from_matrix(self.map_to_odom)
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.odom_frame
        message.transform.translation.x = self.map_to_odom[0, 3]
        message.transform.translation.y = self.map_to_odom[1, 3]
        message.transform.translation.z = self.map_to_odom[2, 3]
        message.transform.rotation.x = quaternion[0]
        message.transform.rotation.y = quaternion[1]
        message.transform.rotation.z = quaternion[2]
        message.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(message)

    def publish_pose(self, stamp, odometry: Odometry) -> None:
        map_to_base = self.map_to_odom @ self.latest_odom_to_base
        quaternion = quaternion_from_matrix(map_to_base)
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = map_to_base[0, 3]
        message.pose.pose.position.y = map_to_base[1, 3]
        message.pose.pose.position.z = map_to_base[2, 3]
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.pose.covariance = odometry.pose.covariance
        self.pose_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = FastLioMapAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
