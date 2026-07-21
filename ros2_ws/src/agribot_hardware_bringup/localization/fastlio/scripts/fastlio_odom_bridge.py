#!/usr/bin/env python3

from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def vector_param(node: Node, name: str, default: List[float]) -> List[float]:
    return [float(value) for value in node.declare_parameter(name, default).value]


def quaternion_matrix(quaternion: List[float]) -> np.ndarray:
    x, y, z, w = quaternion
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.identity(4)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    matrix = np.identity(4)
    matrix[0, 0] = 1.0 - (yy + zz)
    matrix[0, 1] = xy - wz
    matrix[0, 2] = xz + wy
    matrix[1, 0] = xy + wz
    matrix[1, 1] = 1.0 - (xx + zz)
    matrix[1, 2] = yz - wx
    matrix[2, 0] = xz - wy
    matrix[2, 1] = yz + wx
    matrix[2, 2] = 1.0 - (xx + yy)
    return matrix


def quaternion_from_matrix(matrix: np.ndarray) -> List[float]:
    m = matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def compose_matrix(translation: List[float], rpy: List[float]) -> np.ndarray:
    matrix = np.identity(4)
    matrix[:3, :3] = rotation_matrix_from_rpy(rpy[0], rpy[1], rpy[2])
    matrix[:3, 3] = np.array(translation)
    return matrix


def inverse_transform(matrix: np.ndarray) -> np.ndarray:
    inverse = np.identity(4)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


class FastLioOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_odom_bridge")

        self.input_odom_topic = self.declare_parameter("input_odom_topic", "/Odometry").value
        self.output_odom_topic = self.declare_parameter(
            "output_odom_topic", "/fastlio/odometry"
        ).value
        self.input_odom_frame = self.declare_parameter("input_odom_frame", "camera_init").value
        self.input_body_frame = self.declare_parameter("input_body_frame", "body").value
        self.output_odom_frame = self.declare_parameter("output_odom_frame", "odom").value
        self.output_base_frame = self.declare_parameter("output_base_frame", "base_link").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.stamp_with_current_time = bool(
            self.declare_parameter("stamp_with_current_time", False).value
        )
        self.is_simulation = bool(self.declare_parameter("is_simulation", False).value)

        if self.is_simulation:
            # In simulation, Gazebo's IMU plugin compensates for gravity,
            # so FAST-LIO's body frame starts nearly horizontal (identity).
            # Only the IMU-to-base_link translation offset is needed.
            base_to_body_xyz = vector_param(self, "base_to_body_xyz", [0.19, 0.0, 0.149])
            base_to_body_rpy = vector_param(self, "base_to_body_rpy", [0.0, 0.0, 0.0])
        else:
            base_to_body_xyz = vector_param(self, "base_to_body_xyz", [0.19, 0.0, 0.149])
            base_to_body_rpy = vector_param(self, "base_to_body_rpy", [0.0, -1.5708, 3.1416])

        self.base_to_body = compose_matrix(base_to_body_xyz, base_to_body_rpy)
        self.body_to_base = inverse_transform(self.base_to_body)
        self.rotation_base_body = self.base_to_body[:3, :3]
        self.translation_body_base = self.body_to_base[:3, 3]

        self.publisher = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, self.input_odom_topic, self.handle_odom, 20
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f"Bridging {self.input_odom_topic} ({self.input_odom_frame}->{self.input_body_frame}) "
            f"to {self.output_odom_topic} ({self.output_odom_frame}->{self.output_base_frame})"
        )

    def handle_odom(self, msg: Odometry) -> None:
        stamp = (
            self.get_clock().now().to_msg()
            if self.stamp_with_current_time
            else msg.header.stamp
        )

        input_quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        odom_to_body = quaternion_matrix(input_quat)
        odom_to_body[:3, 3] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        odom_to_base = odom_to_body @ self.body_to_base

        output_quat = quaternion_from_matrix(odom_to_base)
        output_xyz = odom_to_base[:3, 3]

        angular_body = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        linear_body = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        angular_base = self.rotate_vector_body_to_base(angular_body)
        linear_base = self.rotate_vector_body_to_base(
            self.subtract_vectors(
                linear_body, self.cross_product(angular_body, self.translation_body_base)
            )
        )

        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self.output_odom_frame
        output.child_frame_id = self.output_base_frame
        output.pose.pose.position.x = output_xyz[0]
        output.pose.pose.position.y = output_xyz[1]
        output.pose.pose.position.z = output_xyz[2]
        output.pose.pose.orientation.x = output_quat[0]
        output.pose.pose.orientation.y = output_quat[1]
        output.pose.pose.orientation.z = output_quat[2]
        output.pose.pose.orientation.w = output_quat[3]
        output.pose.covariance = msg.pose.covariance
        output.twist.twist.linear.x = linear_base[0]
        output.twist.twist.linear.y = linear_base[1]
        output.twist.twist.linear.z = linear_base[2]
        output.twist.twist.angular.x = angular_base[0]
        output.twist.twist.angular.y = angular_base[1]
        output.twist.twist.angular.z = angular_base[2]
        output.twist.covariance = msg.twist.covariance
        self.publisher.publish(output)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.output_odom_frame
            tf_msg.child_frame_id = self.output_base_frame
            tf_msg.transform.translation.x = output_xyz[0]
            tf_msg.transform.translation.y = output_xyz[1]
            tf_msg.transform.translation.z = output_xyz[2]
            tf_msg.transform.rotation.x = output_quat[0]
            tf_msg.transform.rotation.y = output_quat[1]
            tf_msg.transform.rotation.z = output_quat[2]
            tf_msg.transform.rotation.w = output_quat[3]
            self.tf_broadcaster.sendTransform(tf_msg)

    def rotate_vector_body_to_base(self, vector: List[float]) -> List[float]:
        return [
            sum(self.rotation_base_body[row][col] * vector[col] for col in range(3))
            for row in range(3)
        ]

    @staticmethod
    def subtract_vectors(a: List[float], b: List[float]) -> List[float]:
        return [a[i] - b[i] for i in range(3)]

    @staticmethod
    def cross_product(a: List[float], b: List[float]) -> List[float]:
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]


def main() -> None:
    rclpy.init()
    node = FastLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
