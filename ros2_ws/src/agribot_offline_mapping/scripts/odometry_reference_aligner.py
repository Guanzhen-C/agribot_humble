#!/usr/bin/env python3

import math
from collections import deque

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in quaternion)


def multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return normalize((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def inverse(quaternion):
    x, y, z, w = normalize(quaternion)
    return (-x, -y, -z, w)


def rotate(quaternion, vector):
    x, y, z, w = normalize(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_pose(left_position, left_orientation, right_position, right_orientation):
    rotated = rotate(left_orientation, right_position)
    return (
        tuple(left_position[index] + rotated[index] for index in range(3)),
        multiply(left_orientation, right_orientation),
    )


def inverse_pose(position, orientation):
    orientation_inverse = inverse(orientation)
    return (
        rotate(orientation_inverse, tuple(-value for value in position)),
        orientation_inverse,
    )


def odometry_pose(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return (
        (position.x, position.y, position.z),
        normalize((orientation.x, orientation.y, orientation.z, orientation.w)),
    )


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


class OdometryReferenceAligner(Node):
    def __init__(self):
        super().__init__("odometry_reference_aligner")
        source_topic = self.declare_parameter(
            "source_topic", "/comparison/fastlio/odometry"
        ).value
        reference_topic = self.declare_parameter(
            "reference_topic", "/comparison/robot_localization/rtk_odom"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/comparison/robot_localization/fastlio_odom"
        ).value
        self.output_frame = self.declare_parameter("output_frame", "enu").value
        self.output_child_frame = self.declare_parameter(
            "output_child_frame", "base_link"
        ).value
        self.maximum_time_delta = float(
            self.declare_parameter("maximum_time_delta_sec", 0.15).value
        )
        self.position_variance = float(
            self.declare_parameter("position_variance_m2", 0.0025).value
        )
        yaw_std_deg = float(
            self.declare_parameter("yaw_std_deg", 0.5).value
        )
        self.yaw_variance = math.radians(yaw_std_deg) ** 2
        if (
            self.maximum_time_delta <= 0.0
            or self.position_variance <= 0.0
            or yaw_std_deg <= 0.0
        ):
            raise ValueError("alignment timing and covariance parameters must be positive")

        self.source_buffer = deque(maxlen=100)
        self.reference_buffer = deque(maxlen=20)
        self.reference_from_source = None
        self.publisher = self.create_publisher(Odometry, output_topic, 50)
        self.create_subscription(Odometry, source_topic, self.handle_source, 100)
        self.create_subscription(Odometry, reference_topic, self.handle_reference, 50)
        self.get_logger().info(
            f"Waiting to align {source_topic} to {self.output_frame} from "
            f"{reference_topic}"
        )

    def handle_source(self, message):
        self.source_buffer.append(message)
        self.try_initialize()
        if self.reference_from_source is not None:
            self.publish_aligned(message)

    def handle_reference(self, message):
        self.reference_buffer.append(message)
        self.try_initialize()

    def try_initialize(self):
        if self.reference_from_source is not None:
            return
        candidates = []
        for source in self.source_buffer:
            source_stamp = stamp_seconds(source)
            for reference in self.reference_buffer:
                delta = abs(source_stamp - stamp_seconds(reference))
                if delta <= self.maximum_time_delta:
                    candidates.append((delta, source, reference))
        if not candidates:
            return
        delta, source, reference = min(candidates, key=lambda item: item[0])
        reference_from_base = odometry_pose(reference)
        source_from_base = odometry_pose(source)
        self.reference_from_source = compose_pose(
            *reference_from_base, *inverse_pose(*source_from_base)
        )
        self.get_logger().info(
            "Initialized ENU alignment from RTK and FAST-LIO2 "
            f"with {delta:.3f} s timestamp separation"
        )

    def publish_aligned(self, message):
        position, orientation = compose_pose(
            *self.reference_from_source, *odometry_pose(message)
        )
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.output_frame
        output.child_frame_id = self.output_child_frame
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation.x = orientation[0]
        output.pose.pose.orientation.y = orientation[1]
        output.pose.pose.orientation.z = orientation[2]
        output.pose.pose.orientation.w = orientation[3]
        output.pose.covariance = [0.0] * 36
        output.pose.covariance[0] = self.position_variance
        output.pose.covariance[7] = self.position_variance
        output.pose.covariance[14] = self.position_variance
        output.pose.covariance[21] = 1.0e6
        output.pose.covariance[28] = 1.0e6
        output.pose.covariance[35] = self.yaw_variance
        output.twist = message.twist
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = OdometryReferenceAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
