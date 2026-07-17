#!/usr/bin/env python3

import math
from typing import Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Transform = Tuple[Vector3, Quaternion]


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quaternion)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_inverse(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_transform(left: Transform, right: Transform) -> Transform:
    left_position, left_orientation = left
    right_position, right_orientation = right
    rotated_position = rotate_vector(left_orientation, right_position)
    return (
        tuple(left_position[i] + rotated_position[i] for i in range(3)),
        quaternion_multiply(left_orientation, right_orientation),
    )


def inverse_transform(transform: Transform) -> Transform:
    position, orientation = transform
    inverse_orientation = quaternion_inverse(orientation)
    return (
        rotate_vector(
            inverse_orientation,
            (-position[0], -position[1], -position[2]),
        ),
        inverse_orientation,
    )


def pose_transform(pose) -> Transform:
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        normalize_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
    )


class NavSatPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("navsat_pose_bridge")
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/odometry/filtered_navsat"
        ).value
        self.pose_topic = self.declare_parameter("pose_topic", "/amcl_pose").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tf_mode = self.declare_parameter("tf_mode", "map_to_odom").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, 20)

    def handle_odom(self, msg: Odometry) -> None:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        self.pose_pub.publish(pose_msg)

        if self.tf_mode == "odom_to_base":
            self.publish_identity_map_to_odom(msg)
            self.publish_odom_to_base(msg)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                return

        map_to_base = pose_transform(msg.pose.pose)
        odom_to_base = (
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            normalize_quaternion(
                (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
            ),
        )
        map_to_odom_position, map_to_odom_orientation = compose_transform(
            map_to_base,
            inverse_transform(odom_to_base),
        )

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = map_to_odom_position[0]
        tf_msg.transform.translation.y = map_to_odom_position[1]
        tf_msg.transform.translation.z = map_to_odom_position[2]
        tf_msg.transform.rotation.x = map_to_odom_orientation[0]
        tf_msg.transform.rotation.y = map_to_odom_orientation[1]
        tf_msg.transform.rotation.z = map_to_odom_orientation[2]
        tf_msg.transform.rotation.w = map_to_odom_orientation[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_identity_map_to_odom(self, msg: Odometry) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_odom_to_base(self, msg: Odometry) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = NavSatPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
