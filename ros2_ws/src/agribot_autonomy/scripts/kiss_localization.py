#!/usr/bin/env python3

import math
from typing import Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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
    inverse_position = rotate_vector(
        inverse_orientation,
        (-position[0], -position[1], -position[2]),
    )
    return inverse_position, inverse_orientation


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


class KissLocalization(Node):
    def __init__(self) -> None:
        super().__init__("kiss_localization")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_topic = self.declare_parameter("odom_topic", "/kiss/odometry").value
        self.base_odom_topic = self.declare_parameter("base_odom_topic", "").value
        self.initial_pose_topic = self.declare_parameter("initial_pose_topic", "/initialpose").value
        self.pose_topic = self.declare_parameter("pose_topic", "/amcl_pose").value
        self.default_initial_x = float(self.declare_parameter("initial_pose_x", 0.0).value)
        self.default_initial_y = float(self.declare_parameter("initial_pose_y", 0.0).value)
        self.default_initial_z = float(self.declare_parameter("initial_pose_z", 0.0).value)
        self.default_initial_roll = float(self.declare_parameter("initial_pose_roll", 0.0).value)
        self.default_initial_pitch = float(self.declare_parameter("initial_pose_pitch", 0.0).value)
        self.default_initial_yaw = float(self.declare_parameter("initial_pose_yaw", 0.0).value)
        self.planar_mode = bool(self.declare_parameter("planar_mode", True).value)
        self.use_auto_initial_pose = bool(
            self.declare_parameter("use_auto_initial_pose", False).value
        )
        self.auto_initial_pose_topic = self.declare_parameter(
            "auto_initial_pose_topic", ""
        ).value
        self.auto_initial_pose_message_type = self.declare_parameter(
            "auto_initial_pose_message_type", "pose"
        ).value
        self.allow_reinitialization = bool(
            self.declare_parameter("allow_reinitialization", False).value
        )
        self.stamp_with_current_time = bool(
            self.declare_parameter("stamp_with_current_time", False).value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.handle_odom, 20
        )
        self.base_odom_subscription = None
        if self.base_odom_topic:
            self.base_odom_subscription = self.create_subscription(
                Odometry, self.base_odom_topic, self.handle_base_odom, 20
            )
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.handle_initial_pose,
            10,
        )
        self.auto_initial_pose_subscription = None
        if self.use_auto_initial_pose and self.auto_initial_pose_topic:
            if self.auto_initial_pose_message_type == "odometry":
                self.auto_initial_pose_subscription = self.create_subscription(
                    Odometry,
                    self.auto_initial_pose_topic,
                    self.handle_auto_initial_odom,
                    10,
                )
            else:
                self.auto_initial_pose_subscription = self.create_subscription(
                    PoseWithCovarianceStamped,
                    self.auto_initial_pose_topic,
                    self.handle_auto_initial_pose,
                    10,
                )

        self.latest_odom: Optional[Odometry] = None
        self.latest_base_odom: Optional[Odometry] = None
        self.reported_waiting_for_base_odom = False
        self.pending_initial_pose = None
        if not self.use_auto_initial_pose:
            self.pending_initial_pose = (
                (
                    self.default_initial_x,
                    self.default_initial_y,
                    0.0 if self.planar_mode else self.default_initial_z,
                ),
                quaternion_from_yaw(self.default_initial_yaw)
                if self.planar_mode
                else quaternion_from_rpy(
                    self.default_initial_roll,
                    self.default_initial_pitch,
                    self.default_initial_yaw,
                ),
            )
        self.map_to_odom = None
        self.auto_initial_pose_applied = False

        self.get_logger().info(
            f"Using {self.odom_topic} as localization source, publishing "
            f"{self.map_frame}->{self.odom_frame} in "
            f"{'planar' if self.planar_mode else '6-DoF'} mode"
        )
        if self.base_odom_topic:
            self.get_logger().info(
                f"Using {self.base_odom_topic} as the {self.odom_frame}->{self.base_frame} odometry source"
            )
        if self.use_auto_initial_pose and self.auto_initial_pose_topic:
            self.get_logger().info(
                "Waiting for automatic initial pose from "
                f"{self.auto_initial_pose_topic} ({self.auto_initial_pose_message_type})"
            )

    def handle_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.map_to_odom is not None and not self.allow_reinitialization:
            return
        pose = msg.pose.pose
        self.pending_initial_pose = self.transform_from_pose(pose)
        self.try_initialize_transform()

    def handle_auto_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.auto_initial_pose_applied:
            return
        if self.map_to_odom is not None and not self.allow_reinitialization:
            return
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        self.pending_initial_pose = self.transform_from_pose(pose)
        self.auto_initial_pose_applied = True
        self.get_logger().info(
            "Automatic initial pose received from "
            f"{self.auto_initial_pose_topic}: "
            f"({pose.position.x:.2f}, {pose.position.y:.2f}, {yaw:.2f})"
        )
        self.try_initialize_transform()

    def handle_auto_initial_odom(self, msg: Odometry) -> None:
        if self.auto_initial_pose_applied:
            return
        if self.map_to_odom is not None and not self.allow_reinitialization:
            return
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        self.pending_initial_pose = self.transform_from_pose(pose)
        self.auto_initial_pose_applied = True
        self.get_logger().info(
            "Automatic initial odom pose received from "
            f"{self.auto_initial_pose_topic}: "
            f"({pose.position.x:.2f}, {pose.position.y:.2f}, {yaw:.2f})"
        )
        self.try_initialize_transform()

    def handle_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.try_initialize_transform()
        if self.map_to_odom is None:
            return
        self.publish_localization(msg)

    def handle_base_odom(self, msg: Odometry) -> None:
        self.latest_base_odom = msg

    def try_initialize_transform(self) -> None:
        if self.latest_odom is None or self.pending_initial_pose is None:
            return

        initial_transform = self.pending_initial_pose
        odom_transform = self.transform_from_pose(self.latest_odom.pose.pose)
        self.map_to_odom = compose_transform(
            initial_transform,
            inverse_transform(odom_transform),
        )
        self.pending_initial_pose = None
        initial_position, initial_orientation = initial_transform
        initial_yaw = yaw_from_quaternion(initial_orientation)
        self.get_logger().info(
            "Initialized KISS localization anchor at "
            f"xyz=({initial_position[0]:.2f}, {initial_position[1]:.2f}, "
            f"{initial_position[2]:.2f}) yaw={initial_yaw:.2f}"
        )

    def publish_localization(self, odom_msg: Odometry) -> None:
        stamp = self.get_clock().now().to_msg() if self.stamp_with_current_time else odom_msg.header.stamp
        map_to_base = compose_transform(
            self.map_to_odom,
            self.transform_from_pose(odom_msg.pose.pose),
        )

        base_odom_msg = self.latest_base_odom if self.base_odom_topic else odom_msg
        if base_odom_msg is None:
            if not self.reported_waiting_for_base_odom:
                self.get_logger().warn(
                    f"Waiting for base odometry on {self.base_odom_topic} before publishing TF"
                )
                self.reported_waiting_for_base_odom = True
            return

        dynamic_map_to_odom = compose_transform(
            map_to_base,
            inverse_transform(self.transform_from_pose(base_odom_msg.pose.pose)),
        )
        map_to_odom_position, map_to_odom_orientation = dynamic_map_to_odom
        map_position, map_orientation = map_to_base

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
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

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.pose.position.x = map_position[0]
        pose_msg.pose.pose.position.y = map_position[1]
        pose_msg.pose.pose.position.z = map_position[2]
        pose_msg.pose.pose.orientation.x = map_orientation[0]
        pose_msg.pose.pose.orientation.y = map_orientation[1]
        pose_msg.pose.pose.orientation.z = map_orientation[2]
        pose_msg.pose.pose.orientation.w = map_orientation[3]
        pose_msg.pose.covariance = odom_msg.pose.covariance
        self.pose_publisher.publish(pose_msg)

    def transform_from_pose(self, pose) -> Transform:
        yaw = yaw_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        if self.planar_mode:
            return (
                (pose.position.x, pose.position.y, 0.0),
                quaternion_from_yaw(yaw),
            )
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


def main() -> None:
    rclpy.init()
    node = KissLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
