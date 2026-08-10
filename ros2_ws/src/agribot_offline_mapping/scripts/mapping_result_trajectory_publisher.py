#!/usr/bin/env python3

import bisect
import math
from pathlib import Path
import sqlite3

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


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


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def message_quaternion(message):
    return normalize((message.x, message.y, message.z, message.w))


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


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
    return (
        (position.x, position.y, position.z),
        message_quaternion(message.pose.pose.orientation),
    )


def sensor_odometry_pose_to_base(message, sensor_mount):
    base_from_sensor = (
        tuple(float(value) for value in sensor_mount["xyz"]),
        quaternion_from_rpy(
            *(float(value) for value in sensor_mount["rpy"])
        ),
    )
    sensor_from_base = inverse_pose(*base_from_sensor)
    return compose_pose(*odometry_pose(message), *sensor_from_base)


def stamped_pose(message):
    position = message.pose.position
    return (
        (position.x, position.y, position.z),
        message_quaternion(message.pose.orientation),
    )


def result_databases(result_bag):
    databases = sorted(result_bag.glob("*.db3"))
    if not databases:
        raise RuntimeError(f"result bag contains no db3 file: {result_bag}")
    return databases


def topic_messages(result_bag, topic_name):
    messages = []
    for database in result_databases(result_bag):
        connection = sqlite3.connect(database)
        topic = connection.execute(
            "SELECT id, type FROM topics WHERE name = ?", (topic_name,)
        ).fetchone()
        if topic is None:
            connection.close()
            continue
        topic_id, topic_type = topic
        message_type = get_message(topic_type)
        rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
        messages.extend(
            (timestamp, deserialize_message(serialized, message_type))
            for timestamp, serialized in rows
        )
        connection.close()
    messages.sort(key=lambda item: item[0])
    if not messages:
        raise RuntimeError(f"result bag topic is empty: {topic_name}")
    return messages


def pose_stamped(source, position, orientation, flatten):
    result = PoseStamped()
    result.header = source.header
    result.header.frame_id = "map"
    result.pose.position.x = position[0]
    result.pose.position.y = position[1]
    result.pose.position.z = 0.0 if flatten else position[2]
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = orientation
    return result


def load_lio_base_path(result_bag, mounts, flatten):
    source_path = topic_messages(result_bag, "/lio_sam/mapping/path")[-1][1]
    lidar_translation = tuple(float(value) for value in mounts["lidar"]["xyz"])
    base_from_lidar = quaternion_from_rpy(
        *(float(value) for value in mounts["lidar"]["rpy"])
    )
    path = PathMessage()
    path.header.frame_id = "map"
    for source in source_path.poses:
        map_from_lidar = message_quaternion(source.pose.orientation)
        map_from_base = multiply(map_from_lidar, inverse(base_from_lidar))
        base_to_lidar = rotate(map_from_base, lidar_translation)
        position = (
            source.pose.position.x - base_to_lidar[0],
            source.pose.position.y - base_to_lidar[1],
            source.pose.position.z - base_to_lidar[2],
        )
        path.poses.append(
            pose_stamped(source, position, map_from_base, flatten)
        )
    return path


def load_rtk_base_path(result_bag, mounts, georeference, flatten):
    headings = [
        (stamp_seconds(message), message_quaternion(message.pose.pose.orientation))
        for _, message in topic_messages(result_bag, "/lio_sam/odometry/heading")
    ]
    heading_stamps = [item[0] for item in headings]
    antenna_messages = topic_messages(
        result_bag, "/lio_sam/odometry/rtk_antenna"
    )
    translation = tuple(
        float(value) for value in georeference["map_from_enu"]["xyz"]
    )
    map_from_enu = quaternion_from_rpy(
        *(float(value) for value in georeference["map_from_enu"]["rpy"])
    )
    base_to_antenna = tuple(float(value) for value in mounts["rtk"]["xyz"])

    path = PathMessage()
    path.header.frame_id = "map"
    for _, message in antenna_messages:
        stamp = stamp_seconds(message)
        index = bisect.bisect_left(heading_stamps, stamp)
        candidates = [
            candidate
            for candidate in (index - 1, index)
            if 0 <= candidate < len(headings)
        ]
        if not candidates:
            continue
        nearest = min(
            candidates, key=lambda candidate: abs(heading_stamps[candidate] - stamp)
        )
        if abs(heading_stamps[nearest] - stamp) > 0.70:
            continue
        enu_from_base = headings[nearest][1]
        rotated_lever = rotate(enu_from_base, base_to_antenna)
        antenna = message.pose.pose.position
        enu_base = (
            antenna.x - rotated_lever[0],
            antenna.y - rotated_lever[1],
            antenna.z - rotated_lever[2],
        )
        rotated_position = rotate(map_from_enu, enu_base)
        map_position = tuple(
            translation[index] + rotated_position[index] for index in range(3)
        )
        map_from_base = multiply(map_from_enu, enu_from_base)
        source = PoseStamped()
        source.header = message.header
        path.poses.append(
            pose_stamped(source, map_position, map_from_base, flatten)
        )
    return path


def anchor_base_odometry(odometry, base_poses, lio_path, flatten, name):
    if not lio_path.poses:
        raise RuntimeError(
            f"LIO-SAM path is empty; {name} cannot be anchored"
        )
    if len(odometry) != len(base_poses):
        raise RuntimeError(f"{name} odometry and pose counts do not match")
    stamps = [stamp_seconds(message) for message in odometry]
    anchor = lio_path.poses[0]
    anchor_stamp = stamp_seconds(anchor)
    insertion = bisect.bisect_left(stamps, anchor_stamp)
    candidates = [
        index for index in (insertion - 1, insertion)
        if 0 <= index < len(odometry)
    ]
    nearest = min(candidates, key=lambda index: abs(stamps[index] - anchor_stamp))
    # LIO-SAM creates its first key pose before the other estimators finish
    # initialization. Both are still stationary during this startup interval.
    if abs(stamps[nearest] - anchor_stamp) > 5.0:
        raise RuntimeError(
            f"{name} and LIO-SAM have no common startup interval"
        )

    map_from_base = stamped_pose(anchor)
    odom_from_base = base_poses[nearest]
    base_from_odom = inverse_pose(*odom_from_base)
    map_from_odom = compose_pose(*map_from_base, *base_from_odom)

    path = PathMessage()
    path.header.frame_id = "map"
    for message, base_pose in zip(odometry, base_poses):
        map_pose = compose_pose(*map_from_odom, *base_pose)
        source = PoseStamped()
        source.header = message.header
        path.poses.append(pose_stamped(source, *map_pose, flatten))
    return path


def load_fastlio_base_path(comparison_bag, lio_path, flatten):
    odometry = [
        message for _, message in topic_messages(
            comparison_bag, "/comparison/fastlio/odometry"
        )
    ]
    return anchor_base_odometry(
        odometry,
        [odometry_pose(message) for message in odometry],
        lio_path,
        flatten,
        "FAST-LIO2",
    )


def load_fastlivo_base_path(comparison_bag, lio_path, mounts, flatten):
    odometry = [
        message for _, message in topic_messages(
            comparison_bag, "/comparison/fastlivo/odometry"
        )
    ]
    return anchor_base_odometry(
        odometry,
        [
            sensor_odometry_pose_to_base(message, mounts["imu"])
            for message in odometry
        ],
        lio_path,
        flatten,
        "FAST-LIVO2",
    )


def load_enu_base_path(comparison_bag, topic_name, georeference, flatten):
    odometry = topic_messages(
        comparison_bag, topic_name
    )
    map_from_enu = (
        tuple(float(value) for value in georeference["map_from_enu"]["xyz"]),
        quaternion_from_rpy(
            *(float(value) for value in georeference["map_from_enu"]["rpy"])
        ),
    )
    path = PathMessage()
    path.header.frame_id = "map"
    for _, message in odometry:
        map_pose = compose_pose(*map_from_enu, *odometry_pose(message))
        source = PoseStamped()
        source.header = message.header
        path.poses.append(pose_stamped(source, *map_pose, flatten))
    return path


def load_kf_gins_base_path(comparison_bag, georeference, flatten):
    return load_enu_base_path(
        comparison_bag,
        "/comparison/kf_gins/odometry",
        georeference,
        flatten,
    )


class MappingResultTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("mapping_result_trajectory_publisher")
        result_bag = Path(
            self.declare_parameter("result_bag", "").value
        ).expanduser()
        georeference_file = Path(
            self.declare_parameter("georeference_file", "").value
        ).expanduser()
        default_mounts = Path(
            get_package_share_directory("agribot_hardware_bringup")
        ) / "config" / "sensor_mounts.yaml"
        mounts_file = Path(
            self.declare_parameter("sensor_mounts_file", str(default_mounts)).value
        ).expanduser()
        comparison_bag_value = str(
            self.declare_parameter("comparison_bag", "").value
        ).strip()
        flatten = bool(self.declare_parameter("flatten_z", True).value)
        if not (result_bag / "metadata.yaml").is_file():
            raise RuntimeError(f"invalid result bag: {result_bag}")
        if not georeference_file.is_file():
            raise RuntimeError(f"georeference file not found: {georeference_file}")
        if not mounts_file.is_file():
            raise RuntimeError(f"sensor mounts file not found: {mounts_file}")
        mounts = yaml.safe_load(mounts_file.read_text(encoding="utf-8"))
        georeference = yaml.safe_load(
            georeference_file.read_text(encoding="utf-8")
        )
        self.lio_path = load_lio_base_path(result_bag, mounts, flatten)
        self.rtk_path = load_rtk_base_path(
            result_bag, mounts, georeference, flatten
        )
        self.fastlio_path = None
        self.fastlivo_path = None
        self.kf_gins_path = None
        self.robot_localization_path = None
        if comparison_bag_value:
            comparison_bag = Path(comparison_bag_value).expanduser()
            if not (comparison_bag / "metadata.yaml").is_file():
                raise RuntimeError(f"invalid comparison bag: {comparison_bag}")
            self.fastlio_path = load_fastlio_base_path(
                comparison_bag, self.lio_path, flatten
            )
            self.fastlivo_path = load_fastlivo_base_path(
                comparison_bag, self.lio_path, mounts, flatten
            )
            self.kf_gins_path = load_kf_gins_base_path(
                comparison_bag, georeference, flatten
            )
            self.robot_localization_path = load_enu_base_path(
                comparison_bag,
                "/comparison/robot_localization/odometry",
                georeference,
                flatten,
            )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lio_publisher = self.create_publisher(
            PathMessage, "/mapping_result/lio_sam_path", qos
        )
        self.rtk_publisher = self.create_publisher(
            PathMessage, "/mapping_result/rtk_path", qos
        )
        self.fastlio_publisher = None
        self.fastlivo_publisher = None
        self.kf_gins_publisher = None
        self.robot_localization_publisher = None
        if self.fastlio_path is not None:
            self.fastlio_publisher = self.create_publisher(
                PathMessage, "/mapping_result/fastlio_path", qos
            )
            self.fastlivo_publisher = self.create_publisher(
                PathMessage, "/mapping_result/fastlivo_path", qos
            )
            self.kf_gins_publisher = self.create_publisher(
                PathMessage, "/mapping_result/kf_gins_path", qos
            )
            self.robot_localization_publisher = self.create_publisher(
                PathMessage, "/mapping_result/robot_localization_path", qos
            )
        self.create_timer(1.0, self.publish)
        counts = (
            f"LIO-SAM={len(self.lio_path.poses)} poses, "
            f"RTK={len(self.rtk_path.poses)} poses"
        )
        if self.fastlio_path is not None:
            counts += (
                ", recomputed FAST-LIO2="
                f"{len(self.fastlio_path.poses)} poses, "
                "recomputed FAST-LIVO2="
                f"{len(self.fastlivo_path.poses)} poses, "
                f"recomputed KF-GINS={len(self.kf_gins_path.poses)} poses, "
                "robot_localization EKF="
                f"{len(self.robot_localization_path.poses)} poses"
            )
        self.get_logger().info("Loaded rear-axle paths: " + counts)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.lio_path.header.stamp = stamp
        self.rtk_path.header.stamp = stamp
        self.lio_publisher.publish(self.lio_path)
        self.rtk_publisher.publish(self.rtk_path)
        if self.fastlio_path is not None:
            self.fastlio_path.header.stamp = stamp
            self.fastlivo_path.header.stamp = stamp
            self.kf_gins_path.header.stamp = stamp
            self.robot_localization_path.header.stamp = stamp
            self.fastlio_publisher.publish(self.fastlio_path)
            self.fastlivo_publisher.publish(self.fastlivo_path)
            self.kf_gins_publisher.publish(self.kf_gins_path)
            self.robot_localization_publisher.publish(
                self.robot_localization_path
            )


def main():
    rclpy.init()
    node = MappingResultTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
