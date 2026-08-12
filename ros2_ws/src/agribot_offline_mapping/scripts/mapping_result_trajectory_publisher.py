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


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    semi_major_axis = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    prime_vertical_radius = semi_major_axis / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude
    )
    return (
        (prime_vertical_radius + altitude_m)
        * math.cos(latitude) * math.cos(longitude),
        (prime_vertical_radius + altitude_m)
        * math.cos(latitude) * math.sin(longitude),
        (
            prime_vertical_radius * (1.0 - eccentricity_squared)
            + altitude_m
        ) * sin_latitude,
    )


def geodetic_to_enu(
    latitude_deg, longitude_deg, altitude_m,
    reference_latitude_deg, reference_longitude_deg, reference_altitude_m,
):
    position = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
    reference = geodetic_to_ecef(
        reference_latitude_deg, reference_longitude_deg, reference_altitude_m
    )
    delta_x, delta_y, delta_z = (
        position[index] - reference[index] for index in range(3)
    )
    latitude = math.radians(reference_latitude_deg)
    longitude = math.radians(reference_longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (
        -sin_longitude * delta_x + cos_longitude * delta_y,
        -sin_latitude * cos_longitude * delta_x
        - sin_latitude * sin_longitude * delta_y
        + cos_latitude * delta_z,
        cos_latitude * cos_longitude * delta_x
        + cos_latitude * sin_longitude * delta_y
        + sin_latitude * delta_z,
    )


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


def last_topic_message(result_bag, topic_name):
    latest = None
    for database in result_databases(result_bag):
        connection = sqlite3.connect(database)
        topic = connection.execute(
            "SELECT id, type FROM topics WHERE name = ?", (topic_name,)
        ).fetchone()
        if topic is not None:
            topic_id, topic_type = topic
            row = connection.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp DESC LIMIT 1",
                (topic_id,),
            ).fetchone()
            if row is not None and (latest is None or row[0] > latest[0]):
                latest = (row[0], row[1], topic_type)
        connection.close()
    if latest is None:
        raise RuntimeError(f"result bag topic is empty: {topic_name}")
    timestamp, serialized, topic_type = latest
    return timestamp, deserialize_message(serialized, get_message(topic_type))


def nearest_timed_message(messages, timestamps, timestamp, maximum_delta_sec):
    insertion = bisect.bisect_left(timestamps, timestamp)
    candidates = [
        index for index in (insertion - 1, insertion)
        if 0 <= index < len(messages)
    ]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda index: abs(timestamps[index] - timestamp))
    if abs(timestamps[nearest] - timestamp) > maximum_delta_sec * 1.0e9:
        return None
    return messages[nearest][1]


def final_contiguous_quality_interval(
    quality_messages, required_quality=5, maximum_gap_sec=0.5
):
    matching = [
        index for index, (_, message) in enumerate(quality_messages)
        if int(message.data) == required_quality
    ]
    if not matching:
        raise RuntimeError(f"RTK quality {required_quality} is absent")
    end = matching[-1]
    start = end
    maximum_gap_ns = maximum_gap_sec * 1.0e9
    while start > 0:
        previous_stamp, previous_message = quality_messages[start - 1]
        current_stamp = quality_messages[start][0]
        if (
            int(previous_message.data) != required_quality
            or current_stamp - previous_stamp > maximum_gap_ns
        ):
            break
        start -= 1
    return quality_messages[start][0], quality_messages[end][0]


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
    source_path = last_topic_message(result_bag, "/lio_sam/mapping/path")[1]
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


def load_final_float_rtk_base_path(source_bag, mounts, georeference, flatten):
    quality_messages = topic_messages(source_bag, "/rtk/fix_quality")
    interval_start, interval_end = final_contiguous_quality_interval(
        quality_messages
    )
    quality_stamps = [timestamp for timestamp, _ in quality_messages]
    headings = topic_messages(source_bag, "/rtk/heading_with_covariance")
    heading_stamps = [timestamp for timestamp, _ in headings]
    fixes = topic_messages(source_bag, "/rtk/fix")

    reference = georeference["reference"]
    translation = tuple(
        float(value) for value in georeference["map_from_enu"]["xyz"]
    )
    map_from_enu = quaternion_from_rpy(
        *(float(value) for value in georeference["map_from_enu"]["rpy"])
    )
    base_to_antenna = tuple(float(value) for value in mounts["rtk"]["xyz"])

    path = PathMessage()
    path.header.frame_id = "map"
    for timestamp, fix in fixes:
        if timestamp < interval_start or timestamp > interval_end:
            continue
        quality = nearest_timed_message(
            quality_messages, quality_stamps, timestamp, 0.20
        )
        if quality is None or int(quality.data) != 5:
            continue
        heading = nearest_timed_message(headings, heading_stamps, timestamp, 0.70)
        if heading is None:
            continue
        enu_from_base = message_quaternion(heading.pose.pose.orientation)
        enu_antenna = geodetic_to_enu(
            fix.latitude,
            fix.longitude,
            fix.altitude,
            float(reference["latitude_deg"]),
            float(reference["longitude_deg"]),
            float(reference["altitude_m"]),
        )
        rotated_lever = rotate(enu_from_base, base_to_antenna)
        enu_base = tuple(
            enu_antenna[index] - rotated_lever[index] for index in range(3)
        )
        rotated_position = rotate(map_from_enu, enu_base)
        map_position = tuple(
            translation[index] + rotated_position[index] for index in range(3)
        )
        map_from_base = multiply(map_from_enu, enu_from_base)
        source = PoseStamped()
        source.header = fix.header
        path.poses.append(
            pose_stamped(source, map_position, map_from_base, flatten)
        )
    if not path.poses:
        raise RuntimeError("final contiguous RTK float interval has no usable poses")
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


def load_fastlivo_base_path(
    comparison_bag, lio_path, mounts, flatten,
    topic_name="/comparison/fastlivo/odometry",
):
    odometry = [
        message for _, message in topic_messages(
            comparison_bag, topic_name
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
        source_bag_value = str(
            self.declare_parameter("source_bag", "").value
        ).strip()
        fastlivo_bag_value = str(
            self.declare_parameter("fastlivo_bag", "").value
        ).strip()
        fastlivo_topic = str(
            self.declare_parameter(
                "fastlivo_topic", "/comparison/fastlivo/odometry"
            ).value
        ).strip()
        if not fastlivo_topic.startswith("/"):
            raise RuntimeError("fastlivo_topic must be an absolute topic name")
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
        self.rtk_float_path = None
        if comparison_bag_value:
            comparison_bag = Path(comparison_bag_value).expanduser()
            if not (comparison_bag / "metadata.yaml").is_file():
                raise RuntimeError(f"invalid comparison bag: {comparison_bag}")
            fastlivo_bag = (
                Path(fastlivo_bag_value).expanduser()
                if fastlivo_bag_value
                else comparison_bag
            )
            if not (fastlivo_bag / "metadata.yaml").is_file():
                raise RuntimeError(f"invalid FAST-LIVO2 bag: {fastlivo_bag}")
            self.fastlio_path = load_fastlio_base_path(
                comparison_bag, self.lio_path, flatten
            )
            self.fastlivo_path = load_fastlivo_base_path(
                fastlivo_bag, self.lio_path, mounts, flatten, fastlivo_topic
            )
            self.kf_gins_path = load_kf_gins_base_path(
                comparison_bag, georeference, flatten
            )
            source_bag = Path(source_bag_value).expanduser()
            if not (source_bag / "metadata.yaml").is_file():
                raise RuntimeError(f"invalid source bag: {source_bag}")
            self.rtk_float_path = load_final_float_rtk_base_path(
                source_bag, mounts, georeference, flatten
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
        self.rtk_float_publisher = None
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
            self.rtk_float_publisher = self.create_publisher(
                PathMessage, "/mapping_result/rtk_float_path", qos
            )
        self.publish_timer = self.create_timer(0.5, self.publish_once)
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
                "final contiguous RTK float="
                f"{len(self.rtk_float_path.poses)} poses"
            )
        self.get_logger().info("Loaded rear-axle paths: " + counts)

    def publish_once(self):
        stamp = self.get_clock().now().to_msg()
        self.lio_path.header.stamp = stamp
        self.rtk_path.header.stamp = stamp
        self.lio_publisher.publish(self.lio_path)
        self.rtk_publisher.publish(self.rtk_path)
        if self.fastlio_path is not None:
            self.fastlio_path.header.stamp = stamp
            self.fastlivo_path.header.stamp = stamp
            self.kf_gins_path.header.stamp = stamp
            self.rtk_float_path.header.stamp = stamp
            self.fastlio_publisher.publish(self.fastlio_path)
            self.fastlivo_publisher.publish(self.fastlivo_path)
            self.kf_gins_publisher.publish(self.kf_gins_path)
            self.rtk_float_publisher.publish(self.rtk_float_path)
        self.publish_timer.cancel()
        self.get_logger().info("Published transient-local mapping result paths")


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
