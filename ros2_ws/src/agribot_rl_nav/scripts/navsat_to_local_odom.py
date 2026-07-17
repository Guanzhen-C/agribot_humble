#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


EARTH_RADIUS_M = 6378137.0


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavSatToLocalOdom(Node):
    def __init__(self) -> None:
        super().__init__("navsat_to_local_odom")
        self.fix_topic = self.declare_parameter("fix_topic", "/navsat/fix").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.child_frame_id = self.declare_parameter("child_frame_id", "base_link").value
        self.yaw_source_topic = self.declare_parameter("yaw_source_topic", "/imu/data").value
        self.yaw_source_message_type = self.declare_parameter(
            "yaw_source_message_type", "imu"
        ).value
        self.zero_altitude = as_bool(self.declare_parameter("zero_altitude", True).value)
        self.pose_topic = self.declare_parameter("pose_topic", "/navsat_pose").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.origin_x = float(self.declare_parameter("origin_x", 0.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", 0.0).value)
        self.origin_z = float(self.declare_parameter("origin_z", 0.0).value)
        self.origin_yaw = float(self.declare_parameter("origin_yaw", 0.0).value)
        self.invert_gazebo_axes = as_bool(
            self.declare_parameter("invert_gazebo_axes", False).value
        )
        self.publish_pose_and_tf = as_bool(
            self.declare_parameter("publish_pose_and_tf", True).value
        )
        self.yaw_variance = float(
            self.declare_parameter("yaw_variance", 1.0e6).value
        )
        self.datum_lat = self.declare_parameter(
            "datum_lat", Parameter.Type.DOUBLE
        ).value
        self.datum_lon = self.declare_parameter(
            "datum_lon", Parameter.Type.DOUBLE
        ).value
        self.datum_alt = self.declare_parameter(
            "datum_alt", Parameter.Type.DOUBLE
        ).value

        self.datum = None
        if self.datum_lat is not None and self.datum_lon is not None:
            self.datum = (
                float(self.datum_lat),
                float(self.datum_lon),
                float(self.datum_alt) if self.datum_alt is not None else 0.0,
            )

        self.latest_orientation = None
        self.latest_odom_source = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, "odometry/gps", 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(NavSatFix, self.fix_topic, self.handle_fix, 10)
        if self.yaw_source_topic:
            if self.yaw_source_message_type == "imu":
                self.create_subscription(
                    Imu, self.yaw_source_topic, self.handle_yaw_source_imu, 10
                )
            else:
                self.create_subscription(
                    Odometry, self.yaw_source_topic, self.handle_yaw_source_odom, 10
                )

    def handle_yaw_source_odom(self, msg: Odometry) -> None:
        self.latest_odom_source = msg
        self.latest_orientation = msg.pose.pose.orientation

    def handle_yaw_source_imu(self, msg: Imu) -> None:
        self.latest_orientation = msg.orientation

    def lookup_odom_pose(self, stamp) -> Odometry:
        if self.latest_odom_source is not None:
            return self.latest_odom_source

        query_times = []
        try:
            query_times.append(Time.from_msg(stamp))
        except Exception:
            pass
        query_times.append(Time())

        for query_time in query_times:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.child_frame_id,
                    query_time,
                )
                odom = Odometry()
                odom.header.stamp = transform.header.stamp
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.child_frame_id
                odom.pose.pose.position.x = transform.transform.translation.x
                odom.pose.pose.position.y = transform.transform.translation.y
                odom.pose.pose.position.z = transform.transform.translation.z
                odom.pose.pose.orientation = transform.transform.rotation
                return odom
            except TransformException:
                continue
        return None

    def handle_fix(self, msg: NavSatFix) -> None:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return

        if self.datum is None:
            self.datum = (
                float(msg.latitude),
                float(msg.longitude),
                float(msg.altitude) if math.isfinite(msg.altitude) else 0.0,
            )
            self.get_logger().info(
                "navsat_to_local_odom datum set to lat=%.8f lon=%.8f alt=%.3f"
                % self.datum
            )

        lat0, lon0, alt0 = self.datum
        lon = math.radians(msg.longitude)
        lon0_rad = math.radians(lon0)
        lat0_rad = math.radians(lat0)

        east = EARTH_RADIUS_M * (lon - lon0_rad) * math.cos(lat0_rad)
        north = EARTH_RADIUS_M * (math.radians(msg.latitude) - math.radians(lat0))

        if self.invert_gazebo_axes:
            east = -east
            north = -north

        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        x = self.origin_x + cos_yaw * east - sin_yaw * north
        y = self.origin_y + sin_yaw * east + cos_yaw * north
        z = 0.0 if self.zero_altitude else (
            self.origin_z + ((msg.altitude - alt0) if math.isfinite(msg.altitude) else 0.0)
        )

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        if self.latest_orientation is not None:
            odom.pose.pose.orientation = self.latest_orientation
        else:
            quat = yaw_to_quaternion(self.origin_yaw)
            odom.pose.pose.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            )

        cov = list(msg.position_covariance)
        if len(cov) >= 9:
            odom.pose.covariance[0] = cov[0]
            odom.pose.covariance[7] = cov[4]
            odom.pose.covariance[14] = cov[8] if not self.zero_altitude else 1e3
        else:
            odom.pose.covariance[0] = 1.0
            odom.pose.covariance[7] = 1.0
            odom.pose.covariance[14] = 1e3
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = self.yaw_variance
        self.odom_pub.publish(odom)
        if self.publish_pose_and_tf:
            self.publish_pose_and_tf_message(
                msg.header.stamp, x, y, z, odom.pose.pose.orientation
            )

    def publish_pose_and_tf_message(
        self,
        stamp,
        map_x: float,
        map_y: float,
        map_z: float,
        map_orientation: Quaternion,
    ) -> None:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.pose.position.x = map_x
        pose_msg.pose.pose.position.y = map_y
        pose_msg.pose.pose.position.z = map_z
        pose_msg.pose.pose.orientation = map_orientation
        pose_msg.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02,
        ]
        self.pose_pub.publish(pose_msg)

        odom_source = self.lookup_odom_pose(stamp)
        if odom_source is None:
            return

        odom_pose = odom_source.pose.pose
        map_yaw = quaternion_to_yaw(map_orientation)
        odom_yaw = quaternion_to_yaw(odom_pose.orientation)
        map_to_odom_yaw = map_yaw - odom_yaw
        cos_yaw = math.cos(map_to_odom_yaw)
        sin_yaw = math.sin(map_to_odom_yaw)
        map_to_odom_x = map_x - (
            cos_yaw * odom_pose.position.x - sin_yaw * odom_pose.position.y
        )
        map_to_odom_y = map_y - (
            sin_yaw * odom_pose.position.x + cos_yaw * odom_pose.position.y
        )
        quat = yaw_to_quaternion(map_to_odom_yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = map_to_odom_x
        tf_msg.transform.translation.y = map_to_odom_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = NavSatToLocalOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
