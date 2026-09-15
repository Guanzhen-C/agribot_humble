#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import UInt8


EARTH_RADIUS_M = 6378137.0


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_to_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def rotate_vector(quaternion, vector):
    x, y, z = vector
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-12:
        return vector
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


class SimNavSatBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_navsat_bridge")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.fix_topic = self.declare_parameter("fix_topic", "/navsat/fix").value
        self.quality_topic = self.declare_parameter(
            "quality_topic", "/rtk/fix_quality"
        ).value
        self.fix_quality = int(self.declare_parameter("fix_quality", 4).value)
        if not 0 <= self.fix_quality <= 255:
            raise ValueError("fix_quality must be in the UInt8 range")
        self.heading_topic = self.declare_parameter(
            "heading_topic", "/rtk/heading_with_covariance"
        ).value
        self.imu_topic = self.declare_parameter("imu_topic", "/imu/data").value
        self.imu_corrected_topic = self.declare_parameter(
            "imu_corrected_topic", "/imu/data_corrected"
        ).value
        self.publish_imu_enabled = bool(self.declare_parameter("publish_imu", False).value)
        self.reference_lat = float(self.declare_parameter("reference_lat", 30.5).value)
        self.reference_lon = float(self.declare_parameter("reference_lon", 114.0).value)
        self.reference_alt = float(self.declare_parameter("reference_alt", 20.0).value)
        self.origin_x = float(self.declare_parameter("origin_x", 2.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", 36.0).value)
        self.origin_z = float(self.declare_parameter("origin_z", 0.1275).value)
        self.origin_yaw = float(self.declare_parameter("origin_yaw", 0.0).value)
        self.base_to_antenna = tuple(
            float(value)
            for value in self.declare_parameter(
                "base_to_antenna_m", [0.1425, 0.2952585, 0.78476]
            ).value
        )
        if len(self.base_to_antenna) != 3:
            raise ValueError("base_to_antenna_m must contain exactly three values")
        self.fix_rate_hz = float(self.declare_parameter("fix_rate_hz", 10.0).value)
        self.heading_rate_hz = float(self.declare_parameter("heading_rate_hz", 1.0).value)
        # NavSatFix stores variance, not standard deviation.
        self.fix_covariance_xy = float(
            self.declare_parameter("fix_covariance_xy", 0.0009).value
        )
        self.fix_covariance_z = float(
            self.declare_parameter("fix_covariance_z", 0.0036).value
        )
        self.heading_std_rad = math.radians(
            float(self.declare_parameter("heading_std_deg", 1.0).value)
        )

        self.ground_truth_pub = self.create_publisher(Odometry, self.ground_truth_topic, 10)
        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.quality_pub = self.create_publisher(UInt8, self.quality_topic, 10)
        self.heading_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.heading_topic, 10
        )
        self.imu_pub = None
        self.imu_corrected_pub = None
        if self.publish_imu_enabled:
            self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
            self.imu_corrected_pub = self.create_publisher(
                Imu, self.imu_corrected_topic, 10
            )
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, 20)

        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        origin_offset = (
            cos_yaw * self.base_to_antenna[0]
            - sin_yaw * self.base_to_antenna[1],
            sin_yaw * self.base_to_antenna[0]
            + cos_yaw * self.base_to_antenna[1],
            self.base_to_antenna[2],
        )
        self.origin_antenna = (
            self.origin_x + origin_offset[0],
            self.origin_y + origin_offset[1],
            self.origin_z + origin_offset[2],
        )
        self.cos_reference_lat = math.cos(math.radians(self.reference_lat))
        self.last_fix_stamp = None
        self.last_heading_stamp = None

        self.get_logger().info(
            "Simulated dual-antenna RTK uses master lever arm "
            f"{self.base_to_antenna}, fix={self.fix_rate_hz:.1f} Hz, "
            f"heading={self.heading_rate_hz:.1f} Hz"
        )

    @staticmethod
    def stamp_seconds(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9

    @staticmethod
    def due(now: float, previous, rate_hz: float) -> bool:
        if rate_hz <= 0.0:
            return False
        return previous is None or now < previous or now - previous >= 1.0 / rate_hz - 1.0e-6

    def handle_odom(self, msg: Odometry) -> None:
        stamp = self.stamp_seconds(msg)
        self.publish_ground_truth(msg)
        if self.due(stamp, self.last_heading_stamp, self.heading_rate_hz):
            self.publish_heading(msg)
            self.last_heading_stamp = stamp
        if self.due(stamp, self.last_fix_stamp, self.fix_rate_hz):
            self.publish_navsat_fix(msg)
            self.last_fix_stamp = stamp
        if self.publish_imu_enabled:
            self.publish_imu(msg)

    def publish_ground_truth(self, msg: Odometry) -> None:
        ground_truth = Odometry()
        ground_truth.header = msg.header
        ground_truth.child_frame_id = msg.child_frame_id
        ground_truth.pose = msg.pose
        ground_truth.twist = msg.twist
        self.ground_truth_pub.publish(ground_truth)

    def antenna_position(self, msg: Odometry):
        offset = rotate_vector(msg.pose.pose.orientation, self.base_to_antenna)
        return (
            msg.pose.pose.position.x + offset[0],
            msg.pose.pose.position.y + offset[1],
            msg.pose.pose.position.z + offset[2],
        )

    def publish_navsat_fix(self, msg: Odometry) -> None:
        antenna = self.antenna_position(msg)
        east = antenna[0] - self.origin_antenna[0]
        north = antenna[1] - self.origin_antenna[1]
        up = antenna[2] - self.origin_antenna[2]

        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = "rtk_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self.reference_lat + math.degrees(north / EARTH_RADIUS_M)
        fix.longitude = self.reference_lon + math.degrees(
            east / (EARTH_RADIUS_M * max(self.cos_reference_lat, 1.0e-6))
        )
        fix.altitude = self.reference_alt + up
        fix.position_covariance = [
            self.fix_covariance_xy,
            0.0,
            0.0,
            0.0,
            self.fix_covariance_xy,
            0.0,
            0.0,
            0.0,
            self.fix_covariance_z,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        quality = UInt8()
        quality.data = self.fix_quality
        self.quality_pub.publish(quality)
        self.fix_pub.publish(fix)

    def publish_heading(self, msg: Odometry) -> None:
        heading = PoseWithCovarianceStamped()
        heading.header = msg.header
        heading.header.frame_id = "rtk_link"
        heading.pose.pose.orientation = msg.pose.pose.orientation
        heading.pose.covariance[35] = self.heading_std_rad * self.heading_std_rad
        self.heading_pub.publish(heading)

    def publish_imu(self, msg: Odometry) -> None:
        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = "base_link"
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular
        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.02
        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02
        imu.linear_acceleration_covariance[0] = 0.5
        imu.linear_acceleration_covariance[4] = 0.5
        imu.linear_acceleration_covariance[8] = 0.5
        self.imu_pub.publish(imu)
        self.imu_corrected_pub.publish(imu)


def main() -> None:
    rclpy.init()
    node = SimNavSatBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
