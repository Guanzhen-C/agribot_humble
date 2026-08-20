#!/usr/bin/env python3

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8


WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


def geodetic_to_ecef(latitude_rad, longitude_rad, altitude_m):
    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + altitude_m) * cos_lat * math.cos(longitude_rad),
        (radius + altitude_m) * cos_lat * math.sin(longitude_rad),
        (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat,
    )


def ecef_to_geodetic(x, y, z):
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1.0 - WGS84_E2))
    altitude = 0.0
    for _ in range(10):
        sin_lat = math.sin(latitude)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        altitude = horizontal / max(math.cos(latitude), 1.0e-12) - radius
        next_latitude = math.atan2(
            z,
            horizontal * (1.0 - WGS84_E2 * radius / (radius + altitude)),
        )
        if abs(next_latitude - latitude) < 1.0e-13:
            latitude = next_latitude
            break
        latitude = next_latitude
    return latitude, longitude, altitude


class DifferentialSimSensorBridge(Node):
    def __init__(self):
        super().__init__("differential_sim_sensor_bridge")
        self.reference_latitude_deg = float(
            self.declare_parameter("reference_latitude_deg", 30.5).value
        )
        self.reference_longitude_deg = float(
            self.declare_parameter("reference_longitude_deg", 114.0).value
        )
        self.reference_altitude_m = float(
            self.declare_parameter("reference_altitude_m", 20.0).value
        )
        self.fix_rate_hz = float(self.declare_parameter("fix_rate_hz", 10.0).value)
        self.horizontal_sigma_m = float(
            self.declare_parameter("horizontal_sigma_m", 0.03).value
        )
        self.quality = int(self.declare_parameter("fix_quality", 4).value)
        self.last_fix_time_ns = None

        self.reference_latitude_rad = math.radians(self.reference_latitude_deg)
        self.reference_longitude_rad = math.radians(self.reference_longitude_deg)
        self.reference_ecef = geodetic_to_ecef(
            self.reference_latitude_rad,
            self.reference_longitude_rad,
            self.reference_altitude_m,
        )
        self.fix_publisher = self.create_publisher(
            NavSatFix, "/rtk/fix", qos_profile_sensor_data
        )
        self.quality_publisher = self.create_publisher(UInt8, "/rtk/fix_quality", 20)
        self.ground_truth_publisher = self.create_publisher(
            Odometry, "/simulation/ground_truth", qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.handle_odom, qos_profile_sensor_data
        )

    def enu_to_geodetic(self, east, north, up):
        sin_lat = math.sin(self.reference_latitude_rad)
        cos_lat = math.cos(self.reference_latitude_rad)
        sin_lon = math.sin(self.reference_longitude_rad)
        cos_lon = math.cos(self.reference_longitude_rad)
        dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
        dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
        dz = cos_lat * north + sin_lat * up
        latitude, longitude, altitude = ecef_to_geodetic(
            self.reference_ecef[0] + dx,
            self.reference_ecef[1] + dy,
            self.reference_ecef[2] + dz,
        )
        return math.degrees(latitude), math.degrees(longitude), altitude

    def handle_odom(self, message):
        ground_truth = Odometry()
        ground_truth.header = copy.deepcopy(message.header)
        ground_truth.header.frame_id = "map"
        ground_truth.child_frame_id = "base_link_ground_truth"
        ground_truth.pose = copy.deepcopy(message.pose)
        ground_truth.pose.pose.position.z = 0.0
        ground_truth.twist = copy.deepcopy(message.twist)
        self.ground_truth_publisher.publish(ground_truth)

        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        period_ns = int(1_000_000_000 / max(self.fix_rate_hz, 0.1))
        if self.last_fix_time_ns is not None:
            if stamp_ns >= self.last_fix_time_ns and stamp_ns - self.last_fix_time_ns < period_ns:
                return
        self.last_fix_time_ns = stamp_ns

        latitude, longitude, altitude = self.enu_to_geodetic(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            0.0,
        )
        quality = UInt8()
        quality.data = self.quality
        self.quality_publisher.publish(quality)

        fix = NavSatFix()
        fix.header = copy.deepcopy(message.header)
        fix.header.frame_id = "rtk_antenna"
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude
        variance = self.horizontal_sigma_m * self.horizontal_sigma_m
        fix.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, 0.04,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_publisher.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialSimSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
