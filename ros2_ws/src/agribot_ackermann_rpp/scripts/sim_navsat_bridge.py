#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6378137.0


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SimNavSatBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_navsat_bridge")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.fix_topic = self.declare_parameter("fix_topic", "/navsat/fix").value
        self.imu_topic = self.declare_parameter("imu_topic", "/imu/data").value
        self.imu_corrected_topic = self.declare_parameter(
            "imu_corrected_topic", "/imu/data_corrected"
        ).value
        self.publish_imu_enabled = bool(self.declare_parameter("publish_imu", True).value)
        self.reference_lat = float(self.declare_parameter("reference_lat", 30.5).value)
        self.reference_lon = float(self.declare_parameter("reference_lon", 114.0).value)
        self.reference_alt = float(self.declare_parameter("reference_alt", 20.0).value)
        self.origin_x = float(self.declare_parameter("origin_x", 2.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", 36.0).value)
        self.origin_z = float(self.declare_parameter("origin_z", 0.24).value)
        self.origin_yaw = float(self.declare_parameter("origin_yaw", 0.0).value)
        self.fix_covariance_xy = float(self.declare_parameter("fix_covariance_xy", 0.03).value)
        self.fix_covariance_z = float(self.declare_parameter("fix_covariance_z", 0.05).value)

        self.ground_truth_pub = self.create_publisher(Odometry, self.ground_truth_topic, 10)
        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.imu_pub = None
        self.imu_corrected_pub = None
        if self.publish_imu_enabled:
            self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
            self.imu_corrected_pub = self.create_publisher(Imu, self.imu_corrected_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, 20)

        self.cos_origin_yaw = math.cos(self.origin_yaw)
        self.sin_origin_yaw = math.sin(self.origin_yaw)
        self.cos_reference_lat = math.cos(math.radians(self.reference_lat))

    def handle_odom(self, msg: Odometry) -> None:
        self.publish_ground_truth(msg)
        self.publish_navsat_fix(msg)
        if self.publish_imu_enabled:
            self.publish_imu(msg)

    def publish_ground_truth(self, msg: Odometry) -> None:
        ground_truth = Odometry()
        ground_truth.header = msg.header
        ground_truth.child_frame_id = msg.child_frame_id
        ground_truth.pose = msg.pose
        ground_truth.twist = msg.twist
        self.ground_truth_pub.publish(ground_truth)

    def publish_navsat_fix(self, msg: Odometry) -> None:
        dx = msg.pose.pose.position.x - self.origin_x
        dy = msg.pose.pose.position.y - self.origin_y

        east_inverted = self.cos_origin_yaw * dx + self.sin_origin_yaw * dy
        north_inverted = -self.sin_origin_yaw * dx + self.cos_origin_yaw * dy
        east = -east_inverted
        north = -north_inverted

        latitude = self.reference_lat + math.degrees(north / EARTH_RADIUS_M)
        longitude = self.reference_lon + math.degrees(
            east / (EARTH_RADIUS_M * max(self.cos_reference_lat, 1e-6))
        )
        altitude = self.reference_alt + (msg.pose.pose.position.z - self.origin_z)

        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = "gps_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude
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
        self.fix_pub.publish(fix)

    def publish_imu(self, msg: Odometry) -> None:
        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = "base_link"
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.orientation_covariance = [
            0.02,
            0.0,
            0.0,
            0.0,
            0.02,
            0.0,
            0.0,
            0.0,
            0.02,
        ]
        imu.angular_velocity_covariance = [
            0.02,
            0.0,
            0.0,
            0.0,
            0.02,
            0.0,
            0.0,
            0.0,
            0.02,
        ]
        imu.linear_acceleration_covariance = [
            0.5,
            0.0,
            0.0,
            0.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.5,
        ]
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
