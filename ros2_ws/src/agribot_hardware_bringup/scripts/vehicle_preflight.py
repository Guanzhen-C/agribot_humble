#!/usr/bin/env python3

import fcntl
import math
import socket
import struct
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scout_msgs.msg import ScoutStatus
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import Bool, String, UInt8


IFF_UP = 0x1
SIOCGIFFLAGS = 0x8913


def interface_is_up(interface: str) -> bool:
    if not interface or len(interface.encode("ascii", errors="ignore")) >= 16:
        return False
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as descriptor:
            request = struct.pack("16sH", interface.encode("ascii"), 0)
            response = fcntl.ioctl(descriptor.fileno(), SIOCGIFFLAGS, request)
        flags = struct.unpack("16sH", response[:18])[1]
        return bool(flags & IFF_UP)
    except OSError:
        return False


def quaternion_is_valid(x: float, y: float, z: float, w: float) -> bool:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return False
    norm = math.sqrt(sum(value * value for value in values))
    return 0.5 <= norm <= 1.5


class VehiclePreflight(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_preflight")
        self.localization_mode = str(
            self.declare_parameter("localization_mode", "navsat").value
        ).lower()
        if self.localization_mode not in ("navsat", "fastlio"):
            raise ValueError("localization_mode must be 'navsat' or 'fastlio'")

        self.require_can = bool(self.declare_parameter("require_can", False).value)
        self.require_can_interface = bool(
            self.declare_parameter("require_can_interface", self.require_can).value
        )
        self.require_chassis_feedback = bool(
            self.declare_parameter("require_chassis_feedback", self.require_can).value
        )
        self.can_interface = self.declare_parameter("can_interface", "can0").value
        self.sensor_timeout = float(
            self.declare_parameter("sensor_timeout_sec", 1.0).value
        )
        self.localization_timeout = float(
            self.declare_parameter("localization_timeout_sec", 2.0).value
        )
        self.chassis_timeout = float(
            self.declare_parameter("chassis_timeout_sec", 1.0).value
        )
        self.accepted_rtk_qualities = {
            int(value)
            for value in self.declare_parameter(
                "accepted_rtk_qualities", [4, 5]
            ).value
        }

        if min(self.sensor_timeout, self.localization_timeout, self.chassis_timeout) <= 0.0:
            raise ValueError("preflight timeouts must be positive")

        self.received_at: Dict[str, int] = {}
        self.rtk_quality: Optional[int] = None
        self.heading_valid = False
        self.last_ready = None
        self.last_status = ""

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_publisher = self.create_publisher(
            Bool, "/hardware/preflight_ready", state_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/hardware/preflight_status", state_qos
        )

        self.create_subscription(
            PointCloud2, "/lidar/points", self.handle_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.handle_scan, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/data", self.handle_imu, qos_profile_sensor_data)

        if self.localization_mode == "navsat":
            self.create_subscription(
                NavSatFix, "/rtk/fix", self.handle_fix, qos_profile_sensor_data
            )
            self.create_subscription(UInt8, "/rtk/fix_quality", self.handle_quality, 10)
            self.create_subscription(
                QuaternionStamped, "/rtk/heading", self.handle_heading, 10
            )
            self.create_subscription(
                Bool, "/rtk/heading_valid", self.handle_heading_valid, 10
            )
            odom_topic = "/odometry/filtered_navsat"
        else:
            odom_topic = "/fastlio/odometry"

        self.create_subscription(
            Odometry, odom_topic, self.handle_localization, qos_profile_sensor_data
        )
        if self.require_chassis_feedback:
            self.create_subscription(
                ScoutStatus, "/scout_status", self.handle_chassis, 10
            )

        self.create_timer(0.2, self.evaluate)
        self.get_logger().info(
            "Vehicle preflight ready: localization=%s can_interface=%s "
            "chassis_feedback=%s interface=%s"
            % (
                self.localization_mode,
                self.require_can_interface,
                self.require_chassis_feedback,
                self.can_interface,
            )
        )

    def mark(self, name: str) -> None:
        self.received_at[name] = self.get_clock().now().nanoseconds

    def fresh(self, name: str, timeout: float) -> bool:
        stamp = self.received_at.get(name)
        if stamp is None:
            return False
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return 0.0 <= age <= timeout

    def handle_cloud(self, msg: PointCloud2) -> None:
        if msg.width * msg.height > 0 and msg.point_step > 0 and len(msg.data) > 0:
            self.mark("lidar_points")

    def handle_scan(self, msg: LaserScan) -> None:
        if len(msg.ranges) > 0 and math.isfinite(msg.angle_increment):
            self.mark("scan")

    def handle_imu(self, msg: Imu) -> None:
        inertial_values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        orientation = msg.orientation
        if all(math.isfinite(value) for value in inertial_values) and quaternion_is_valid(
            orientation.x, orientation.y, orientation.z, orientation.w
        ):
            self.mark("imu")

    def handle_fix(self, msg: NavSatFix) -> None:
        coordinates = (msg.latitude, msg.longitude, msg.altitude)
        if msg.status.status != NavSatStatus.STATUS_NO_FIX and all(
            math.isfinite(value) for value in coordinates
        ):
            self.mark("rtk_fix")

    def handle_quality(self, msg: UInt8) -> None:
        self.rtk_quality = int(msg.data)
        if self.rtk_quality in self.accepted_rtk_qualities:
            self.mark("rtk_quality")

    def handle_heading(self, msg: QuaternionStamped) -> None:
        orientation = msg.quaternion
        if quaternion_is_valid(
            orientation.x, orientation.y, orientation.z, orientation.w
        ):
            self.mark("rtk_heading")

    def handle_heading_valid(self, msg: Bool) -> None:
        self.heading_valid = bool(msg.data)
        if self.heading_valid:
            self.mark("rtk_heading_valid")

    def handle_localization(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        if all(
            math.isfinite(value) for value in (position.x, position.y, position.z)
        ) and quaternion_is_valid(
            orientation.x, orientation.y, orientation.z, orientation.w
        ):
            self.mark("localization")

    def handle_chassis(self, _msg: ScoutStatus) -> None:
        self.mark("chassis")

    def missing_requirements(self):
        missing = []
        for name in ("lidar_points", "scan", "imu"):
            if not self.fresh(name, self.sensor_timeout):
                missing.append(name)

        if self.localization_mode == "navsat":
            for name in ("rtk_fix", "rtk_quality", "rtk_heading", "rtk_heading_valid"):
                if not self.fresh(name, self.localization_timeout):
                    missing.append(name)
            if self.rtk_quality not in self.accepted_rtk_qualities:
                missing.append(f"rtk_quality={self.rtk_quality}")
            if not self.heading_valid:
                missing.append("rtk_heading_invalid")

        if not self.fresh("localization", self.localization_timeout):
            missing.append("localization")

        if self.require_can_interface:
            if not interface_is_up(self.can_interface):
                missing.append(f"{self.can_interface}_down")
        if self.require_chassis_feedback:
            if not self.fresh("chassis", self.chassis_timeout):
                missing.append("chassis_feedback")
        return missing

    def evaluate(self) -> None:
        missing = self.missing_requirements()
        ready = not missing
        status = "READY" if ready else "WAITING: " + ", ".join(missing)

        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_publisher.publish(ready_msg)
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

        if ready != self.last_ready or status != self.last_status:
            if ready:
                self.get_logger().info("Vehicle preflight passed")
            elif ready != self.last_ready:
                self.get_logger().warn(status)
            self.last_ready = ready
            self.last_status = status


def main() -> None:
    rclpy.init()
    node = VehiclePreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
