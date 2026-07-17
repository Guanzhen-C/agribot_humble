#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class NavSatTruthErrorMonitor(Node):
    def __init__(self) -> None:
        super().__init__("navsat_truth_error_monitor")
        self.fused_odom_topic = self.declare_parameter(
            "fused_odom_topic", "/odometry/filtered_navsat"
        ).value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.report_hz = float(self.declare_parameter("report_hz", 1.0).value)

        self.fused_pose: Optional[Tuple[float, float, float]] = None
        self.truth_pose: Optional[Tuple[float, float, float]] = None
        self.fused_origin: Optional[Tuple[float, float, float]] = None
        self.truth_origin: Optional[Tuple[float, float, float]] = None

        self.error_pub = self.create_publisher(Vector3Stamped, "navsat_fusion_error", 10)
        self.create_subscription(Odometry, self.fused_odom_topic, self.handle_fused, 10)
        self.create_subscription(Odometry, self.ground_truth_topic, self.handle_truth, 10)
        self.create_timer(1.0 / max(self.report_hz, 1e-3), self.report)

    def handle_fused(self, msg: Odometry) -> None:
        pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quat(msg.pose.pose.orientation),
        )
        self.fused_pose = pose
        if self.fused_origin is None:
            self.fused_origin = pose

    def handle_truth(self, msg: Odometry) -> None:
        pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quat(msg.pose.pose.orientation),
        )
        self.truth_pose = pose
        if self.truth_origin is None:
            self.truth_origin = pose

    def report(self) -> None:
        if None in (self.fused_pose, self.truth_pose, self.fused_origin, self.truth_origin):
            return

        fx, fy, fyaw = self.fused_pose
        tx, ty, tyaw = self.truth_pose
        fox, foy, foyaw = self.fused_origin
        tox, toy, toyaw = self.truth_origin

        raw_dx = fx - tx
        raw_dy = fy - ty
        raw_pos_err = math.hypot(raw_dx, raw_dy)
        raw_dyaw = norm_angle(fyaw - tyaw)

        aligned_fx = fx - fox
        aligned_fy = fy - foy
        aligned_fyaw = norm_angle(fyaw - foyaw)
        # Ground truth is reported in Gazebo's world frame, while fused navsat
        # odometry is expressed in the navigation map frame. Align truth into the
        # fused frame using the initial pose offset and initial heading delta.
        truth_dx_world = tx - tox
        truth_dy_world = ty - toy
        truth_to_fused_yaw = norm_angle(foyaw - toyaw)
        cos_yaw = math.cos(truth_to_fused_yaw)
        sin_yaw = math.sin(truth_to_fused_yaw)
        aligned_tx = cos_yaw * truth_dx_world - sin_yaw * truth_dy_world
        aligned_ty = sin_yaw * truth_dx_world + cos_yaw * truth_dy_world
        aligned_tyaw = norm_angle(tyaw - toyaw)

        dx = aligned_fx - aligned_tx
        dy = aligned_fy - aligned_ty
        pos_err = math.hypot(dx, dy)
        dyaw = norm_angle(aligned_fyaw - aligned_tyaw)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = dx
        msg.vector.y = dy
        msg.vector.z = dyaw
        self.error_pub.publish(msg)

        self.get_logger().info(
            "navsat fusion error raw: dx=%.3f dy=%.3f pos=%.3f dyaw=%.3f rad | "
            "aligned: dx=%.3f dy=%.3f pos=%.3f dyaw=%.3f rad"
            % (raw_dx, raw_dy, raw_pos_err, raw_dyaw, dx, dy, pos_err, dyaw)
        )


def main() -> None:
    rclpy.init()
    node = NavSatTruthErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
