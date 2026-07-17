#!/usr/bin/env python3
"""
Compare FAST-LIO localization against ground truth.

Uses the ground truth position at startup time as the anchor,
so FAST-LIO's initial odom position maps to wherever the robot
actually is when FAST-LIO starts.

Run alongside ground_truth navigation mode, then start FAST-LIO
separately. In another terminal:
  python3 fastlio_compare.py
"""

import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_full_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class FastLioCompare(Node):
    def __init__(self):
        super().__init__("fastlio_compare")

        fastlio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.gt_sub = self.create_subscription(
            Odometry, "/base_pose_ground_truth", self.handle_gt, 10
        )
        self.fastlio_sub = self.create_subscription(
            Odometry, "/fastlio/odometry", self.handle_fastlio, fastlio_qos
        )

        self.gt_pose = None
        self.gt_anchor = None  # (x, y, yaw) at the moment FAST-LIO initializes
        self.fastlio_pose = None
        self.anchor = None  # (map_to_odom_x, map_to_odom_y, delta_yaw)
        self.fastlio_initialized = False

        self.report_timer = self.create_timer(1.0, self.report)

    def handle_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.gt_pose = (
            p.x,
            p.y,
            yaw_from_full_quat(o.x, o.y, o.z, o.w),
        )
        # Save the GT position when FAST-LIO first initializes
        if self.fastlio_initialized and self.gt_anchor is None:
            self.gt_anchor = self.gt_pose
            self.get_logger().info(
                f"GT anchor captured: ({self.gt_anchor[0]:.3f}, {self.gt_anchor[1]:.3f}, yaw={self.gt_anchor[2]:.4f})"
            )

    def handle_fastlio(self, msg: Odometry):
        self.fastlio_pose = msg.pose.pose

        if not self.fastlio_initialized:
            # Wait until we have ground truth to use as anchor
            if self.gt_pose is None:
                return
            # Use current ground truth position as anchor
            anchor_x, anchor_y, anchor_yaw = self.gt_pose
            odom_yaw = yaw_from_full_quat(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            delta_yaw = anchor_yaw - odom_yaw
            cos_y = math.cos(delta_yaw)
            sin_y = math.sin(delta_yaw)
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            map_to_odom_x = anchor_x - (cos_y * ox - sin_y * oy)
            map_to_odom_y = anchor_y - (sin_y * ox + cos_y * oy)
            self.anchor = (map_to_odom_x, map_to_odom_y, delta_yaw)
            self.fastlio_initialized = True
            self.gt_anchor = self.gt_pose
            self.get_logger().info(
                f"Anchor initialized at GT ({anchor_x:.3f}, {anchor_y:.3f}, yaw={anchor_yaw:.4f}): "
                f"map_to_odom=({map_to_odom_x:.3f}, {map_to_odom_y:.3f}, delta_yaw={delta_yaw:.4f})"
            )

    def compute_fastlio_map_pose(self):
        if not self.fastlio_initialized or self.fastlio_pose is None:
            return None
        mx, my, dy = self.anchor
        ox = self.fastlio_pose.position.x
        oy = self.fastlio_pose.position.y
        cos_y = math.cos(dy)
        sin_y = math.sin(dy)
        map_x = mx + (cos_y * ox - sin_y * oy)
        map_y = my + (sin_y * ox + cos_y * oy)
        odom_yaw = yaw_from_full_quat(
            self.fastlio_pose.orientation.x,
            self.fastlio_pose.orientation.y,
            self.fastlio_pose.orientation.z,
            self.fastlio_pose.orientation.w,
        )
        map_yaw = dy + odom_yaw
        return (map_x, map_y, map_yaw)

    def report(self):
        if self.gt_pose is None:
            self.get_logger().info("Waiting for ground truth...")
            return
        fastlio_map = self.compute_fastlio_map_pose()
        if fastlio_map is None:
            self.get_logger().info("Waiting for FAST-LIO...")
            return

        gx, gy, gyaw = self.gt_pose
        fx, fy, fyaw = fastlio_map
        dx = fx - gx
        dy = fy - gy
        dist = math.sqrt(dx * dx + dy * dy)
        yaw_err = fyaw - gyaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        self.get_logger().info(
            f"GT: ({gx:.3f}, {gy:.3f}, yaw={gyaw:.4f})  "
            f"FL: ({fx:.3f}, {fy:.3f}, yaw={fyaw:.4f})  "
            f"ERR: pos={dist:.3f}m yaw={yaw_err*180/math.pi:.2f}deg"
        )


def main():
    rclpy.init()
    node = FastLioCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
