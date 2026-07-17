#!/usr/bin/env python3
"""
Diagnose FAST-LIO localization vs ground truth.

Reports yaw and position error by anchoring FAST-LIO's odom-frame
position into the map frame using the same KISS localization logic,
then comparing against /base_pose_ground_truth.
"""

import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def yaw_from_full_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DiagnoseNode(Node):
    def __init__(self):
        super().__init__("fastlio_yaw_diagnose")

        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.gt_anchor_set = False

        fastlio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.raw_msg = None
        self.bridged_msg = None
        self.gt_msg = None
        self.anchor = None  # (map_to_odom_x, map_to_odom_y, delta_yaw)

        self.raw_sub = self.create_subscription(
            Odometry, "/Odometry", self.handle_raw, fastlio_qos
        )
        self.bridged_sub = self.create_subscription(
            Odometry, "/fastlio/odometry", self.handle_bridged, fastlio_qos
        )
        self.gt_sub = self.create_subscription(
            Odometry, "/base_pose_ground_truth", self.handle_gt, 10
        )

        self.timer = self.create_timer(1.0, self.report)

    def try_anchor(self):
        if self.anchor is not None or self.bridged_msg is None or self.initial_x is None:
            return
        odom_yaw = yaw_from_full_quat(
            self.bridged_msg.pose.pose.orientation.x,
            self.bridged_msg.pose.pose.orientation.y,
            self.bridged_msg.pose.pose.orientation.z,
            self.bridged_msg.pose.pose.orientation.w,
        )
        delta_yaw = self.initial_yaw - odom_yaw
        cos_y = math.cos(delta_yaw)
        sin_y = math.sin(delta_yaw)
        ox = self.bridged_msg.pose.pose.position.x
        oy = self.bridged_msg.pose.pose.position.y
        map_to_odom_x = self.initial_x - (cos_y * ox - sin_y * oy)
        map_to_odom_y = self.initial_y - (sin_y * ox + cos_y * oy)
        self.anchor = (map_to_odom_x, map_to_odom_y, delta_yaw)
        self.get_logger().info(
            f"Anchor initialized: map_to_odom=({map_to_odom_x:.3f}, {map_to_odom_y:.3f}, delta_yaw={delta_yaw:.4f})"
        )

    def handle_raw(self, msg):
        self.raw_msg = msg

    def handle_bridged(self, msg):
        self.bridged_msg = msg
        self.try_anchor()

    def handle_gt(self, msg):
        self.gt_msg = msg
        if not self.gt_anchor_set:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            self.initial_x = p.x
            self.initial_y = p.y
            self.initial_yaw = yaw_from_full_quat(o.x, o.y, o.z, o.w)
            self.gt_anchor_set = True
            self.get_logger().info(
                f"GT anchor set: ({self.initial_x:.3f}, {self.initial_y:.3f}, yaw={self.initial_yaw:.4f})"
            )
            self.try_anchor()

    def bridged_to_map(self):
        if self.anchor is None or self.bridged_msg is None:
            return None
        mx, my, dy = self.anchor
        ox = self.bridged_msg.pose.pose.position.x
        oy = self.bridged_msg.pose.pose.position.y
        cos_y = math.cos(dy)
        sin_y = math.sin(dy)
        map_x = mx + (cos_y * ox - sin_y * oy)
        map_y = my + (sin_y * ox + cos_y * oy)
        odom_yaw = yaw_from_full_quat(
            self.bridged_msg.pose.pose.orientation.x,
            self.bridged_msg.pose.pose.orientation.y,
            self.bridged_msg.pose.pose.orientation.z,
            self.bridged_msg.pose.pose.orientation.w,
        )
        map_yaw = dy + odom_yaw
        return (map_x, map_y, map_yaw)

    def report(self):
        gt_yaw = None
        raw_yaw = None
        bridged_yaw = None
        gt_pos = None

        if self.gt_msg:
            o = self.gt_msg.pose.pose.orientation
            gt_yaw = yaw_from_full_quat(o.x, o.y, o.z, o.w)
            p = self.gt_msg.pose.pose.position
            gt_pos = (p.x, p.y)

        if self.raw_msg:
            o = self.raw_msg.pose.pose.orientation
            raw_yaw = yaw_from_full_quat(o.x, o.y, o.z, o.w)

        if self.bridged_msg:
            o = self.bridged_msg.pose.pose.orientation
            bridged_yaw = yaw_from_full_quat(o.x, o.y, o.z, o.w)

        fl_map = self.bridged_to_map()

        line = ""
        if gt_pos is not None and gt_yaw is not None:
            line += f"GT({gt_pos[0]:.3f},{gt_pos[1]:.3f},yaw={gt_yaw*180/math.pi:.2f}deg) "

        if fl_map is not None:
            line += f"FL_map({fl_map[0]:.3f},{fl_map[1]:.3f},yaw={fl_map[2]*180/math.pi:.2f}deg) "

        if raw_yaw is not None:
            line += f"RAW_yaw={raw_yaw:.4f}({raw_yaw*180/math.pi:.2f}deg) "
        if bridged_yaw is not None:
            line += f"Bridged_yaw={bridged_yaw:.4f}({bridged_yaw*180/math.pi:.2f}deg) "

        if raw_yaw is not None and bridged_yaw is not None:
            diff = bridged_yaw - raw_yaw
            line += f"offset={diff:.4f} "

        if gt_yaw is not None and bridged_yaw is not None:
            yaw_err = bridged_yaw - gt_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            line += f"yaw_ERR={yaw_err*180/math.pi:.2f}deg "

        if gt_pos is not None and fl_map is not None:
            dx = fl_map[0] - gt_pos[0]
            dy = fl_map[1] - gt_pos[1]
            pos_err = math.sqrt(dx * dx + dy * dy)
            line += f"pos_ERR={pos_err:.3f}m "
            yaw_map_err = fl_map[2] - gt_yaw
            yaw_map_err = math.atan2(math.sin(yaw_map_err), math.cos(yaw_map_err))
            line += f"map_yaw_ERR={yaw_map_err*180/math.pi:.2f}deg"

        if line:
            self.get_logger().info(line)
        else:
            self.get_logger().info("Waiting for data...")


def main():
    rclpy.init()
    node = DiagnoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
