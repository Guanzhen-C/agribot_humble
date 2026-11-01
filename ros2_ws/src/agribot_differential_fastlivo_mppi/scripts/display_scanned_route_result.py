#!/usr/bin/env python3
"""Restore completed route displays without starting any navigation action."""
import argparse
import csv
import json
import math
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def read_result(bundle):
    report = json.loads((bundle/"result.json").read_text())
    if not report.get("passed") or report.get("diagnostic_only"):
        raise ValueError("Display requires a successfully verified complete route")
    route = json.loads((bundle/"saved_route.json").read_text())
    with (bundle/"actual_trajectory.csv").open(newline="") as stream:
        actual = [{key: float(row[key]) for key in ("x", "y", "yaw")}
                  for row in csv.DictReader(stream)]
    if not actual or not all(math.isfinite(v) for point in actual for v in point.values()):
        raise ValueError("Actual trajectory must contain finite poses")
    return route, actual


def path_message(poses):
    message = Path()
    message.header.frame_id = "map"
    for point in poses:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(point["x"])
        pose.pose.position.y = float(point["y"])
        pose.pose.orientation.z = math.sin(float(point["yaw"])/2)
        pose.pose.orientation.w = math.cos(float(point["yaw"])/2)
        message.poses.append(pose)
    return message


class ResultDisplay(Node):
    def __init__(self, bundle):
        super().__init__("scanned_saved_route_result_display")
        route, actual = read_result(bundle)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_pub = self.create_publisher(Path, "/validation/full_saved_path", qos)
        self.actual_pub = self.create_publisher(Path, "/validation/actual_path", qos)
        self.markers_pub = self.create_publisher(MarkerArray, "/validation/anchors", qos)
        self.plan, self.actual = path_message(route["path"]), path_message(actual)
        self.markers = MarkerArray()
        for number, index in enumerate(route["anchor_path_indices"], 1):
            point = route["path"][index]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns, marker.id = "saved_104", number
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = float(point["x"]), float(point["y"])
            marker.pose.position.z, marker.pose.orientation.w = .5, 1.
            marker.scale.z = .6
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .6, .05, 1.
            marker.text = str(number)
            self.markers.markers.append(marker)
        self.create_timer(5., self.publish)
        self.publish()

    def publish(self):
        self.plan_pub.publish(self.plan)
        self.actual_pub.publish(self.actual)
        self.markers_pub.publish(self.markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=FilePath)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ResultDisplay(args.bundle.resolve())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
