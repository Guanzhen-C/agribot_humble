#!/usr/bin/env python3

"""Publish a validated semantic route preview without commanding Nav2."""

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


def point(position, z_offset=0.0):
    message = Point()
    message.x = float(position["x"])
    message.y = float(position["y"])
    message.z = float(position.get("z", 0.0)) + z_offset
    return message


class SemanticRoutePublisher(Node):
    def __init__(self):
        super().__init__("semantic_route_publisher")
        self.declare_parameter("route_file", "")
        self.declare_parameter("frame_id", "")

        route_path = Path(str(self.get_parameter("route_file").value)).expanduser()
        if not route_path.is_file():
            raise RuntimeError("semantic route file does not exist: {}".format(route_path))
        self.route = json.loads(route_path.read_text(encoding="utf-8"))
        if self.route.get("schema_version") != 3:
            raise RuntimeError("unsupported semantic route schema version")
        execution_policy = self.route.get("execution_policy", {})
        if (
            not execution_policy.get("preview_only", False)
            or execution_policy.get("execution_authorized") is not False
        ):
            raise RuntimeError("semantic route must be marked preview_only")
        configured_frame = str(self.get_parameter("frame_id").value)
        self.frame_id = configured_frame or str(self.route.get("frame_id", "map"))
        self.poses = self.route.get("route", {}).get("poses", [])
        if not self.poses:
            raise RuntimeError("semantic route contains no poses")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            PathMessage, "/semantic_navigation/route_preview", qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/semantic_navigation/route_preview_markers", qos
        )
        self.path = self.build_path()
        self.markers = self.build_markers()
        self.timer = self.create_timer(2.0, self.publish)
        self.publish()
        self.get_logger().info(
            "Loaded preview-only semantic route %s with %d poses and %d requested stops"
            % (
                self.route.get("route_id", "unknown"),
                len(self.poses),
                len(self.route.get("resolved_stops", [])),
            )
        )

    def build_path(self):
        path = PathMessage()
        path.header.frame_id = self.frame_id
        for item in self.poses:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(item["position"]["x"])
            pose.pose.position.y = float(item["position"]["y"])
            pose.pose.position.z = float(item["position"].get("z", 0.0)) + 0.22
            yaw = float(item["yaw"])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)
        return path

    def marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def build_markers(self):
        markers = []
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.append(clear)

        route_line = self.marker("route_preview", 0, Marker.LINE_STRIP)
        route_line.scale.x = 0.30
        route_line.color = color(1.0, 0.35, 0.02, 1.0)
        for item in self.poses:
            route_line.points.append(point(item["position"], 0.24))
        markers.append(route_line)

        stop_points = self.marker("route_stops", 0, Marker.SPHERE_LIST)
        stop_points.scale.x = 0.85
        stop_points.scale.y = 0.85
        stop_points.scale.z = 0.40
        stop_points.color = color(0.05, 0.45, 1.0, 1.0)
        for stop in self.route.get("resolved_stops", []):
            stop_points.points.append(point(stop["position"], 0.35))
        markers.append(stop_points)

        for index, stop in enumerate(self.route.get("resolved_stops", [])):
            label = self.marker("route_stop_labels", index, Marker.TEXT_VIEW_FACING)
            label.pose.position = point(stop["position"], 1.10)
            label.scale.z = 0.65
            label.color = color(0.05, 0.20, 1.0, 1.0)
            label.text = "{}: {}".format(index + 1, stop["selector"])
            markers.append(label)

        landmark_anchor_links = self.marker(
            "route_landmark_anchor_links", 0, Marker.LINE_LIST
        )
        landmark_anchor_links.scale.x = 0.10
        landmark_anchor_links.color = color(0.15, 0.55, 1.0, 0.85)
        landmark_anchor_points = self.marker(
            "route_landmark_anchors", 0, Marker.SPHERE_LIST
        )
        landmark_anchor_points.scale.x = 0.55
        landmark_anchor_points.scale.y = 0.55
        landmark_anchor_points.scale.z = 0.30
        landmark_anchor_points.color = color(0.15, 0.80, 1.0, 1.0)
        for stop in self.route.get("resolved_stops", []):
            if stop.get("kind") != "landmark":
                continue
            landmark_anchor_links.points.append(point(stop["position"], 0.30))
            landmark_anchor_links.points.append(
                point(stop["navigation_anchor_position"], 0.30)
            )
            landmark_anchor_points.points.append(
                point(stop["navigation_anchor_position"], 0.32)
            )
        markers.append(landmark_anchor_links)
        markers.append(landmark_anchor_points)

        avoidance = self.route.get("avoidance_constraints", {})
        radius = float(avoidance.get("radius_m", 0.0))
        for index, semantic_node in enumerate(avoidance.get("nodes", [])):
            zone = self.marker("route_avoidance_zones", index, Marker.CYLINDER)
            zone.pose.position = point(semantic_node["position"], 0.08)
            zone.scale.x = max(0.10, 2.0 * radius)
            zone.scale.y = max(0.10, 2.0 * radius)
            zone.scale.z = 0.12
            zone.color = color(1.0, 0.05, 0.05, 0.35)
            markers.append(zone)

            label = self.marker(
                "route_avoidance_labels", index, Marker.TEXT_VIEW_FACING
            )
            label.pose.position = point(semantic_node["position"], 0.90)
            label.scale.z = 0.55
            label.color = color(0.85, 0.0, 0.0, 1.0)
            label.text = "AVOID: {}".format(semantic_node["selector"])
            markers.append(label)
        return MarkerArray(markers=markers)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.path_publisher.publish(self.path)
        self.marker_publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticRoutePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
