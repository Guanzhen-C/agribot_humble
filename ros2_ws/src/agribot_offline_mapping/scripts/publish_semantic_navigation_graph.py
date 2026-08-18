#!/usr/bin/env python3

"""Publish a semantic navigation graph as transient RViz markers."""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def point_from_position(position, z_offset=0.0):
    point = Point()
    point.x = float(position["x"])
    point.y = float(position["y"])
    point.z = float(position.get("z", 0.0)) + z_offset
    return point


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


class SemanticNavigationGraphPublisher(Node):
    def __init__(self):
        super().__init__("semantic_navigation_graph_publisher")
        self.declare_parameter("graph_file", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("show_place_labels", True)
        self.declare_parameter("show_place_summaries", False)
        self.declare_parameter("label_scale", 1.2)
        self.declare_parameter("place_marker_scale", 1.2)
        self.declare_parameter("connection_width", 0.18)

        graph_path = Path(str(self.get_parameter("graph_file").value)).expanduser()
        if not graph_path.is_file():
            raise RuntimeError("semantic navigation graph does not exist: {}".format(graph_path))
        self.graph = json.loads(graph_path.read_text(encoding="utf-8"))
        if self.graph.get("schema_version") != 3:
            raise RuntimeError("unsupported semantic navigation graph schema version")
        if any(field in self.graph for field in ("nodes", "edges", "place_edges")):
            raise RuntimeError("semantic navigation graph contains legacy dense route fields")
        configured_frame = str(self.get_parameter("frame_id").value)
        self.frame_id = configured_frame or str(self.graph.get("frame_id", "map"))
        self.show_place_labels = bool(self.get_parameter("show_place_labels").value)
        self.show_place_summaries = bool(
            self.get_parameter("show_place_summaries").value
        )
        self.label_scale = float(self.get_parameter("label_scale").value)
        self.place_marker_scale = float(
            self.get_parameter("place_marker_scale").value
        )
        self.connection_width = float(self.get_parameter("connection_width").value)
        if min(
            self.label_scale, self.place_marker_scale, self.connection_width
        ) <= 0.0:
            raise RuntimeError("marker sizes must be positive")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/semantic_navigation/topology_markers", qos
        )
        self.markers = self.build_markers()
        self.timer = self.create_timer(2.0, self.publish)
        self.publish()
        self.get_logger().info(
            "Loaded semantic graph with %d places, %d landmarks and %d connections"
            % (
                len(self.graph.get("places", [])),
                len(self.graph.get("landmarks", [])),
                len(self.graph.get("connections", [])),
            )
        )

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

        places = {item["id"]: item for item in self.graph.get("places", [])}
        landmarks = {
            item["id"]: item for item in self.graph.get("landmarks", [])
        }
        semantic_nodes = dict(places)
        semantic_nodes.update(landmarks)
        connections = self.marker("place_connections", 0, Marker.LINE_LIST)
        connections.scale.x = self.connection_width
        connections.color = color(0.05, 0.95, 0.75, 0.9)
        associations = self.marker("landmark_associations", 0, Marker.LINE_LIST)
        associations.scale.x = 0.035
        associations.color = color(0.25, 0.65, 1.0, 0.35)
        for connection in self.graph.get("connections", []):
            marker = (
                connections
                if connection.get("kind") == "drivable"
                else associations
            )
            centerline = connection.get("centerline")
            if (
                connection.get("kind") == "drivable"
                and isinstance(centerline, list)
                and len(centerline) >= 2
            ):
                for first, second in zip(centerline[:-1], centerline[1:]):
                    marker.points.append(point_from_position(first, 0.12))
                    marker.points.append(point_from_position(second, 0.12))
                continue
            marker.points.append(
                point_from_position(
                    semantic_nodes[connection["source"]]["position"], 0.12
                )
            )
            marker.points.append(
                point_from_position(
                    semantic_nodes[connection["target"]]["position"], 0.12
                )
            )
        markers.append(connections)
        markers.append(associations)

        place_nodes = self.marker("place_nodes", 0, Marker.SPHERE_LIST)
        place_nodes.scale.x = self.place_marker_scale
        place_nodes.scale.y = self.place_marker_scale
        place_nodes.scale.z = self.place_marker_scale
        place_nodes.color = color(1.0, 0.25, 0.05, 1.0)
        for place in self.graph.get("places", []):
            place_nodes.points.append(point_from_position(place["position"], 0.28))
        markers.append(place_nodes)

        landmark_nodes = self.marker("landmark_nodes", 0, Marker.SPHERE_LIST)
        landmark_nodes.scale.x = 0.22
        landmark_nodes.scale.y = 0.22
        landmark_nodes.scale.z = 0.22
        landmark_nodes.color = color(0.15, 0.55, 1.0, 0.9)
        for landmark in self.graph.get("landmarks", []):
            landmark_nodes.points.append(
                point_from_position(landmark["position"], 0.20)
            )
        markers.append(landmark_nodes)

        if self.show_place_labels:
            for index, place in enumerate(self.graph.get("places", [])):
                label = self.marker("place_labels", index, Marker.TEXT_VIEW_FACING)
                label.pose.position = point_from_position(place["position"], 0.85)
                label.scale.z = self.label_scale
                label.color = color(1.0, 1.0, 1.0, 0.95)
                summary = place.get("semantic_summary", [])
                label.text = str(place.get("name", place["id"]))
                if self.show_place_summaries and summary:
                    label.text += ": " + " / ".join(summary[:2])
                markers.append(label)
        return MarkerArray(markers=markers)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticNavigationGraphPublisher()
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
