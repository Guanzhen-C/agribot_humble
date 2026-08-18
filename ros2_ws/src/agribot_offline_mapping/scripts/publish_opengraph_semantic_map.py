#!/usr/bin/env python3

"""Publish portable OpenGraph instance metadata as RViz markers."""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def rotation_quaternion(rotation):
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("object rotation matrix produced a zero quaternion")
    return quaternion / norm


class SemanticMapPublisher(Node):
    def __init__(self):
        super().__init__("opengraph_semantic_map_publisher")
        self.declare_parameter("metadata_file", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("minimum_detections", 2)
        self.declare_parameter("maximum_labels", 20)
        self.declare_parameter("label_scale", 0.35)
        self.declare_parameter("show_bounding_boxes", False)

        metadata_path = Path(
            self.get_parameter("metadata_file").get_parameter_value().string_value
        ).expanduser()
        if not metadata_path.is_file():
            raise RuntimeError(f"OpenGraph metadata file does not exist: {metadata_path}")
        document = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(document.get("objects"), list):
            raise RuntimeError("OpenGraph metadata must contain an objects list")

        configured_frame = self.get_parameter("frame_id").value
        self.frame_id = configured_frame or document.get("frame_id", "map")
        self.minimum_detections = int(self.get_parameter("minimum_detections").value)
        self.maximum_labels = int(self.get_parameter("maximum_labels").value)
        self.label_scale = float(self.get_parameter("label_scale").value)
        self.show_bounding_boxes = bool(
            self.get_parameter("show_bounding_boxes").value
        )
        if self.minimum_detections < 1 or self.maximum_labels < 0:
            raise RuntimeError("marker filtering parameters must be nonnegative")
        if self.label_scale <= 0.0:
            raise RuntimeError("label_scale must be positive")

        self.objects = [
            item
            for item in document["objects"]
            if int(item.get("num_detections", 0)) >= self.minimum_detections
        ]
        self.objects.sort(
            key=lambda item: (-int(item.get("num_detections", 0)), int(item["id"]))
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/opengraph/semantic_markers", qos
        )
        self.marker_array = self.build_markers()
        self.timer = self.create_timer(2.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info(
            "Loaded %d stable OpenGraph objects; publishing %d labels in frame %s"
            % (
                len(self.objects),
                min(len(self.objects), self.maximum_labels),
                self.frame_id,
            )
        )

    def build_markers(self):
        markers = []
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        label_ids = {
            int(item["id"])
            for item in self.objects[: self.maximum_labels]
        }
        for item in self.objects:
            object_id = int(item["id"])
            center = np.asarray(item["center"], dtype=np.float64)
            extent = np.maximum(np.asarray(item["extent"], dtype=np.float64), 0.02)
            color = np.clip(np.asarray(item["color"], dtype=np.float64), 0.0, 1.0)
            quaternion = rotation_quaternion(item["rotation"])

            if self.show_bounding_boxes:
                box = Marker()
                box.header.frame_id = self.frame_id
                box.header.stamp = stamp
                box.ns = "opengraph_boxes"
                box.id = object_id
                box.type = Marker.CUBE
                box.action = Marker.ADD
                box.pose.position.x = float(center[0])
                box.pose.position.y = float(center[1])
                box.pose.position.z = float(center[2])
                box.pose.orientation.x = float(quaternion[0])
                box.pose.orientation.y = float(quaternion[1])
                box.pose.orientation.z = float(quaternion[2])
                box.pose.orientation.w = float(quaternion[3])
                box.scale.x = float(extent[0])
                box.scale.y = float(extent[1])
                box.scale.z = float(extent[2])
                box.color.r = float(color[0])
                box.color.g = float(color[1])
                box.color.b = float(color[2])
                box.color.a = 0.06
                markers.append(box)

            if object_id not in label_ids:
                continue
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "opengraph_labels"
            label.id = object_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(center[0])
            label.pose.position.y = float(center[1])
            label.pose.position.z = float(center[2] + 0.5 * extent[2] + 0.15)
            label.pose.orientation.w = 1.0
            label.scale.z = self.label_scale
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            caption = str(item.get("caption", "object"))
            detections = int(item.get("num_detections", 0))
            label.text = f"[{object_id}] {caption} ({detections})"
            markers.append(label)
        return MarkerArray(markers=markers)

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMapPublisher()
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
