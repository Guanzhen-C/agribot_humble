#!/usr/bin/env python3
"""Show the entire scanner reference and save manually picked, snapped waypoints."""

from array import array
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


class SelectionStore:
    def __init__(self, map_base, reference_csv, output, snap_distance=0.75):
        self.base, self.reference_csv, self.output = map_base, reference_csv, output
        if output.resolve() in (reference_csv.resolve(), Path(f"{map_base}_routes.json").resolve()):
            raise ValueError("Selection must not overwrite the reference or existing route presets")
        with reference_csv.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.reference = np.array([[float(row[key]) for key in ("map_x_m", "map_y_m", "map_z_m")] for row in rows])
        self.timestamps = np.array([float(row["capture_timestamp_s"]) for row in rows])
        if len(rows) < 2 or not np.isfinite(self.reference).all() or not np.isfinite(self.timestamps).all() or (np.diff(self.timestamps) <= 0).any():
            raise ValueError("Invalid complete scanner reference")
        if not math.isfinite(snap_distance) or snap_distance <= 0:
            raise ValueError("Snap distance must be positive")
        self.snap_distance = snap_distance
        self.reference_hash = hashlib.sha256(reference_csv.read_bytes()).hexdigest()
        geo = Path(f"{map_base}_georeference.yaml")
        geometry = yaml.safe_load(map_base.with_suffix(".yaml").read_text())
        self.binding = {"georeference_sha256": hashlib.sha256(geo.read_bytes()).hexdigest(), "resolution": geometry["resolution"], "origin": geometry["origin"]}
        self.poses, self.indices = [], []
        if output.exists():
            document = json.loads(output.read_text())
            if document.get("map_id") != map_base.name or document.get("reference_csv_sha256") != self.reference_hash or document.get("map_binding") != self.binding:
                raise ValueError("Saved picks belong to another map/reference; choose another output")
            self.poses = document["routes"][0]["poses"]
            self.indices = document["routes"][0]["source_sample_indices"]
            if len(self.poses) != len(self.indices) or len(self.poses) > 110:
                raise ValueError("Invalid saved picks")

    def append(self, x, y, yaw):
        if len(self.poses) >= 110:
            raise ValueError("The selector allows at most 110 waypoints")
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("Non-finite picked pose")
        distances = np.linalg.norm(self.reference[:, :2] - [x, y], axis=1)
        index = int(distances.argmin())
        if distances[index] > self.snap_distance:
            raise ValueError(f"Pick closer to the scanner trace: {distances[index]:.2f} m away")
        if self.indices and self.indices[-1] == index:
            raise ValueError("The same scanner sample is already the last waypoint")
        self.indices.append(index)
        self.poses.append({"x": float(self.reference[index, 0]), "y": float(self.reference[index, 1]), "yaw": yaw})
        self.save()
        return index, float(distances[index])

    def undo(self):
        if self.poses:
            self.poses.pop()
            self.indices.pop()
        self.save()

    def clear(self):
        self.poses.clear()
        self.indices.clear()
        self.save()

    def save(self):
        document = {"schema_version": 1, "map_id": self.base.name, "frame_id": "map", "map_binding": self.binding,
                    "reference_csv": str(self.reference_csv), "reference_csv_sha256": self.reference_hash,
                    "reference_sample_count": len(self.reference), "ready_for_navigation": len(self.poses) >= 2,
                    "routes": [{"id": "manual_scanner_20261030", "label": "人工沿扫描轨迹选点", "vehicle_type": "differential", "poses": self.poses,
                                "source_sample_indices": self.indices, "warning": "人工选点，位置吸附到原始扫描轨迹，朝向来自人工拖动；尚未进行规划及整车碰撞验收。"}]}
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_name(self.output.name + ".tmp")
        temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("reference_csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--snap-distance", type=float, default=0.75)
    args = parser.parse_args()
    store = SelectionStore(args.map_base, args.reference_csv, args.output, args.snap_distance)
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray
    from agribot_mobile_app.catalog import grid_from_nav2_yaml, quaternion_yaw
    rclpy.init()
    node = Node("scanner_reference_point_selector")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    map_publisher = node.create_publisher(OccupancyGrid, "/scanner_selection/map", qos)
    path_publisher = node.create_publisher(NavPath, "/scanner_selection/reference", qos)
    marker_publisher = node.create_publisher(MarkerArray, "/scanner_selection/selected", qos)
    broadcaster = StaticTransformBroadcaster(node)
    origin = TransformStamped()
    origin.header.frame_id, origin.child_frame_id = "map", "scanner_selection_display_origin"
    origin.header.stamp = node.get_clock().now().to_msg()
    origin.transform.rotation.w = 1.0
    broadcaster.sendTransform(origin)
    grid = grid_from_nav2_yaml(args.map_base.with_suffix(".yaml"))
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.info.width, message.info.height = grid.width, grid.height
    message.info.resolution = grid.resolution
    message.info.origin.position.x, message.info.origin.position.y = grid.origin_x, grid.origin_y
    message.info.origin.orientation.z, message.info.origin.orientation.w = math.sin(grid.origin_yaw / 2), math.cos(grid.origin_yaw / 2)
    message.data = array("b", grid.data)
    map_publisher.publish(message)
    reference = NavPath()
    reference.header = message.header
    for position in store.reference:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(position[0]), float(position[1]), 0.04
        pose.pose.orientation.w = 1.0
        reference.poses.append(pose)
    path_publisher.publish(reference)

    def publish_selected():
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, pose in enumerate(store.poses):
            arrow = Marker()
            arrow.header.frame_id, arrow.ns, arrow.id = "map", "manual_waypoints", index
            arrow.type = Marker.ARROW
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = pose["x"], pose["y"], 0.08
            arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(pose["yaw"] / 2), math.cos(pose["yaw"] / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.8, 0.20, 0.20
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.9, 0.45, 0.05, 1.0
            markers.markers.append(arrow)
            label = Marker()
            label.header.frame_id, label.ns, label.id = "map", "manual_labels", index
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = pose["x"], pose["y"] + 0.35, 0.2
            label.pose.orientation.w = 1.0
            label.scale.z, label.color.a = 0.6, 1.0
            label.text = str(index + 1)
            markers.markers.append(label)
        marker_publisher.publish(markers)

    def picked(message):
        try:
            if message.header.frame_id.lstrip("/") != "map":
                raise ValueError("Pick frame must be map")
            q = message.pose.orientation
            index, distance = store.append(message.pose.position.x, message.pose.position.y, quaternion_yaw(q.x, q.y, q.z, q.w))
            publish_selected()
            node.get_logger().info(f"Saved point {len(store.poses)}: scanner sample {index}, snap {distance:.3f} m")
        except ValueError as error:
            node.get_logger().warning(str(error))

    def service(action):
        def callback(_request, response):
            action()
            publish_selected()
            response.success, response.message = True, f"{len(store.poses)} selected waypoints"
            return response
        return callback

    node.create_subscription(PoseStamped, "/scanner_selection/pick", picked, 10)
    node.create_service(Trigger, "/scanner_selection/undo", service(store.undo))
    node.create_service(Trigger, "/scanner_selection/clear", service(store.clear))
    publish_selected()
    node.get_logger().info(f"Full reference: {len(store.reference)} samples; restored {len(store.poses)} manual points; output={store.output}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
