#!/usr/bin/env python3

"""Publish the semantic A* preference and keepout mask for Nav2."""

from array import array
import json
import math
from pathlib import Path

from agribot_mobile_app.catalog import grid_from_nav2_yaml
from agribot_mobile_app.route_costmap import (
    RouteCostmapError,
    RouteCostmapPolicy,
    build_route_costmap,
)
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class SemanticRouteCostmapPublisher(Node):
    def __init__(self):
        super().__init__("semantic_route_costmap_publisher")
        map_yaml = Path(
            str(self.declare_parameter("map_yaml", "").value)
        ).expanduser().resolve()
        route_file = Path(
            str(self.declare_parameter("route_file", "").value)
        ).expanduser().resolve()
        mask_topic = str(
            self.declare_parameter(
                "mask_topic", "/semantic_navigation/costmap_mask"
            ).value
        )
        info_topic = str(
            self.declare_parameter(
                "filter_info_topic", "/semantic_navigation/costmap_filter_info"
            ).value
        )
        policy = RouteCostmapPolicy(
            core_half_width_m=float(
                self.declare_parameter("route_core_half_width_m", 0.485974).value
            ),
            gradient_width_m=float(
                self.declare_parameter("route_gradient_width_m", 2.0).value
            ),
            maximum_preference_cost=int(
                self.declare_parameter("maximum_preference_cost", 80).value
            ),
        )
        if not map_yaml.is_file() or not route_file.is_file():
            raise RuntimeError("map_yaml and route_file must exist")
        document = json.loads(route_file.read_text(encoding="utf-8"))
        if document.get("schema_version") != 3 or document.get("frame_id") != "map":
            raise RuntimeError("semantic route must use schema 3 in the map frame")
        route = document.get("route")
        centerline = route.get("centerline") if isinstance(route, dict) else None
        avoidance = document.get("avoidance_constraints")
        if not isinstance(avoidance, dict):
            raise RuntimeError("semantic route has no avoidance constraints")
        radius = float(avoidance.get("radius_m", 0.0))
        zones = []
        for node in avoidance.get("nodes", []):
            position = node.get("position") if isinstance(node, dict) else None
            if not isinstance(position, dict):
                raise RuntimeError("semantic avoidance node is invalid")
            zones.append(
                {
                    "selector": str(node.get("selector", "")),
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "radius_m": radius,
                }
            )

        grid = grid_from_nav2_yaml(map_yaml)
        mask = build_route_costmap(grid, centerline, zones, policy)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mask_publisher = self.create_publisher(OccupancyGrid, mask_topic, qos)
        self.info_publisher = self.create_publisher(CostmapFilterInfo, info_topic, qos)

        stamp = self.get_clock().now().to_msg()
        info = CostmapFilterInfo()
        info.header.stamp = stamp
        info.header.frame_id = "map"
        info.type = 0
        info.filter_mask_topic = mask_topic
        info.base = 0.0
        info.multiplier = 1.0
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.info.map_load_time = stamp
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.z = math.sin(0.5 * grid.origin_yaw)
        message.info.origin.orientation.w = math.cos(0.5 * grid.origin_yaw)
        message.data = array("b", mask.reshape(-1).tobytes())
        self.info_publisher.publish(info)
        self.mask_publisher.publish(message)
        self.get_logger().info(
            "Published semantic route mask: %d centerline points, %d keepout zones"
            % (len(centerline), len(zones))
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRouteCostmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (KeyError, OSError, ValueError, json.JSONDecodeError, RouteCostmapError) as error:
        raise SystemExit("error: {}".format(error)) from error
