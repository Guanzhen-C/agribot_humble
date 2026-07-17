#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def bresenham(x0: int, y0: int, x1: int, y1: int):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            yield x, y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            yield x, y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    yield x1, y1


class GroundTruthScanMapper(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_scan_mapper")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.resolution = float(self.declare_parameter("resolution", 0.05).value)
        self.origin_x = float(self.declare_parameter("origin_x", -20.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -20.0).value)
        self.width_m = float(self.declare_parameter("width", 140.0).value)
        self.height_m = float(self.declare_parameter("height", 140.0).value)
        self.max_usable_range = float(self.declare_parameter("max_usable_range", 5.5).value)
        self.max_range = float(self.declare_parameter("max_range", 6.1).value)
        self.beam_stride = max(1, int(self.declare_parameter("beam_stride", 1).value))
        self.publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self.occupied_log_odds = float(self.declare_parameter("occupied_log_odds", 0.85).value)
        self.free_log_odds = float(self.declare_parameter("free_log_odds", -0.4).value)
        self.min_log_odds = float(self.declare_parameter("min_log_odds", -4.0).value)
        self.max_log_odds = float(self.declare_parameter("max_log_odds", 4.0).value)
        self.occupied_threshold = float(self.declare_parameter("occupied_threshold", 1.0).value)
        self.free_threshold = float(self.declare_parameter("free_threshold", -1.0).value)

        self.width = int(round(self.width_m / self.resolution))
        self.height = int(round(self.height_m / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.seen = np.zeros((self.height, self.width), dtype=np.bool_)
        self.pose_xyyaw: Optional[Tuple[float, float, float]] = None
        self.scan_frame_offsets = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", qos)
        self.meta_pub = self.create_publisher(MapMetaData, "map_metadata", qos)
        self.create_subscription(Odometry, self.ground_truth_topic, self.handle_ground_truth, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.handle_scan, 10)
        self.create_timer(1.0 / max(self.publish_hz, 1e-3), self.publish_map)

    def world_to_cell(self, x: float, y: float):
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def handle_ground_truth(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)
        self.pose_xyyaw = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(yaw))

    def get_scan_frame_offset(self, scan_frame: str) -> Tuple[float, float, float]:
        cached = self.scan_frame_offsets.get(scan_frame)
        if cached is not None:
            return cached
        transform = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, rclpy.time.Time())
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q)
        offset = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(yaw),
        )
        self.scan_frame_offsets[scan_frame] = offset
        return offset

    def integrate_ray(self, start_cell, end_cell, mark_occupied: bool):
        cells = list(bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1]))
        free_cells = cells[:-1] if mark_occupied else cells
        for cx, cy in free_cells:
            self.seen[cy, cx] = True
            self.log_odds[cy, cx] = np.clip(
                self.log_odds[cy, cx] + self.free_log_odds, self.min_log_odds, self.max_log_odds
            )
        if mark_occupied and cells:
            cx, cy = cells[-1]
            self.seen[cy, cx] = True
            self.log_odds[cy, cx] = np.clip(
                self.log_odds[cy, cx] + self.occupied_log_odds, self.min_log_odds, self.max_log_odds
            )

    def handle_scan(self, msg: LaserScan) -> None:
        if self.pose_xyyaw is None:
            return
        robot_x, robot_y, robot_yaw = self.pose_xyyaw
        try:
            sensor_offset_x, sensor_offset_y, sensor_yaw_offset = self.get_scan_frame_offset(
                msg.header.frame_id or self.base_frame
            )
        except TransformException:
            return

        sensor_x = robot_x + math.cos(robot_yaw) * sensor_offset_x - math.sin(robot_yaw) * sensor_offset_y
        sensor_y = robot_y + math.sin(robot_yaw) * sensor_offset_x + math.cos(robot_yaw) * sensor_offset_y
        sensor_yaw = robot_yaw + sensor_yaw_offset
        start_cell = self.world_to_cell(sensor_x, sensor_y)
        if start_cell is None:
            return

        angle = msg.angle_min
        max_range = min(self.max_range, msg.range_max)
        max_usable_range = min(self.max_usable_range, max_range)
        for index, distance in enumerate(msg.ranges):
            if index % self.beam_stride != 0:
                angle += msg.angle_increment
                continue
            hit_in_usable_range = math.isfinite(distance) and msg.range_min < distance < max_usable_range
            ray_length = min(distance, max_range) if math.isfinite(distance) else max_range
            end_x = sensor_x + ray_length * math.cos(sensor_yaw - angle)
            end_y = sensor_y + ray_length * math.sin(sensor_yaw - angle)
            end_cell = self.world_to_cell(end_x, end_y)
            if end_cell is not None:
                self.integrate_ray(start_cell, end_cell, hit_in_usable_range)
            angle += msg.angle_increment

    def publish_map(self) -> None:
        meta = MapMetaData()
        meta.map_load_time = self.get_clock().now().to_msg()
        meta.resolution = self.resolution
        meta.width = self.width
        meta.height = self.height
        meta.origin.position.x = self.origin_x
        meta.origin.position.y = self.origin_y
        meta.origin.orientation.w = 1.0
        self.meta_pub.publish(meta)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info = meta

        data = np.full((self.height, self.width), -1, dtype=np.int8)
        occupied = self.seen & (self.log_odds >= self.occupied_threshold)
        free = self.seen & (self.log_odds <= self.free_threshold)
        unknown_seen = self.seen & ~(occupied | free)
        data[free] = 0
        data[occupied] = 100
        data[unknown_seen] = 50
        grid.data = data.reshape(-1).tolist()
        self.map_pub.publish(grid)


def main() -> None:
    rclpy.init()
    node = GroundTruthScanMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
