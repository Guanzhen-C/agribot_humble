#!/usr/bin/env python3
"""Remove configured vehicle self-returns without changing LiDAR timing or fields."""

import array
import copy
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2, PointField

from agribot_vehicle_config_tools.model import matrix_from_rpy


def remove_self_returns(message, rotation, translation, minimum, maximum, extra_boxes=()):
    if message.width * message.height == 0:
        return message
    if (message.point_step <= 0 or message.row_step < message.width * message.point_step
            or len(message.data) < message.row_step * message.height):
        raise ValueError("Invalid point-cloud layout")
    fields = {field.name: field for field in message.fields}
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if (field is None or field.datatype != PointField.FLOAT32 or field.count != 1
                or field.offset + 4 > message.point_step):
            raise ValueError(f"LiDAR requires a FLOAT32 {name} field")
        coordinates.append(np.ndarray(
            (message.height, message.width),
            dtype=">f4" if message.is_bigendian else "<f4", buffer=message.data,
            offset=field.offset, strides=(message.row_step, message.point_step)).reshape(-1))
    points = np.column_stack(coordinates)
    base_points = points @ np.asarray(rotation).T + np.asarray(translation)
    inside = np.all((base_points >= minimum) & (base_points <= maximum), axis=1)
    for box in extra_boxes:
        inside |= np.all((base_points >= box[:3]) & (base_points <= box[3:]), axis=1)
    keep = np.all(np.isfinite(points), axis=1) & ~inside
    payload = np.ndarray((message.height, message.width, message.point_step), dtype=np.uint8,
                         buffer=message.data,
                         strides=(message.row_step, message.point_step, 1))
    selected = payload.reshape(-1, message.point_step)[keep]
    result = copy.copy(message)
    result.header = copy.deepcopy(message.header)
    result.height = 1
    result.width = len(selected)
    result.row_step = result.width * result.point_step
    result.data = array.array("B", selected.tobytes())
    result.is_dense = True
    return result


class SimulationVehicleCloudFilter(Node):
    def __init__(self):
        super().__init__("simulation_vehicle_cloud_filter")
        source = self.declare_parameter("input_topic", "/configured/unfiltered_lidar_points").value
        target = self.declare_parameter("output_topic", "/lidar/points").value
        frame = self.declare_parameter("lidar_frame", "lidar_link").value
        self.frame = frame
        xyz = self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.0]).value
        rpy = self.declare_parameter("lidar_rpy", [0.0, 0.0, 0.0]).value
        self.minimum = np.asarray(self.declare_parameter("self_min", [-1.0, -0.5, -0.5]).value)
        self.maximum = np.asarray(self.declare_parameter("self_max", [1.0, 0.5, 1.5]).value)
        flat_boxes = self.declare_parameter("extra_self_boxes", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        flat_boxes = np.asarray(flat_boxes if flat_boxes is not None else [], dtype=float)
        if flat_boxes.size % 6 or not np.all(np.isfinite(flat_boxes)):
            raise ValueError("Invalid extra self-filter boxes")
        self.extra_boxes = flat_boxes.reshape(-1, 6)
        if np.any(self.extra_boxes[:, :3] >= self.extra_boxes[:, 3:]):
            raise ValueError("Invalid extra self-filter box bounds")
        if (source == target or any(len(value) != 3 for value in (xyz, rpy, self.minimum, self.maximum))
                or not np.all(np.isfinite([xyz, rpy, self.minimum, self.maximum]))
                or np.any(self.minimum >= self.maximum)):
            raise ValueError("Invalid self-filter configuration")
        self.rotation = np.asarray(matrix_from_rpy(rpy))
        self.translation = np.asarray(xyz)
        self.publisher = self.create_publisher(PointCloud2, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(PointCloud2, source, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message):
        if message.header.frame_id != self.frame:
            self.get_logger().error("Unexpected LiDAR frame; refusing an incorrect self filter",
                                    throttle_duration_sec=5.0)
            return
        try:
            self.publisher.publish(remove_self_returns(message, self.rotation, self.translation,
                                                       self.minimum, self.maximum, self.extra_boxes))
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)


def main():
    # Finish the current message conversion before invalidating its ROS context.
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = SimulationVehicleCloudFilter()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
        finally:
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
