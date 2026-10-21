#!/usr/bin/env python3
"""Describe Gazebo's simultaneous ray sample without inventing rolling scans."""

import array
import copy
import math
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import LivelinessPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2, PointField


LOCALIZATION_QOS = QoSProfile(
    depth=2, reliability=ReliabilityPolicy.RELIABLE, liveliness=LivelinessPolicy.AUTOMATIC)


def instantaneous_scan(message, scan_period_sec):
    if not math.isfinite(scan_period_sec) or scan_period_sec <= 0.0:
        raise ValueError("scan_period_sec must be positive and finite")
    if message.width * message.height == 0:
        return message
    field = next((field for field in message.fields if field.name == "time"), None)
    if field is None or field.datatype != PointField.FLOAT32 or field.count != 1:
        raise ValueError("Gazebo scan must have one FLOAT32 time field")
    if (field.offset + 4 > message.point_step or
            message.row_step < message.width * message.point_step or
            len(message.data) < message.row_step * message.height):
        raise ValueError("Invalid point-cloud layout")
    result = copy.copy(message)
    result.header = copy.deepcopy(message.header)
    result.data = array.array("B", message.data)
    measured_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    begin_ns = max(0, measured_ns - round(scan_period_sec * 1_000_000_000))
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(begin_ns, 1_000_000_000)
    # LIO frontends require a positive final offset to suppress their azimuth
    # fallback. All points still describe the original, same measurement time.
    times = np.ndarray(
        (message.height, message.width), dtype=">f4" if message.is_bigendian else "<f4",
        buffer=result.data, offset=field.offset,
        strides=(message.row_step, message.point_step))
    times.fill((measured_ns - begin_ns) * 1e-9)
    return result


class SimulationLidarTimeAdapter(Node):
    def __init__(self):
        super().__init__("simulation_lidar_time_adapter")
        input_topic = self.declare_parameter("input_topic", "/lidar/points").value
        output_topic = self.declare_parameter(
            "output_topic", "/configured/localization_points").value
        self.period = self.declare_parameter("scan_period_sec", 0.1).value
        if not math.isfinite(self.period) or self.period <= 0.0:
            raise ValueError("scan_period_sec must be positive and finite")
        if input_topic == output_topic:
            raise ValueError("The adapter must not overwrite its input topic")
        # FAST-LIVO2 requests reliable LiDAR delivery; this also satisfies
        # best-effort FAST-LIO subscribers without an unbounded history.
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, LOCALIZATION_QOS)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message):
        try:
            self.publisher.publish(instantaneous_scan(message, self.period))
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
        node = SimulationLidarTimeAdapter()
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
