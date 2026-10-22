"""Read-only checks of simulation sensor streams; never publishes commands."""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as ImageMessage, Imu, NavSatFix, PointCloud2
from std_msgs.msg import UInt8
from tf2_ros import Buffer, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=12)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "86" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Restricted to isolated local K2 simulation domain 86")
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error("Observation time must be within 1..60 seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("k2_read_only_sensor_check")
    transforms = Buffer(cache_time=Duration(seconds=10))
    listener = TransformListener(transforms, node)
    stamps, counts, frames, samples = defaultdict(list), defaultdict(int), {}, {}
    clocks, subscriptions = [], []

    def on_clock(message):
        clocks.append((time.monotonic(), stamp_seconds(message.clock)))

    subscriptions.append(node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data))

    def on_sensor(topic, message):
        counts[topic] += 1
        if hasattr(message, "header"):
            stamps[topic].append(stamp_seconds(message.header.stamp))
            frames[topic] = message.header.frame_id
        if isinstance(message, ImageMessage):
            channels = 3 if message.encoding in ("rgb8", "bgr8") else 1
            array = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            array = array[:, :message.width * channels].reshape(message.height, message.width, channels)
            if message.encoding == "bgr8":
                array = array[:, :, ::-1]
            samples[topic] = {"size": [message.width, message.height], "encoding": message.encoding,
                              "pixel_std": float(array.std())}
            if len(stamps[topic]) == 5:
                Image.fromarray(array.copy() if channels == 3 else array[:, :, 0].copy()).save(args.output / "camera.png")
        elif isinstance(message, PointCloud2) and topic not in samples:
            offsets = {field.name: field for field in message.fields}
            fields = [offsets[key] for key in ("x", "y", "z")]
            if any(field.datatype != 7 for field in fields):
                raise ValueError("Expected float32 simulated lidar XYZ")
            dtype = np.dtype({"names": ["x", "y", "z"],
                              "formats": [">f4" if message.is_bigendian else "<f4"] * 3,
                              "offsets": [field.offset for field in fields], "itemsize": message.point_step})
            data = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                              strides=(message.row_step, message.point_step)).reshape(-1)
            xyz = np.column_stack([data[key] for key in ("x", "y", "z")])
            finite = np.isfinite(xyz).all(axis=1)
            samples[topic] = {"points": len(data), "finite_xyz": int(finite.sum()),
                              "fields": list(offsets),
                              "range_p50_m": float(np.median(np.linalg.norm(xyz[finite], axis=1)))}
        elif isinstance(message, Imu):
            a = message.linear_acceleration
            q = message.orientation
            samples[topic] = {"acceleration_norm_mps2": math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z),
                              "orientation_quaternion_norm": math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)}
        elif isinstance(message, NavSatFix):
            samples[topic] = {"latitude": message.latitude, "longitude": message.longitude,
                              "altitude": message.altitude, "status": message.status.status}
        elif isinstance(message, UInt8):
            samples[topic] = {"quality": message.data}

    topics = [("/lidar/points", PointCloud2), ("/configured/localization_points", PointCloud2),
              ("/camera/rgb/image_raw", ImageMessage), ("/imu/data", Imu),
              ("/navsat/fix", NavSatFix), ("/rtk/fix_quality", UInt8)]
    for topic, kind in topics:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, name=topic: on_sensor(name, msg), qos_profile_sensor_data))
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {"scope": "read-only current simulated sensor streams; no motion commands",
              "topics": {}, "samples": samples, "tf_to_base_link": {}}
    for topic, _ in topics:
        values = stamps[topic]
        report["topics"][topic] = {"messages": counts[topic], "frame": frames.get(topic)}
        if len(values) > 1:
            deltas = np.diff(values)
            report["topics"][topic].update(
                sim_hz=(len(values)-1)/(values[-1]-values[0]),
                nonmonotonic=int(np.sum(deltas <= 0)),
                interval_median_ms=float(np.median(deltas)*1000),
                interval_p95_ms=float(np.percentile(deltas, 95)*1000))
        if frames.get(topic):
            try:
                transform = transforms.lookup_transform("base_link", frames[topic], Time())
                t = transform.transform.translation
                report["tf_to_base_link"][topic] = {"available": True, "xyz": [t.x, t.y, t.z]}
            except Exception as error:
                report["tf_to_base_link"][topic] = {"available": False, "error": str(error)}
    lidar, camera = stamps["/lidar/points"], stamps["/camera/rgb/image_raw"]
    if lidar and camera:
        differences = [min(abs(c-t) for t in lidar) for c in camera if lidar[0] <= c <= lidar[-1]]
        report["camera_nearest_lidar_stamp_ms"] = {
            "median": float(np.median(differences)*1000),
            "p95": float(np.percentile(differences, 95)*1000)}
    if len(clocks) > 1:
        report["real_time_factor"] = (clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0])
    (args.output / "sensor_health.json").write_text(json.dumps(report, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(report, indent=2))
