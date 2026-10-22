"""Check a running isolated K2 simulation without stopping its GUI or launch."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMessage
from ament_index_python.packages import get_package_prefix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--distance", type=float, default=0,
                        help="Opt into a simulated forward motion test; zero is observation only")
    parser.add_argument("--lateral", type=float, default=0)
    args = parser.parse_args()
    if (os.environ.get("ROS_DOMAIN_ID") != "86" or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or os.environ.get("GAZEBO_MASTER_URI") != "http://127.0.0.1:11375"):
        parser.error("This check is restricted to local simulation domain 86 / master 11375")
    if not math.isfinite(args.distance) or not 0 <= args.distance <= 10:
        parser.error("Distance must be finite, 0..10 m")
    if not math.isfinite(args.lateral) or abs(args.lateral) > 2:
        parser.error("Lateral offset must be finite, within 2 m")
    args.output.mkdir(parents=True, exist_ok=False)
    script = Path(get_package_prefix("agribot_vehicle_description")) / "lib/agribot_vehicle_description/verify_algorithm_runtime.py"
    spec = importlib.util.spec_from_file_location("k2_runtime_probe", script)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    root = args.scene_manifest.resolve().parent
    report = {"scope": "short isolated simulation check, not full-campus acceptance",
              "scene_manifest": str(args.scene_manifest.resolve()), "status": "RUNNING"}
    rclpy.init()
    node = runtime.Probe("fastlivo_rtk")
    latest = []

    def camera(message):
        if message.encoding not in ("rgb8", "bgr8"):
            return
        data = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        data = data[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        if message.encoding == "bgr8":
            data = data[:, :, ::-1]
        latest[:] = [(data, message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)]

    subscription = node.create_subscription(ImageMessage, "/camera/rgb/image_raw", camera,
                                             qos_profile_sensor_data)
    started = time.monotonic()
    try:
        node.ready(120)
        for role in ("server", "client"):
            state = json.loads((root / "gaussian" / f"render_status_{role}.json").read_text())
            if state["state"] != "rendered" or state["loaded_gaussians"] != 8051840:
                raise RuntimeError(f"Full Gaussian renderer not active: {role}")
            report[role + "_renderer"] = state
        if not node.until(lambda: bool(latest), 10):
            raise RuntimeError("No native Gazebo camera image")
        image, stamp = latest[0]
        Image.fromarray(image).save(args.output / "camera.png")
        report["camera"] = {"stamp": stamp, "size": [image.shape[1], image.shape[0]],
                             "standard_deviation": float(image.std()),
                             "rgb_min": image.min(axis=(0, 1)).tolist(),
                             "rgb_max": image.max(axis=(0, 1)).tolist()}
        if image.std() < 5:
            raise RuntimeError("Camera image is near blank")
        if args.distance:
            yaw = runtime.pose_yaw(node.estimate)
            start = node.estimate.position
            target = runtime.pose(start.x + args.distance * math.cos(yaw) - args.lateral * math.sin(yaw),
                                  start.y + args.distance * math.sin(yaw) + args.lateral * math.cos(yaw), yaw)
            plan = node.plan([target], False)
            report["path_poses"] = len(plan.poses)
            (args.output / "path.json").write_text(json.dumps([
                [pose.pose.position.x, pose.pose.position.y] for pose in plan.poses]))
            status, displacement = node.follow(plan, 180)
            report["controller_status"] = status
            report["physical_displacement_m"] = displacement
            report["target_xy"] = [target.pose.position.x, target.pose.position.y]
            node.until(lambda: False, 3)
            report["terminal_truth_xy"] = [node.truth.position.x, node.truth.position.y]
            report["terminal_truth_error_m"] = math.hypot(
                node.truth.position.x - target.pose.position.x, node.truth.position.y - target.pose.position.y)
            if status != 4 or displacement < args.distance - .7:
                raise RuntimeError("Simulated vehicle did not finish the requested motion")
        if int(node.fusion_diagnostics.get("fixed_rtk_factors", "0")) < 2:
            raise RuntimeError("No accepted simulated RTK constraints")
        report["status"] = "PASS"
    except Exception as error:
        report["status"] = "FAIL"
        report["error"] = str(error)
    finally:
        report["wall_seconds"] = time.monotonic() - started
        report["topic_counts"] = node.counts
        report["fusion"] = node.fusion_diagnostics
        if node.position_errors:
            report["horizontal_rmse_m"] = float(np.sqrt(np.mean(np.square(node.position_errors))))
            report["horizontal_max_error_m"] = max(node.position_errors)
        if len(node.clock_samples) > 1:
            a, b = node.clock_samples[0], node.clock_samples[-1]
            report["real_time_factor"] = (b[1] - a[1]) / (b[0] - a[0])
        (args.output / "trace.json").write_text(json.dumps(node.trace))
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)
    if report["status"] != "PASS":
        raise SystemExit(1)
