#!/usr/bin/env python3

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Path as PathMessage
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool
import yaml


REQUIRED_INPUT_TOPICS = (
    "/lidar/points",
    "/imu/data",
    "/camera/rgb/image_raw",
)
PLAYBACK_TOPICS = REQUIRED_INPUT_TOPICS + (
    "/rtk/fix",
    "/rtk/fix_quality",
)
OUTPUT_TOPICS = (
    "/fastlivo/odometry",
    "/fastlivo_rtk/odometry",
    "/fastlivo_rtk/path",
    "/fastlivo_rtk/fastlivo_path",
    "/localization/status",
    "/localization_pose",
    "/localization/lidar_ready",
    "/fastlivo_rtk/ready",
    "/fastlivo_rtk/fixed_active",
    "/fastlivo_rtk/fixed_rtk_path",
    "/diagnostics",
)


class ValidationError(RuntimeError):
    pass


def command_text(command):
    return " ".join(str(value) for value in command)


def start(command, environment):
    print("\n$ " + command_text(command), flush=True)
    return subprocess.Popen(
        [str(value) for value in command],
        env=environment,
        start_new_session=True,
    )


def stop(process, name, timeout=20.0):
    if process is None or process.poll() is not None:
        return
    print(f"Stopping {name} ...", flush=True)
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=5.0)


def topic_counts(bag):
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in information["topics_with_message_count"]
    }


def database_paths(bag):
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    paths = [bag / entry for entry in information.get("relative_file_paths", [])]
    return paths or sorted(bag.glob("*.db3"))


def topic_messages(bag, topic_name, message_type):
    messages = []
    for database in database_paths(bag):
        connection = sqlite3.connect(database)
        topic = connection.execute(
            "SELECT id FROM topics WHERE name = ?", (topic_name,)
        ).fetchone()
        if topic is not None:
            rows = connection.execute(
                "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic[0],),
            )
            messages.extend(
                deserialize_message(serialized, message_type)
                for (serialized,) in rows
            )
        connection.close()
    return messages


def latest_topic_message(bag, topic_name, message_type):
    latest = None
    latest_timestamp = -1
    for database in database_paths(bag):
        connection = sqlite3.connect(database)
        topic = connection.execute(
            "SELECT id FROM topics WHERE name = ?", (topic_name,)
        ).fetchone()
        if topic is not None:
            row = connection.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp DESC LIMIT 1",
                (topic[0],),
            ).fetchone()
            if row is not None and row[0] > latest_timestamp:
                latest_timestamp = row[0]
                latest = deserialize_message(row[1], message_type)
        connection.close()
    return latest


def compare_paths(fused_path, local_path):
    if fused_path is None or local_path is None:
        raise ValidationError("FAST-LIVO2 comparison paths are absent")
    if len(fused_path.poses) != len(local_path.poses):
        raise ValidationError(
            "fused and local path lengths differ without RTK: "
            f"{len(fused_path.poses)} != {len(local_path.poses)}"
        )
    if not fused_path.poses:
        raise ValidationError("FAST-LIVO2 comparison paths contain no poses")

    maximum_position_delta = 0.0
    maximum_orientation_delta = 0.0
    for fused, local in zip(fused_path.poses, local_path.poses):
        position_delta = math.sqrt(
            (fused.pose.position.x - local.pose.position.x) ** 2
            + (fused.pose.position.y - local.pose.position.y) ** 2
            + (fused.pose.position.z - local.pose.position.z) ** 2
        )
        maximum_position_delta = max(maximum_position_delta, position_delta)
        fused_q = fused.pose.orientation
        local_q = local.pose.orientation
        quaternion_dot = abs(
            fused_q.x * local_q.x
            + fused_q.y * local_q.y
            + fused_q.z * local_q.z
            + fused_q.w * local_q.w
        )
        orientation_delta = 2.0 * math.acos(min(1.0, quaternion_dot))
        maximum_orientation_delta = max(
            maximum_orientation_delta, orientation_delta
        )

    return maximum_position_delta, maximum_orientation_delta


def wait_for_subscription(topic, environment, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            env=environment,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if line.startswith("Subscription count:"):
                    if int(line.split(":", 1)[1]) > 0:
                        return
        time.sleep(0.25)
    raise ValidationError(f"no subscriber became ready for {topic}")


def wait_for_message(topic, environment, timeout):
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            env=environment,
            check=False,
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        raise ValidationError(f"timed out waiting for {topic}") from error
    if result.returncode != 0:
        raise ValidationError(f"failed while waiting for {topic}")


def publish_manual_prior(arguments, environment):
    yaw = math.radians(arguments.initial_yaw_deg)
    covariance = [0.0] * 36
    covariance[0] = arguments.initial_position_std_m**2
    covariance[7] = arguments.initial_position_std_m**2
    covariance[35] = math.radians(arguments.initial_yaw_std_deg) ** 2
    message = {
        "header": {"frame_id": "map"},
        "pose": {
            "pose": {
                "position": {
                    "x": arguments.initial_x,
                    "y": arguments.initial_y,
                    "z": 0.0,
                },
                "orientation": {
                    "z": math.sin(yaw / 2.0),
                    "w": math.cos(yaw / 2.0),
                },
            },
            "covariance": covariance,
        },
    }
    command = [
        "ros2", "topic", "pub", "--once", "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped", json.dumps(message),
    ]
    print("\n$ " + command_text(command), flush=True)
    result = subprocess.run(command, env=environment, check=False)
    if result.returncode != 0:
        raise ValidationError("failed to publish the manual initial pose")


def validate_result(output_bag):
    counts = topic_counts(output_bag)
    required = (
        "/fastlivo/odometry",
        "/fastlivo_rtk/odometry",
        "/fastlivo_rtk/path",
        "/fastlivo_rtk/fastlivo_path",
        "/localization_pose",
        "/fastlivo_rtk/ready",
        "/fastlivo_rtk/fixed_active",
        "/diagnostics",
    )
    missing = [topic for topic in required if counts.get(topic, 0) < 1]
    if missing:
        raise ValidationError(
            "validation output is missing topics:\n  " + "\n  ".join(missing)
        )

    ready = topic_messages(output_bag, "/fastlivo_rtk/ready", Bool)
    fixed_active = topic_messages(
        output_bag, "/fastlivo_rtk/fixed_active", Bool
    )
    diagnostics = topic_messages(output_bag, "/diagnostics", DiagnosticArray)
    fusion_statuses = [
        status
        for array in diagnostics
        for status in array.status
        if status.name == "agribot/fastlivo_rtk_fusion"
    ]
    if not any(message.data for message in ready):
        raise ValidationError("fusion never became ready after NDT/GICP")
    if any(message.data for message in fixed_active):
        raise ValidationError("fixed RTK unexpectedly became active")
    if not fusion_statuses:
        raise ValidationError("fusion diagnostics are absent")
    latest_values = {
        item.key: item.value for item in fusion_statuses[-1].values
    }
    if latest_values.get("initialized") != "true":
        raise ValidationError("fusion diagnostics report initialized=false")
    if int(latest_values.get("fixed_rtk_factors", "-1")) != 0:
        raise ValidationError("an RTK factor was added during the no-RTK test")
    if latest_values.get("global_correction_frozen") != "true":
        raise ValidationError("global correction was not frozen without RTK")

    fused_path = latest_topic_message(
        output_bag, "/fastlivo_rtk/path", PathMessage
    )
    local_path = latest_topic_message(
        output_bag, "/fastlivo_rtk/fastlivo_path", PathMessage
    )
    position_delta, orientation_delta = compare_paths(fused_path, local_path)
    if position_delta > 1.0e-6 or orientation_delta > 1.0e-6:
        raise ValidationError(
            "fusion altered FAST-LIVO2 propagation without fixed RTK: "
            f"position={position_delta:.9f} m, "
            f"orientation={math.degrees(orientation_delta):.9f} deg"
        )

    return {
        "fastlivo_poses": counts["/fastlivo/odometry"],
        "fused_poses": counts["/fastlivo_rtk/odometry"],
        "localized_poses": counts["/localization_pose"],
        "fixed_rtk_path_poses": counts.get("/fastlivo_rtk/fixed_rtk_path", 0),
        "diagnostic_message": fusion_statuses[-1].message,
        "fixed_rtk_factors": int(latest_values["fixed_rtk_factors"]),
        "gravity_factors": int(latest_values.get("gravity_factors", "0")),
        "comparison_path_poses": len(fused_path.poses),
        "maximum_no_rtk_position_delta_m": position_delta,
        "maximum_no_rtk_orientation_delta_deg": math.degrees(
            orientation_delta
        ),
    }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Replay an indoor lidar/IMU/camera bag through the production "
            "manual NDT/GICP and FAST-LIVO2/RTK fusion chain with no RTK fixes"
        )
    )
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--domain-id", type=int, default=76)
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--initial-position-std-m", type=float, default=1.0)
    parser.add_argument("--initial-yaw-std-deg", type=float, default=20.0)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def validate_inputs(arguments):
    source_bag = arguments.source_bag.expanduser().resolve()
    map_base = arguments.map_base.expanduser().resolve()
    output_bag = arguments.output_bag.expanduser().resolve()
    if not (source_bag / "metadata.yaml").is_file():
        raise ValidationError(f"invalid source bag: {source_bag}")
    if map_base.suffix:
        raise ValidationError("map_base must not contain an extension")
    for suffix in (".pcd", ".yaml"):
        path = map_base.with_suffix(suffix)
        if not path.is_file():
            raise ValidationError(f"map artifact not found: {path}")
    counts = topic_counts(source_bag)
    missing = [topic for topic in REQUIRED_INPUT_TOPICS if counts.get(topic, 0) < 1]
    if missing:
        raise ValidationError(
            "source bag is missing indoor sensor topics:\n  " + "\n  ".join(missing)
        )
    if counts.get("/rtk/fix", 0) != 0:
        raise ValidationError(
            "source bag contains RTK fixes; this command is only for no-RTK validation"
        )
    if output_bag.exists():
        if not arguments.force:
            raise ValidationError(
                f"output bag already exists; use --force: {output_bag}"
            )
        shutil.rmtree(output_bag)
    if source_bag == output_bag:
        raise ValidationError("source and output bag must differ")
    if arguments.playback_rate <= 0.0 or arguments.settle_seconds < 0.0:
        raise ValidationError("playback and settle durations are invalid")
    if not 0 <= arguments.domain_id <= 232:
        raise ValidationError("domain_id must be in [0, 232]")
    if arguments.initial_position_std_m <= 0.0 or arguments.initial_yaw_std_deg <= 0.0:
        raise ValidationError("initial-pose standard deviations must be positive")
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    return source_bag, map_base, output_bag


def main(argv=None):
    arguments = parse_arguments(argv)
    launch_process = player = recorder = None
    try:
        source_bag, map_base, output_bag = validate_inputs(arguments)
        environment = os.environ.copy()
        environment.update(
            {
                "ROS_DOMAIN_ID": str(arguments.domain_id),
                "ROS_LOCALHOST_ONLY": "1",
                "ROS2CLI_NO_DAEMON": "1",
            }
        )
        launch_process = start(
            [
                "ros2", "launch", "agribot_hardware_bringup",
                "ackermann_fastlivo_rtk_localization.launch.py",
                f"map_base:={map_base}", "use_sim_time:=true",
                "start_sensors:=false", "start_camera:=false", "rviz:=" + (
                    "true" if arguments.rviz else "false"
                ),
                "initialization_source:=manual",
                "allow_missing_georeference:=true",
            ],
            environment,
        )
        wait_for_subscription("/initialpose", environment, 45.0)

        recorder = start(
            ["ros2", "bag", "record", "-o", output_bag, *OUTPUT_TOPICS],
            environment,
        )
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise ValidationError("result recorder exited during startup")
        player = start(
            [
                "ros2", "bag", "play", source_bag, "--clock", "100",
                "--rate", str(arguments.playback_rate),
                "--read-ahead-queue-size", "1000", "--topics", *PLAYBACK_TOPICS,
                "--disable-keyboard-controls",
            ],
            environment,
        )
        wait_for_message("/fastlivo/odometry", environment, 90.0)
        publish_manual_prior(arguments, environment)
        wait_for_message("/localization_pose", environment, 120.0)
        if player.wait() != 0:
            raise ValidationError("source bag playback failed")
        player = None
        time.sleep(arguments.settle_seconds)
        stop(recorder, "indoor validation recorder")
        recorder = None

        result = validate_result(output_bag)
        print("\nIndoor no-RTK validation passed:", flush=True)
        print(f"  output_bag: {output_bag}", flush=True)
        for key, value in result.items():
            print(f"  {key}: {value}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        return 130
    except (ValidationError, subprocess.TimeoutExpired) as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        return 1
    finally:
        stop(player, "source bag playback")
        stop(recorder, "indoor validation recorder")
        stop(launch_process, "FAST-LIVO2 indoor validation launch")


if __name__ == "__main__":
    raise SystemExit(main())
