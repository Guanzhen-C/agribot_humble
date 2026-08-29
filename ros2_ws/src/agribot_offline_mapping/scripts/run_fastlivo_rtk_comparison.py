#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool
import yaml


INPUT_TOPICS = (
    "/lidar/points",
    "/imu/data",
    "/camera/rgb/image_raw",
    "/rtk/fix",
    "/rtk/fix_quality",
    "/rtk/heading_with_covariance",
    "/rtk/heading_solution",
)
OUTPUT_TOPICS = (
    "/fastlivo/odometry",
    "/fastlivo_rtk/odometry",
    "/fastlivo_rtk/path",
    "/fastlivo_rtk/fastlivo_path",
    "/fastlivo_rtk/fixed_rtk_path",
    "/localization_pose",
    "/localization/status",
    "/localization/lidar_ready",
    "/fastlivo_rtk/ready",
    "/fastlivo_rtk/fixed_active",
    "/diagnostics",
)
VEHICLE_LAUNCHES = {
    "ackermann": "ackermann_fastlivo_rtk_localization.launch.py",
    "differential": "differential_fastlivo_rtk_localization.launch.py",
}
VISUAL_MAP_SERVICE = "/fastlivo_rtk_visual_mapper/save"


class FusionError(RuntimeError):
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


def stop(process, name, timeout=30.0):
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


def wait_for_service(service_name, environment, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "service", "type", service_name],
            env=environment,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode == 0 and result.stdout.strip():
            return
        time.sleep(0.25)
    raise FusionError(f"service did not become ready: {service_name}")


def save_visual_map(environment):
    result = subprocess.run(
        [
            "ros2", "service", "call", VISUAL_MAP_SERVICE,
            "std_srvs/srv/Trigger", "{}",
        ],
        env=environment,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=120.0,
    )
    if result.stdout:
        print(result.stdout.rstrip(), flush=True)
    normalized = " ".join((result.stdout or "").lower().split())
    if result.returncode != 0 or not (
        "success=true" in normalized or "success: true" in normalized
    ):
        raise FusionError("FAST-LIVO2+RTK visual map save service failed")


def pcd_point_count(path):
    with path.open("rb") as stream:
        for _ in range(64):
            line = stream.readline().decode("ascii", errors="strict").strip()
            if line.startswith("POINTS "):
                return int(line.split()[1])
            if line.startswith("DATA "):
                break
    raise FusionError(f"visual PCD header has no POINTS field: {path}")


def bag_information(path):
    return yaml.safe_load(
        (path / "metadata.yaml").read_text(encoding="utf-8")
    )["rosbag2_bagfile_information"]


def topic_counts(path):
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in bag_information(path)["topics_with_message_count"]
    }


def database_paths(path):
    paths = [
        path / name for name in bag_information(path).get("relative_file_paths", [])
    ]
    return paths or sorted(path.glob("*.db3"))


def topic_messages(path, topic_name, message_type):
    messages = []
    for database in database_paths(path):
        connection = sqlite3.connect(database)
        topic = connection.execute(
            "SELECT id FROM topics WHERE name=?", (topic_name,)
        ).fetchone()
        if topic is not None:
            rows = connection.execute(
                "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic[0],),
            )
            messages.extend(
                deserialize_message(data, message_type) for (data,) in rows
            )
        connection.close()
    return messages


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Replay raw sensors through production NDT/GICP initialization and "
            "FAST-LIVO2+RTK fixed-lag fusion"
        )
    )
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument(
        "--vehicle-profile",
        choices=tuple(VEHICLE_LAUNCHES),
        default="ackermann",
    )
    parser.add_argument("--domain-id", type=int, default=75)
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=8.0)
    parser.add_argument(
        "--visual-map",
        type=Path,
        help=(
            "optional colored PCD built from dense FAST-LIVO2 clouds after "
            "the time-varying RTK fusion correction"
        ),
    )
    parser.add_argument("--visual-voxel-size", type=float, default=0.10)
    parser.add_argument(
        "--visual-sync-tolerance-sec", type=float, default=0.12
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def validate_inputs(arguments):
    source_bag = arguments.source_bag.expanduser().resolve()
    map_base = arguments.map_base.expanduser().resolve()
    output_bag = arguments.output_bag.expanduser().resolve()
    if not (source_bag / "metadata.yaml").is_file():
        raise FusionError(f"invalid source bag: {source_bag}")
    if map_base.suffix:
        raise FusionError("map_base must not have an extension")
    required_maps = (
        map_base.with_suffix(".pcd"),
        map_base.with_suffix(".yaml"),
        map_base.parent / f"{map_base.name}_georeference.yaml",
    )
    missing_maps = [path for path in required_maps if not path.is_file()]
    if missing_maps:
        raise FusionError(
            "mapping artifacts are incomplete:\n  "
            + "\n  ".join(str(path) for path in missing_maps)
        )
    counts = topic_counts(source_bag)
    missing_topics = [topic for topic in INPUT_TOPICS if counts.get(topic, 0) < 1]
    if missing_topics:
        raise FusionError(
            "source bag is missing required topics:\n  "
            + "\n  ".join(missing_topics)
        )
    if output_bag.exists():
        if not arguments.force:
            raise FusionError(f"output bag already exists: {output_bag}")
        shutil.rmtree(output_bag)
    if source_bag == output_bag:
        raise FusionError("source and output bags must differ")
    if arguments.playback_rate <= 0.0 or arguments.settle_seconds < 0.0:
        raise FusionError("playback and settle durations are invalid")
    if arguments.visual_voxel_size <= 0.0:
        raise FusionError("visual voxel size must be positive")
    if arguments.visual_sync_tolerance_sec <= 0.0:
        raise FusionError("visual synchronization tolerance must be positive")
    if not 0 <= arguments.domain_id <= 232:
        raise FusionError("domain_id must be in [0, 232]")
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    visual_map = None
    if arguments.visual_map is not None:
        visual_map = arguments.visual_map.expanduser().resolve()
        if visual_map.suffix.lower() != ".pcd":
            raise FusionError("visual map output must use the .pcd extension")
        if visual_map == map_base.with_suffix(".pcd"):
            raise FusionError("visual map must not overwrite the localization PCD")
        if visual_map.exists():
            if not arguments.force:
                raise FusionError(f"visual map already exists: {visual_map}")
            visual_map.unlink()
        visual_map.parent.mkdir(parents=True, exist_ok=True)
    return source_bag, map_base, output_bag, visual_map


def validate_output(output_bag):
    counts = topic_counts(output_bag)
    required = (
        "/fastlivo/odometry",
        "/fastlivo_rtk/odometry",
        "/localization_pose",
        "/fastlivo_rtk/ready",
        "/fastlivo_rtk/fixed_active",
        "/diagnostics",
    )
    missing = [topic for topic in required if counts.get(topic, 0) < 1]
    if missing:
        raise FusionError(
            "fusion output is missing topics:\n  " + "\n  ".join(missing)
        )
    ready = topic_messages(output_bag, "/fastlivo_rtk/ready", Bool)
    fixed = topic_messages(output_bag, "/fastlivo_rtk/fixed_active", Bool)
    if not any(message.data for message in ready):
        raise FusionError("NDT/GICP never initialized FAST-LIVO2+RTK fusion")
    if not any(message.data for message in fixed):
        raise FusionError("fixed RTK never became active in fusion")
    statuses = [
        status
        for message in topic_messages(output_bag, "/diagnostics", DiagnosticArray)
        for status in message.status
        if status.name == "agribot/fastlivo_rtk_fusion"
    ]
    if not statuses:
        raise FusionError("FAST-LIVO2+RTK diagnostics are absent")
    values = {item.key: item.value for item in statuses[-1].values}
    fixed_factors = int(values.get("fixed_rtk_factors", "0"))
    if fixed_factors < 1:
        raise FusionError("fusion did not insert any fixed RTK factor")
    return counts, fixed_factors, values.get("gravity_factors", "0")


def main(argv=None):
    arguments = parse_arguments(argv)
    launch_process = recorder = player = visual_mapper = None
    try:
        source_bag, map_base, output_bag, visual_map = validate_inputs(arguments)
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
                "ros2",
                "launch",
                "agribot_hardware_bringup",
                VEHICLE_LAUNCHES[arguments.vehicle_profile],
                f"map_base:={map_base}",
                "use_sim_time:=true",
                "start_sensors:=false",
                "start_rtk:=false",
                "start_camera:=false",
                "rviz:=false",
                "initialization_source:=rtk",
                "enable_fpfh:=false",
                "allow_missing_georeference:=false",
                "allow_uncalibrated_camera:=true",
                f"fastlivo_dense_map:={'true' if visual_map else 'false'}",
            ],
            environment,
        )
        time.sleep(5.0)
        if launch_process.poll() is not None:
            raise FusionError("localization launch exited during startup")

        if visual_map is not None:
            visual_mapper = start(
                [
                    "ros2", "run", "agribot_offline_mapping",
                    "fastlivo_rtk_visual_mapper", "--ros-args",
                    "-p", "use_sim_time:=true",
                    "-p", f"output_file:={visual_map}",
                    "-p", f"voxel_size:={arguments.visual_voxel_size}",
                    "-p",
                    "sync_tolerance_sec:="
                    f"{arguments.visual_sync_tolerance_sec}",
                    "-p", f"allow_overwrite:={str(arguments.force).lower()}",
                ],
                environment,
            )
            wait_for_service(VISUAL_MAP_SERVICE, environment)
            if visual_mapper.poll() is not None:
                raise FusionError("visual mapper exited during startup")

        recorder = start(
            ["ros2", "bag", "record", "-o", output_bag, *OUTPUT_TOPICS],
            environment,
        )
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise FusionError("fusion recorder exited during startup")

        player = start(
            [
                "ros2",
                "bag",
                "play",
                source_bag,
                "--clock",
                "100",
                "--rate",
                str(arguments.playback_rate),
                "--read-ahead-queue-size",
                "1000",
                "--topics",
                *INPUT_TOPICS,
                "--disable-keyboard-controls",
            ],
            environment,
        )
        if player.wait() != 0:
            raise FusionError("source bag playback failed")
        player = None
        time.sleep(arguments.settle_seconds)
        visual_points = None
        if visual_map is not None:
            save_visual_map(environment)
            if not visual_map.is_file():
                raise FusionError(f"visual map was not created: {visual_map}")
            visual_points = pcd_point_count(visual_map)
            if visual_points < 1000:
                raise FusionError(
                    f"visual map contains too few colored points: {visual_points}"
                )
        stop(recorder, "FAST-LIVO2+RTK recorder")
        recorder = None

        counts, fixed_factors, gravity_factors = validate_output(output_bag)
        print("\nFAST-LIVO2+RTK recomputation completed:", flush=True)
        print(f"  output_bag: {output_bag}", flush=True)
        print(f"  FAST-LIVO2 poses: {counts['/fastlivo/odometry']}", flush=True)
        print(f"  fused poses: {counts['/fastlivo_rtk/odometry']}", flush=True)
        print(f"  fixed RTK factors: {fixed_factors}", flush=True)
        print(f"  gravity factors: {gravity_factors}", flush=True)
        if visual_map is not None:
            print(f"  visual map: {visual_map}", flush=True)
            print(f"  colored voxels: {visual_points}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        return 130
    except (FusionError, subprocess.TimeoutExpired) as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        return 1
    finally:
        stop(player, "source bag playback")
        stop(recorder, "FAST-LIVO2+RTK recorder")
        stop(visual_mapper, "FAST-LIVO2+RTK visual mapper")
        stop(launch_process, "FAST-LIVO2+RTK localization launch")


if __name__ == "__main__":
    raise SystemExit(main())
