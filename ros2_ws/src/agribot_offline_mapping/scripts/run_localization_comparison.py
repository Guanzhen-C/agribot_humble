#!/usr/bin/env python3

import argparse
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import yaml


RAW_TOPICS = (
    "/lidar/points",
    "/imu/data",
    "/camera/rgb/image_raw",
    "/rtk/fix",
    "/rtk/fix_quality",
    "/rtk/heading_with_covariance",
    "/rtk/heading_solution",
)
FASTLIO_TOPIC = "/comparison/fastlio/odometry"
FASTLIVO_TOPIC = "/comparison/fastlivo/odometry"
KF_GINS_TOPIC = "/comparison/kf_gins/odometry"
COMPARISON_SUFFIX = "_comparison"
GEOREFERENCE_SUFFIX = "_georeference.yaml"


class ComparisonError(RuntimeError):
    pass


def start(command, environment):
    print("\n$ " + " ".join(str(value) for value in command), flush=True)
    return subprocess.Popen(
        [str(value) for value in command],
        env=environment,
        start_new_session=True,
    )


def run(command, environment):
    print("\n$ " + " ".join(str(value) for value in command), flush=True)
    result = subprocess.run(
        [str(value) for value in command], env=environment, check=False
    )
    if result.returncode != 0:
        raise ComparisonError(
            f"command failed with exit code {result.returncode}: "
            + " ".join(str(value) for value in command)
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
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
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


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Recompute FAST-LIO2, FAST-LIVO2 and KF-GINS from raw sensor "
            "topics in one ROS bag"
        )
    )
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--domain-id", type=int, default=74)
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument(
        "--georeference",
        type=Path,
        help=(
            "map georeference YAML used to fix the KF-GINS ENU origin; "
            "inferred for an output bag ending in _comparison"
        ),
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def validate(arguments):
    source_bag = arguments.source_bag.expanduser().resolve()
    output_bag = arguments.output_bag.expanduser().resolve()
    if not (source_bag / "metadata.yaml").is_file():
        raise ComparisonError(f"invalid source ROS bag: {source_bag}")
    if source_bag == output_bag:
        raise ComparisonError("source_bag and output_bag must differ")
    if output_bag.exists():
        if not arguments.force:
            raise ComparisonError(
                f"output bag already exists; use --force to replace it: {output_bag}"
            )
        shutil.rmtree(output_bag)
    if arguments.playback_rate <= 0.0:
        raise ComparisonError("playback_rate must be positive")
    if arguments.settle_seconds < 0.0:
        raise ComparisonError("settle_seconds must not be negative")
    if not 0 <= arguments.domain_id <= 232:
        raise ComparisonError("domain_id must be in [0, 232]")
    source_counts = topic_counts(source_bag)
    missing = [topic for topic in RAW_TOPICS if source_counts.get(topic, 0) < 1]
    if missing:
        raise ComparisonError(
            "source bag is missing required raw topics:\n  " + "\n  ".join(missing)
        )
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    return source_bag, output_bag


def resolve_kf_reference(arguments):
    output_bag = arguments.output_bag.expanduser().resolve()
    georeference = arguments.georeference
    if georeference is None:
        if not output_bag.name.endswith(COMPARISON_SUFFIX):
            return None
        map_name = output_bag.name[: -len(COMPARISON_SUFFIX)]
        georeference = output_bag.parent / f"{map_name}{GEOREFERENCE_SUFFIX}"
    else:
        georeference = georeference.expanduser().resolve()

    if not georeference.is_file():
        raise ComparisonError(f"georeference file not found: {georeference}")
    try:
        document = yaml.safe_load(georeference.read_text(encoding="utf-8"))
        reference = document["reference"]
        values = (
            float(reference["latitude_deg"]),
            float(reference["longitude_deg"]),
            float(reference["altitude_m"]),
        )
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ComparisonError(
            f"invalid georeference reference in {georeference}: {error}"
        ) from error
    if not all(math.isfinite(value) for value in values):
        raise ComparisonError(
            f"georeference reference contains non-finite values: {georeference}"
        )
    latitude, longitude, altitude = values
    if abs(latitude) > 90.0 or abs(longitude) > 180.0:
        raise ComparisonError(
            f"georeference reference is outside valid latitude/longitude bounds: "
            f"{georeference}"
        )
    return georeference, latitude, longitude, altitude


def main(argv=None):
    arguments = parse_arguments(argv)
    processes = []
    recorder = None
    try:
        kf_reference = resolve_kf_reference(arguments)
        source_bag, output_bag = validate(arguments)
        hardware_share = Path(
            get_package_share_directory("agribot_hardware_bringup")
        )
        fastlio_config = hardware_share / "config" / "fast_lio_c16.yaml"
        bridge_config = hardware_share / "config" / "fastlio_bridge.yaml"
        kf_gins_config = hardware_share / "config" / "kf_gins_n300pro.yaml"
        fastlivo_share = Path(get_package_share_directory("fast_livo"))
        fastlivo_config = (
            fastlivo_share / "config" / "agribot_c16_astra.yaml"
        )
        fastlivo_camera_config = (
            fastlivo_share / "config" / "agribot_astra_640.yaml"
        )
        for path in (
            fastlio_config,
            bridge_config,
            kf_gins_config,
            fastlivo_config,
            fastlivo_camera_config,
        ):
            if not path.is_file():
                raise ComparisonError(f"configuration file not found: {path}")

        environment = os.environ.copy()
        environment.update(
            {
                "ROS_DOMAIN_ID": str(arguments.domain_id),
                "ROS_LOCALHOST_ONLY": "1",
                "ROS2CLI_NO_DAEMON": "1",
            }
        )
        processes.append((start(
            [
                "ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
                "--params-file", fastlio_config, "-p", "use_sim_time:=true",
            ],
            environment,
        ), "FAST-LIO2"))
        processes.append((start(
            [
                "ros2", "run", "agribot_hardware_bringup",
                "fastlio_odom_bridge.py", "--ros-args", "--params-file",
                bridge_config, "-p", "use_sim_time:=true", "-p",
                f"output_odom_topic:={FASTLIO_TOPIC}", "-p", "publish_tf:=false",
            ],
            environment,
        ), "FAST-LIO2 odometry bridge"))
        processes.append((start(
            [
                "ros2", "run", "fast_livo", "fastlivo_mapping", "--ros-args",
                "-r", "__node:=fastlivo_comparison",
                "--params-file", fastlivo_config,
                "--params-file", fastlivo_camera_config,
                "-p", "use_sim_time:=true",
                "-r", f"/aft_mapped_to_init:={FASTLIVO_TOPIC}",
                "-r", "/path:=/comparison/fastlivo/path",
                "-r",
                "/cloud_registered:=/comparison/fastlivo/cloud_registered",
                "-r", "/rgb_img:=/comparison/fastlivo/rgb_img",
            ],
            environment,
        ), "FAST-LIVO2"))
        kf_gins_command = [
            "ros2", "run", "agribot_hardware_bringup",
            "rtk_eskf_localization", "--ros-args", "--params-file",
            kf_gins_config, "-p", "use_sim_time:=true", "-p",
            f"output_odom_topic:={KF_GINS_TOPIC}", "-p",
            "raw_pose_topic:=/comparison/kf_gins/raw_pose",
        ]
        if kf_reference is not None:
            georeference, latitude, longitude, altitude = kf_reference
            print(
                "KF-GINS ENU reference fixed from "
                f"{georeference}: {latitude:.10f}, {longitude:.10f}, "
                f"{altitude:.3f} m",
                flush=True,
            )
            kf_gins_command.extend(
                [
                    "-p", "auto_reference_from_first_navsat_fix:=false",
                    "-p", f"reference_lat_deg:={latitude:.12f}",
                    "-p", f"reference_lon_deg:={longitude:.12f}",
                    "-p", f"reference_alt_m:={altitude:.6f}",
                ]
            )
        processes.append((start(kf_gins_command, environment), "KF-GINS"))
        time.sleep(3.0)
        failed = [name for process, name in processes if process.poll() is not None]
        if failed:
            raise ComparisonError("process exited during startup: " + ", ".join(failed))

        recorder = start(
            [
                "ros2", "bag", "record", "-o", output_bag,
                FASTLIO_TOPIC,
                FASTLIVO_TOPIC,
                KF_GINS_TOPIC,
            ],
            environment,
        )
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise ComparisonError("comparison bag recorder exited before playback")

        run(
            [
                "ros2", "bag", "play", source_bag, "--clock", "100",
                "--rate", str(arguments.playback_rate),
                "--read-ahead-queue-size", "1000", "--topics", *RAW_TOPICS,
            ],
            environment,
        )
        time.sleep(arguments.settle_seconds)
        stop(recorder, "comparison bag recorder")
        recorder = None

        counts = topic_counts(output_bag)
        missing = [
            topic for topic in (
                FASTLIO_TOPIC, FASTLIVO_TOPIC, KF_GINS_TOPIC,
            )
            if counts.get(topic, 0) < 1
        ]
        if missing:
            raise ComparisonError(
                "recomputed output is missing topics:\n  " + "\n  ".join(missing)
            )
        print("\nLocalization recomputation completed:", flush=True)
        print(f"  output_bag: {output_bag}", flush=True)
        print(f"  FAST-LIO2 poses: {counts[FASTLIO_TOPIC]}", flush=True)
        print(f"  FAST-LIVO2 poses: {counts[FASTLIVO_TOPIC]}", flush=True)
        print(f"  KF-GINS poses: {counts[KF_GINS_TOPIC]}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        return 130
    except ComparisonError as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        return 1
    finally:
        stop(recorder, "comparison bag recorder")
        for process, name in reversed(processes):
            stop(process, name)


if __name__ == "__main__":
    raise SystemExit(main())
