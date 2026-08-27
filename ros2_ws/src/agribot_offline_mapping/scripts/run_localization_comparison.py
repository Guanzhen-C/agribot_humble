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


FASTLIO_TOPIC = "/comparison/fastlio/odometry"
FASTLIVO_TOPIC = "/comparison/fastlivo/odometry"
KF_GINS_TOPIC = "/comparison/kf_gins/odometry"
ESTIMATOR_TOPICS = {
    "fastlio": FASTLIO_TOPIC,
    "fastlivo": FASTLIVO_TOPIC,
    "kf_gins": KF_GINS_TOPIC,
}
ESTIMATOR_INPUT_TOPICS = {
    "fastlio": ("/lidar/points", "/imu/data"),
    "fastlivo": (
        "/lidar/points",
        "/imu/data",
        "/camera/rgb/image_raw",
    ),
    "kf_gins": (
        "/imu/data",
        "/rtk/fix",
        "/rtk/fix_quality",
        "/rtk/heading_with_covariance",
        "/rtk/heading_solution",
    ),
}
COMPARISON_SUFFIX = "_comparison"
GEOREFERENCE_SUFFIX = "_georeference.yaml"
VEHICLE_PROFILES = ("ackermann", "differential")


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
            "Recompute exactly one localization trajectory from raw sensor "
            "topics. Run this command separately for each estimator."
        )
    )
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--domain-id", type=int, default=74)
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument(
        "--estimator",
        choices=tuple(ESTIMATOR_TOPICS),
        required=True,
        help="the only estimator started during this replay",
    )
    parser.add_argument(
        "--vehicle-profile",
        choices=VEHICLE_PROFILES,
        default="ackermann",
        help="physical sensor geometry used by all recomputed estimators",
    )
    parser.add_argument(
        "--fastlivo-profile",
        choices=("indoor", "outdoor"),
        default="indoor",
        help=(
            "FAST-LIVO2 environment profile; outdoor loads the validated "
            "sparse-scene overrides after the shared sensor calibration"
        ),
    )
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
    required_topics = ESTIMATOR_INPUT_TOPICS[arguments.estimator]
    missing = [
        topic for topic in required_topics if source_counts.get(topic, 0) < 1
    ]
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


def resolve_estimator_configs(arguments, hardware_share, fastlivo_share):
    common_config = hardware_share / "config"
    differential_config = hardware_share / "differential" / "config"
    if arguments.vehicle_profile == "differential":
        fastlio_config = differential_config / "fast_lio_c16.yaml"
        bridge_config = differential_config / "fastlio_bridge.yaml"
        kf_gins_configs = [
            common_config / "kf_gins_n300pro.yaml",
            differential_config / "kf_gins_n300pro.yaml",
        ]
        vehicle_fastlivo_config = (
            differential_config / "fastlivo_sensor_calibration.yaml"
        )
    else:
        fastlio_config = common_config / "fast_lio_c16.yaml"
        bridge_config = common_config / "fastlio_bridge.yaml"
        kf_gins_configs = [common_config / "kf_gins_n300pro.yaml"]
        vehicle_fastlivo_config = None

    fastlivo_parameter_files = [
        fastlivo_share / "config" / "agribot_c16_astra.yaml"
    ]
    if arguments.fastlivo_profile == "outdoor":
        fastlivo_parameter_files.append(
            fastlivo_share / "config" / "agribot_c16_astra_outdoor.yaml"
        )
    if vehicle_fastlivo_config is not None:
        fastlivo_parameter_files.append(vehicle_fastlivo_config)
    fastlivo_parameter_files.append(
        common_config / "fastlivo_hikrobot_mv_cu013.yaml"
    )

    paths = [
        fastlio_config,
        bridge_config,
        *kf_gins_configs,
        *fastlivo_parameter_files,
    ]
    missing = [path for path in paths if not path.is_file()]
    if missing:
        raise ComparisonError(
            "configuration files not found:\n  "
            + "\n  ".join(str(path) for path in missing)
        )
    return {
        "fastlio": fastlio_config,
        "bridge": bridge_config,
        "kf_gins": kf_gins_configs,
        "fastlivo": fastlivo_parameter_files,
    }


def main(argv=None):
    arguments = parse_arguments(argv)
    processes = []
    recorder = None
    try:
        kf_reference = (
            resolve_kf_reference(arguments)
            if arguments.estimator == "kf_gins"
            else None
        )
        source_bag, output_bag = validate(arguments)
        hardware_share = Path(
            get_package_share_directory("agribot_hardware_bringup")
        )
        fastlivo_share = Path(get_package_share_directory("fast_livo"))
        configs = resolve_estimator_configs(
            arguments, hardware_share, fastlivo_share
        )
        fastlio_config = configs["fastlio"]
        bridge_config = configs["bridge"]
        kf_gins_configs = configs["kf_gins"]
        fastlivo_parameter_files = configs["fastlivo"]

        environment = os.environ.copy()
        environment.update(
            {
                "ROS_DOMAIN_ID": str(arguments.domain_id),
                "ROS_LOCALHOST_ONLY": "1",
                "ROS2CLI_NO_DAEMON": "1",
            }
        )
        if arguments.estimator == "fastlio":
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
                    f"output_odom_topic:={FASTLIO_TOPIC}", "-p",
                    "publish_tf:=false",
                ],
                environment,
            ), "FAST-LIO2 odometry bridge"))
        elif arguments.estimator == "fastlivo":
            processes.append((start(
                [
                    "ros2", "run", "fast_livo", "fastlivo_mapping",
                    "--ros-args", "-r", "__node:=fastlivo_comparison",
                    *[
                        item
                        for path in fastlivo_parameter_files
                        for item in ("--params-file", path)
                    ],
                    "-p", "use_sim_time:=true",
                    "-r", f"/aft_mapped_to_init:={FASTLIVO_TOPIC}",
                    "-r", "/path:=/comparison/fastlivo/path",
                    "-r",
                    "/cloud_registered:=/comparison/fastlivo/cloud_registered",
                    "-r", "/rgb_img:=/comparison/fastlivo/rgb_img",
                ],
                environment,
            ), "FAST-LIVO2"))
        else:
            kf_gins_command = [
                "ros2", "run", "agribot_hardware_bringup",
                "rtk_eskf_localization", "--ros-args",
                *[
                    item
                    for path in kf_gins_configs
                    for item in ("--params-file", path)
                ],
                "-p", "use_sim_time:=true", "-p",
                f"output_odom_topic:={KF_GINS_TOPIC}", "-p",
                "raw_pose_topic:=/comparison/kf_gins/raw_pose", "-p",
                "imu_processing_delay_sec:=0.20",
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

        output_topic = ESTIMATOR_TOPICS[arguments.estimator]
        recorder = start(
            ["ros2", "bag", "record", "-o", output_bag, output_topic],
            environment,
        )
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise ComparisonError("comparison bag recorder exited before playback")

        run(
            [
                "ros2", "bag", "play", source_bag, "--clock", "100",
                "--rate", str(arguments.playback_rate),
                "--read-ahead-queue-size", "1000", "--topics",
                *ESTIMATOR_INPUT_TOPICS[arguments.estimator],
            ],
            environment,
        )
        time.sleep(arguments.settle_seconds)
        stop(recorder, "comparison bag recorder")
        recorder = None

        counts = topic_counts(output_bag)
        if counts.get(output_topic, 0) < 1:
            raise ComparisonError(
                f"recomputed output is missing topic: {output_topic}"
            )
        print("\nSingle-estimator recomputation completed:", flush=True)
        print(f"  estimator: {arguments.estimator}", flush=True)
        print(f"  output_bag: {output_bag}", flush=True)
        print(f"  poses: {counts[output_topic]}", flush=True)
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
