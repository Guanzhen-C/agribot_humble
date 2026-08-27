#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import yaml


LOCAL_INPUT_TOPICS = (
    "/lidar/points",
    "/imu/data",
)
RTK_INPUT_TOPICS = (
    "/rtk/fix",
    "/rtk/fix_quality",
    "/rtk/heading_with_covariance",
    "/rtk/heading_solution",
)
LOCAL_RESULT_TOPICS = (
    "/lio_sam/mapping/odometry",
    "/lio_sam/mapping/path",
)
RTK_RESULT_TOPICS = (
    "/lio_sam/odometry/gps",
    "/lio_sam/odometry/heading",
    "/lio_sam/odometry/rtk_antenna",
    "/lio_sam/rtk_adapter_status",
    "/lio_sam/rtk_reference",
)
RESULT_TOPICS = LOCAL_RESULT_TOPICS + RTK_RESULT_TOPICS
VEHICLE_PROFILES = ("ackermann", "differential")


def playback_topics(without_rtk):
    if without_rtk:
        return LOCAL_INPUT_TOPICS
    return LOCAL_INPUT_TOPICS + RTK_INPUT_TOPICS


def resolve_profile_paths(
    vehicle_profile, offline_share=None, hardware_share=None
):
    if vehicle_profile not in VEHICLE_PROFILES:
        raise PipelineError(f"unsupported vehicle profile: {vehicle_profile}")
    offline_share = Path(offline_share or Path(__file__).resolve().parents[1])
    if hardware_share is None:
        source_candidate = offline_share.parent / "agribot_hardware_bringup"
        hardware_share = (
            source_candidate
            if source_candidate.is_dir()
            else Path(get_package_share_directory("agribot_hardware_bringup"))
        )
    else:
        hardware_share = Path(hardware_share)

    suffix = "_differential" if vehicle_profile == "differential" else ""
    mounts = (
        hardware_share / "differential" / "config" / "sensor_mounts.yaml"
        if vehicle_profile == "differential"
        else hardware_share / "config" / "sensor_mounts.yaml"
    )
    paths = {
        "lio_sam": offline_share / "config" / f"lio_sam_c16{suffix}.yaml",
        "point_adapter": (
            offline_share
            / "config"
            / f"lslidar_lio_sam_adapter{suffix}.yaml"
        ),
        "rtk_adapter": (
            offline_share / "config" / f"rtk_odometry_adapter{suffix}.yaml"
        ),
        "georeference": (
            offline_share / "config" / "map_georeference_exporter.yaml"
        ),
        "sensor_mounts": mounts,
    }
    missing = [path for path in paths.values() if not path.is_file()]
    if missing:
        raise PipelineError(
            "vehicle profile configuration is incomplete:\n"
            + "\n".join(f"  - {path}" for path in missing)
        )
    return paths


def ros_parameters(path, node_name=None):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if "/**" in document:
        return document["/**"]["ros__parameters"]
    if node_name is not None and node_name in document:
        return document[node_name]["ros__parameters"]
    raise PipelineError(f"ROS parameter document has an unexpected root: {path}")


class PipelineError(RuntimeError):
    pass


def command_text(command):
    return " ".join(shlex.quote(str(value)) for value in command)


def run(command, environment, *, timeout=None, capture=False):
    print(f"\n$ {command_text(command)}", flush=True)
    result = subprocess.run(
        [str(value) for value in command],
        env=environment,
        check=False,
        timeout=timeout,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )
    if capture and result.stdout:
        print(result.stdout.rstrip(), flush=True)
    if result.returncode != 0:
        raise PipelineError(
            f"command failed with exit code {result.returncode}: "
            f"{command_text(command)}"
        )
    return result.stdout or ""


def start(command, environment):
    print(f"\n$ {command_text(command)}", flush=True)
    return subprocess.Popen([str(value) for value in command], env=environment)


def stop(process, name, timeout=30.0):
    if process is None or process.poll() is not None:
        return
    print(f"Stopping {name} ...", flush=True)
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def wait_for_service(service_name, environment, timeout):
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
        time.sleep(0.5)
    raise PipelineError(f"service did not become ready: {service_name}")


def service_succeeded(output):
    normalized = " ".join(output.lower().split())
    return "success: true" in normalized or "success=true" in normalized


def output_paths(map_base):
    return {
        "pcd": map_base.with_suffix(".pcd"),
        "pgm": map_base.with_suffix(".pgm"),
        "yaml": map_base.with_suffix(".yaml"),
        "georeference": map_base.parent / f"{map_base.name}_georeference.yaml",
        "manifest": map_base.parent / f"{map_base.name}_manifest.yaml",
        "result_bag": map_base.parent / f"{map_base.name}_result",
    }


def validate_inputs(arguments):
    bag = arguments.bag.resolve()
    if not arguments.map_base.expanduser().is_absolute():
        raise PipelineError("map_base must be an absolute path")
    map_base = arguments.map_base.expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise PipelineError(f"ROS bag directory is invalid: {bag}")
    if map_base.suffix:
        raise PipelineError("map_base must not contain .pcd, .pgm or .yaml")
    if re.fullmatch(r"[A-Za-z0-9_-]+", map_base.name) is None:
        raise PipelineError(
            "map name may contain only letters, digits, underscores and hyphens"
        )
    if arguments.playback_rate <= 0.0:
        raise PipelineError("playback_rate must be positive")
    if arguments.settle_seconds < 0.0:
        raise PipelineError("settle_seconds must not be negative")
    if arguments.save_resolution < 0.0:
        raise PipelineError("save_resolution must not be negative")
    if not 0 <= arguments.domain_id <= 232:
        raise PipelineError("domain_id must be in [0, 232]")
    if shutil.which("ros2") is None:
        raise PipelineError("ros2 is not available; source the ROS and workspace setup files")
    return bag, map_base


def remove_existing(paths, work_directory, force):
    existing = [path for path in paths.values() if path.exists()]
    if work_directory.exists():
        existing.append(work_directory)
    if existing and not force:
        formatted = "\n".join(f"  - {path}" for path in existing)
        raise PipelineError(
            "output already exists; choose another map name or pass --force:\n"
            + formatted
        )
    if not force:
        return
    for path in existing:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()


def write_manifest(
    path, bag, map_base, work_directory, paths, arguments, profile_paths=None
):
    rtk_enabled = not arguments.without_rtk
    profile_paths = profile_paths or resolve_profile_paths(
        arguments.vehicle_profile
    )
    lio_parameters = ros_parameters(profile_paths["lio_sam"])
    point_parameters = ros_parameters(
        profile_paths["point_adapter"], "lslidar_lio_sam_adapter"
    )
    rtk_parameters = ros_parameters(
        profile_paths["rtk_adapter"], "rtk_odometry_adapter"
    )
    mounts = yaml.safe_load(
        profile_paths["sensor_mounts"].read_text(encoding="utf-8")
    )
    artifacts = {
        key: str(paths[key]) for key in ("pcd", "pgm", "yaml")
    }
    if rtk_enabled:
        artifacts["georeference"] = str(paths["georeference"])
    document = {
        "schema_version": 1,
        "pipeline": (
            "lio_sam_rtk_gravity_robust_xy_v2"
            if rtk_enabled
            else "lio_sam_gravity_indoor_v1"
        ),
        "rtk_mode": "required" if rtk_enabled else "disabled",
        "vehicle_profile": arguments.vehicle_profile,
        "source_bag": str(bag),
        "map_base": str(map_base),
        "work_directory": str(work_directory),
        "result_bag": str(paths["result_bag"]),
        "artifacts": artifacts,
        "rtk_factor": {
            "enabled": rtk_enabled,
            "horizontal_variance_floor_m2": float(
                lio_parameters["gpsHorizontalCovarianceFloor"]
            ),
            "horizontal_standard_deviation_floor_m": float(
                lio_parameters["gpsHorizontalCovarianceFloor"]
            ) ** 0.5,
            "factor_minimum_distance_m": float(
                lio_parameters["gpsFactorMinDistance"]
            ),
            "use_gps_elevation": bool(lio_parameters["useGpsElevation"]),
            "huber_delta": float(lio_parameters["gpsRobustKernelDelta"]),
            "antenna_to_lidar_flu_m": list(
                rtk_parameters["antenna_to_lidar_flu_m"]
            ),
        },
        "map_leveling": {
            "gravity_attitude_factor": bool(
                lio_parameters["useGravityAttitudeFactor"]
            ),
            "gravity_attitude_sigma_deg": float(
                lio_parameters["gravityAttitudeSigma"]
            ) * 180.0 / 3.141592653589793,
            "gravity_attitude_huber_delta": float(
                lio_parameters["gravityAttitudeRobustKernelDelta"]
            ),
            "initial_roll_pitch_sigma_deg": float(
                lio_parameters["initialRollPitchSigma"]
            ) * 180.0 / 3.141592653589793,
            "initial_z_sigma_m": float(lio_parameters["initialZSigma"]),
        },
        "point_exclusion": {
            "rear_person_region_base_link": {
                "minimum_x_m": float(point_parameters["rear_exclusion_min_x"]),
                "maximum_x_m": float(point_parameters["rear_exclusion_max_x"]),
                "half_width_m": float(
                    point_parameters["rear_exclusion_half_width"]
                ),
            },
            "rtk_antenna_boxes_base_link": {
                "left_center_xyz_m": list(
                    point_parameters["left_antenna_center_xyz"]
                ),
                "right_center_xyz_m": list(
                    point_parameters["right_antenna_center_xyz"]
                ),
                "half_extent_xyz_m": list(
                    point_parameters["antenna_exclusion_half_extent_xyz"]
                ),
            },
        },
        "sensor_mounts": mounts,
        "input_stamp_is_scan_end": bool(
            point_parameters["input_stamp_is_scan_end"]
        ),
        "configuration_files": {
            key: value.name for key, value in profile_paths.items()
        },
        "playback_rate": arguments.playback_rate,
        "level_horizontal_trajectory": False,
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Build a LIO-SAM map and retain matching trajectories. By default "
            "fixed RTK constrains and georeferences the map; --without-rtk "
            "builds an indoor lidar/IMU-only map."
        )
    )
    parser.add_argument("bag", type=Path, help="input ROS 2 bag directory")
    parser.add_argument(
        "map_base", type=Path, help="absolute output map path without an extension"
    )
    parser.add_argument("--domain-id", type=int, default=71)
    parser.add_argument(
        "--vehicle-profile",
        choices=VEHICLE_PROFILES,
        default="ackermann",
        help="physical sensor geometry used by mapping and trajectory export",
    )
    parser.add_argument("--playback-rate", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=10.0)
    parser.add_argument("--save-resolution", type=float, default=0.1)
    parser.add_argument(
        "--work-root",
        type=Path,
        default=Path.home() / "agribot_maps" / "lio_sam_work",
    )
    parser.add_argument(
        "--force", action="store_true", help="replace outputs with the same map name"
    )
    parser.add_argument(
        "--without-rtk",
        action="store_true",
        help=(
            "do not start the RTK adapter/exporter, add RTK factors, or require "
            "a georeference artifact"
        ),
    )
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_arguments(argv)
    launch_process = None
    record_process = None
    try:
        bag, map_base = validate_inputs(arguments)
        paths = output_paths(map_base)
        profile_paths = resolve_profile_paths(
            arguments.vehicle_profile,
            Path(get_package_share_directory("agribot_offline_mapping")),
            Path(get_package_share_directory("agribot_hardware_bringup")),
        )
        work_directory = arguments.work_root.expanduser().resolve() / map_base.name
        try:
            work_relative_to_home = work_directory.relative_to(Path.home().resolve())
        except ValueError as error:
            raise PipelineError("work_root must be inside the current user's home") from error
        remove_existing(paths, work_directory, arguments.force)
        map_base.parent.mkdir(parents=True, exist_ok=True)
        work_directory.parent.mkdir(parents=True, exist_ok=True)

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
                "ros2", "launch", "agribot_offline_mapping",
                "lio_sam_rtk_offline.launch.py",
                f"map_base:={map_base}",
                f"source_bag:={bag}",
                "use_sim_time:=true",
                f"start_rtk_components:={'false' if arguments.without_rtk else 'true'}",
                f"lio_sam_parameters:={profile_paths['lio_sam']}",
                f"point_adapter_parameters:={profile_paths['point_adapter']}",
                f"rtk_adapter_parameters:={profile_paths['rtk_adapter']}",
                f"georeference_parameters:={profile_paths['georeference']}",
                f"sensor_mounts_file:={profile_paths['sensor_mounts']}",
            ],
            environment,
        )
        wait_for_service("/lio_sam/save_map", environment, 45.0)

        record_process = start(
            [
                "ros2", "bag", "record", "-o", str(paths["result_bag"]),
                *(
                    LOCAL_RESULT_TOPICS
                    if arguments.without_rtk
                    else RESULT_TOPICS
                ),
            ],
            environment,
        )
        time.sleep(2.0)
        if record_process.poll() is not None:
            raise PipelineError("result bag recorder exited before playback")

        run(
            [
                "ros2", "bag", "play", str(bag),
                "--clock", "100",
                "--rate", str(arguments.playback_rate),
                "--read-ahead-queue-size", "1000",
                "--topics", *playback_topics(arguments.without_rtk),
                "--disable-keyboard-controls",
            ],
            environment,
        )
        print(
            "Playback complete; waiting "
            f"{arguments.settle_seconds:.1f} s for final optimization ...",
            flush=True,
        )
        time.sleep(arguments.settle_seconds)

        save_output = run(
            [
                "ros2", "service", "call", "/lio_sam/save_map",
                "lio_sam/srv/SaveMap",
                (
                    "{resolution: " + str(arguments.save_resolution)
                    + ", destination: '/" + str(work_relative_to_home) + "'}"
                ),
            ],
            environment,
            timeout=900.0,
            capture=True,
        )
        if not service_succeeded(save_output):
            raise PipelineError("LIO-SAM save_map service did not report success")

        finalize_command = [
            "ros2", "run", "agribot_offline_mapping",
            "finalize_lio_sam_map.py", str(work_directory), str(map_base),
        ]
        run(finalize_command, environment, timeout=900.0)

        if not arguments.without_rtk:
            georeference_output = run(
                [
                    "ros2", "service", "call", "/map_georeference_exporter/save",
                    "std_srvs/srv/Trigger", "{}",
                ],
                environment,
                timeout=120.0,
                capture=True,
            )
            if not service_succeeded(georeference_output):
                raise PipelineError("map georeference service did not report success")

        stop(record_process, "result bag recorder")
        record_process = None
        required = [
            paths["pcd"], paths["pgm"], paths["yaml"],
            paths["result_bag"] / "metadata.yaml",
        ]
        if not arguments.without_rtk:
            required.append(paths["georeference"])
        missing = [path for path in required if not path.exists()]
        if missing:
            raise PipelineError(
                "pipeline completed but required artifacts are missing:\n"
                + "\n".join(f"  - {path}" for path in missing)
            )
        write_manifest(
            paths["manifest"], bag, map_base, work_directory, paths, arguments,
            profile_paths,
        )
        print("\nPipeline completed successfully:", flush=True)
        output_keys = ["pcd", "pgm", "yaml"]
        if not arguments.without_rtk:
            output_keys.append("georeference")
        output_keys.extend(("result_bag", "manifest"))
        for key in output_keys:
            print(f"  {key}: {paths[key]}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        return 130
    except (PipelineError, subprocess.TimeoutExpired) as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        return 1
    finally:
        stop(record_process, "result bag recorder")
        stop(launch_process, "LIO-SAM launch")


if __name__ == "__main__":
    raise SystemExit(main())
