"""Compare a generated vehicle contract with the currently validated Ackermann files."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .model import effective_sensor_pose, fastlivo_extrinsics, footprint


@dataclass(frozen=True)
class CheckResult:
    name: str
    expected: Any
    actual: Any
    passed: bool
    source: str


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _close(expected: Any, actual: Any, tolerance: float = 1e-6) -> bool:
    if isinstance(expected, (list, tuple)) and isinstance(actual, (list, tuple)):
        return len(expected) == len(actual) and all(
            _close(left, right, tolerance) for left, right in zip(expected, actual)
        )
    try:
        return math.isclose(float(expected), float(actual), abs_tol=tolerance, rel_tol=0.0)
    except (TypeError, ValueError):
        return expected == actual


def _check(
    results: list[CheckResult],
    name: str,
    expected: Any,
    actual: Any,
    source: Path,
    tolerance: float = 1e-6,
) -> None:
    results.append(
        CheckResult(
            name=name,
            expected=expected,
            actual=actual,
            passed=_close(expected, actual, tolerance),
            source=str(source),
        )
    )


def verify_current(config: dict[str, Any], workspace_src: Path) -> list[CheckResult]:
    results: list[CheckResult] = []
    hardware = workspace_src / "agribot_hardware_bringup"
    simulation = workspace_src / "agribot_ackermann_mppi"
    fastlivo = workspace_src / "FAST-LIVO2"

    mounts_path = hardware / "config" / "sensor_mounts.yaml"
    mounts = _load_yaml(mounts_path)
    sensor_names = {
        "imu": "imu",
        "lidar": "lidar",
        "right_camera": "camera",
        "rtk_master": "rtk",
    }
    config_sensors = {sensor["id"]: sensor for sensor in config["sensors"]}
    for config_id, current_id in sensor_names.items():
        expected_pose = effective_sensor_pose(config_sensors[config_id])
        _check(
            results,
            f"sensor.{config_id}.xyz",
            expected_pose["xyz"],
            mounts[current_id]["xyz"],
            mounts_path,
        )
        _check(
            results,
            f"sensor.{config_id}.rpy",
            expected_pose["rpy"],
            mounts[current_id]["rpy"],
            mounts_path,
        )

    chassis_path = hardware / "ackermann" / "config" / "chassis_can.yaml"
    chassis = _load_yaml(chassis_path)["/**"]["ros__parameters"]
    geometry = config["geometry"]
    navigation = config["navigation"]
    chassis_config = config["chassis"]
    chassis_fields = {
        "wheelbase_m": geometry["wheelbaseM"],
        "max_steering_angle_rad": geometry["maxSteeringAngleRad"],
        "max_linear_velocity": chassis_config["commandMaxLinearSpeedMps"],
        "max_angular_velocity": chassis_config["commandMaxAngularSpeedRadps"],
    }
    for field, expected in chassis_fields.items():
        _check(results, f"chassis.{field}", expected, chassis[field], chassis_path)

    joint_path = hardware / "ackermann" / "config" / "joint_state_publisher.yaml"
    joint = _load_yaml(joint_path)["/**"]["ros__parameters"]
    joint_fields = {
        "wheelbase_m": geometry["wheelbaseM"],
        "front_track_m": geometry["frontTrackM"],
        "rear_track_m": geometry["rearTrackM"],
        "wheel_radius_m": geometry["wheelRadiusM"],
        "max_steering_angle_rad": geometry["maxSteeringAngleRad"],
    }
    for field, expected in joint_fields.items():
        _check(results, f"joint_state.{field}", expected, joint[field], joint_path)

    nav_path = hardware / "ackermann" / "config" / "nav2_params_ackermann_fastlio_mapped.yaml"
    nav = _load_yaml(nav_path)
    controller = nav["controller_server"]["ros__parameters"]["FollowPath"]
    costmap = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = nav["planner_server"]["ros__parameters"]["GridBased"]
    current_footprint = json.loads(costmap["footprint"])
    _check(results, "nav2.footprint", footprint(config), current_footprint, nav_path)
    _check(
        results,
        "nav2.min_turning_radius",
        geometry["minTurningRadiusM"],
        controller["AckermannConstraints"]["min_turning_r"],
        nav_path,
        # Unity calculates this value from single-precision wheelbase and
        # steering-angle fields; the production YAML is rounded to 6 decimals.
        tolerance=1e-5,
    )
    _check(results, "nav2.max_linear_speed", navigation["maxLinearSpeedMps"], controller["vx_max"], nav_path)
    _check(results, "nav2.max_angular_speed", navigation["maxAngularSpeedRadps"], controller["wz_max"], nav_path)
    _check(results, "nav2.motion_model", "DUBIN", planner["motion_model_for_search"], nav_path)
    _check(
        results,
        "nav2.inflation_radius",
        navigation["inflationRadiusM"],
        costmap["inflation_layer"]["inflation_radius"],
        nav_path,
    )
    _check(
        results,
        "nav2.cost_scaling_factor",
        navigation["costScalingFactor"],
        costmap["inflation_layer"]["cost_scaling_factor"],
        nav_path,
    )

    fastlivo_path = fastlivo / "config" / "agribot_c16_astra.yaml"
    current_extrinsics = _load_yaml(fastlivo_path)["/**"]["ros__parameters"]["extrin_calib"]
    expected_extrinsics = fastlivo_extrinsics(config)
    for field in ("extrinsic_T", "extrinsic_R", "Pcl", "Rcl"):
        _check(
            results,
            f"fastlivo.{field}",
            expected_extrinsics[field],
            current_extrinsics[field],
            fastlivo_path,
            tolerance=2e-6,
        )

    sdf_path = simulation / "models" / "ackermann_scout.sdf"
    root = ET.parse(sdf_path).getroot()
    plugin = root.find(".//plugin[@name='ackermann_drive']")
    if plugin is None:
        raise RuntimeError(f"ackermann_drive plugin is missing from {sdf_path}")
    sdf_fields = {
        "wheelbase": geometry["wheelbaseM"],
        "front_track_width": geometry["frontTrackM"],
        "rear_track_width": geometry["rearTrackM"],
        "wheel_radius": geometry["wheelRadiusM"],
        "max_steering_angle": geometry["maxSteeringAngleRad"],
        "max_speed": navigation["maxLinearSpeedMps"],
    }
    for field, expected in sdf_fields.items():
        _check(results, f"simulation.{field}", expected, float(plugin.findtext(field)), sdf_path)

    wheel_map = {wheel["id"]: wheel for wheel in geometry["wheelCenters"]}
    for wheel_id, wheel in wheel_map.items():
        link = root.find(f".//link[@name='{wheel_id}_wheel_link']")
        if link is None:
            results.append(CheckResult(f"simulation.{wheel_id}.position", wheel["position"], None, False, str(sdf_path)))
            continue
        pose_values = [float(value) for value in link.findtext("pose").split()[:3]]
        _check(
            results,
            f"simulation.{wheel_id}.position",
            wheel["position"],
            pose_values,
            sdf_path,
            tolerance=5e-4,
        )

    sensor_sdf_path = simulation / "models" / "ackermann_scout_sensor.sdf"
    sensor_root = ET.parse(sensor_sdf_path).getroot()
    sdf_sensor_map = {
        "imu": "n300pro_imu",
        "lidar": "lslidar_c16_points",
        "right_camera": "hikrobot_right_camera",
    }
    for sensor_id, sdf_name in sdf_sensor_map.items():
        sensor_element = sensor_root.find(f".//sensor[@name='{sdf_name}']")
        if sensor_element is None:
            results.append(CheckResult(f"simulation_sensor.{sensor_id}", "present", None, False, str(sensor_sdf_path)))
            continue
        current_pose = [float(value) for value in sensor_element.findtext("pose").split()]
        expected_pose = config_sensors[sensor_id]["pose"]
        _check(
            results,
            f"simulation_sensor.{sensor_id}.pose",
            expected_pose["xyz"] + expected_pose["rpy"],
            current_pose,
            sensor_sdf_path,
        )

    return results
