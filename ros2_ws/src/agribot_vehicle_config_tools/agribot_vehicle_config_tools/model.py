"""Canonical vehicle configuration validation and rigid-transform helpers."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable


class ConfigError(ValueError):
    """Raised when a vehicle export cannot safely drive ROS configuration."""


Matrix3 = list[list[float]]
Vector3 = list[float]


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{path} must be finite")
    return result


def _vector(value: Any, size: int, path: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise ConfigError(f"{path} must contain exactly {size} numbers")
    return [_finite_number(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _positive(value: Any, path: str, *, allow_zero: bool = False) -> float:
    result = _finite_number(value, path)
    if result < 0.0 if allow_zero else result <= 0.0:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ConfigError(f"{path} must be {qualifier}")
    return result


def _require_keys(value: Any, keys: Iterable[str], path: str) -> None:
    if not isinstance(value, dict):
        raise ConfigError(f"{path} must be an object")
    missing = [key for key in keys if key not in value]
    if missing:
        raise ConfigError(f"{path} is missing: {', '.join(missing)}")


def quaternion_from_rpy(rpy: Vector3) -> list[float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def matrix_from_rpy(rpy: Vector3) -> Matrix3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rpy_from_matrix(rotation: Matrix3) -> Vector3:
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return [roll, pitch, yaw]


def matmul(left: Matrix3, right: Matrix3) -> Matrix3:
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def matvec(rotation: Matrix3, vector: Vector3) -> Vector3:
    return [sum(rotation[row][k] * vector[k] for k in range(3)) for row in range(3)]


def transpose(rotation: Matrix3) -> Matrix3:
    return [[rotation[column][row] for column in range(3)] for row in range(3)]


def compose_pose(parent_to_child: dict[str, Any], child_to_grandchild: dict[str, Any]) -> dict[str, Any]:
    first_rotation = matrix_from_rpy(parent_to_child["rpy"])
    second_rotation = matrix_from_rpy(child_to_grandchild["rpy"])
    rotation = matmul(first_rotation, second_rotation)
    translated = matvec(first_rotation, child_to_grandchild["xyz"])
    xyz = [parent_to_child["xyz"][index] + translated[index] for index in range(3)]
    rpy = rpy_from_matrix(rotation)
    return {"xyz": xyz, "rpy": rpy, "quaternion": quaternion_from_rpy(rpy)}


def inverse_pose(pose: dict[str, Any]) -> dict[str, Any]:
    inverse_rotation = transpose(matrix_from_rpy(pose["rpy"]))
    xyz = matvec(inverse_rotation, [-value for value in pose["xyz"]])
    rpy = rpy_from_matrix(inverse_rotation)
    return {"xyz": xyz, "rpy": rpy, "quaternion": quaternion_from_rpy(rpy)}


def relative_pose(base_to_source: dict[str, Any], base_to_target: dict[str, Any]) -> dict[str, Any]:
    """Return target <- source, so p_target = R * p_source + t."""
    return compose_pose(inverse_pose(base_to_target), base_to_source)


def effective_sensor_pose(sensor: dict[str, Any]) -> dict[str, Any]:
    measurement_pose = sensor.get("measurementFramePose")
    if measurement_pose is None or not sensor.get("measurementFrameId"):
        return sensor["pose"]
    return compose_pose(sensor["pose"], measurement_pose)


def sensor_by_type(config: dict[str, Any], sensor_type: str) -> dict[str, Any]:
    matches = [sensor for sensor in config["sensors"] if sensor["sensorType"] == sensor_type]
    if len(matches) != 1:
        raise ConfigError(f"expected exactly one {sensor_type} sensor, found {len(matches)}")
    return matches[0]


def sensor_by_role(config: dict[str, Any], role: str) -> dict[str, Any]:
    matches = [sensor for sensor in config["sensors"] if sensor.get("role") == role]
    if len(matches) != 1:
        raise ConfigError(f"expected exactly one sensor with role {role}, found {len(matches)}")
    return matches[0]


def footprint(config: dict[str, Any]) -> list[list[float]]:
    body = config["geometry"]["bodyExtents"]
    clearance = config["navigation"]["clearance"]
    front = body["frontM"] + clearance["frontM"]
    rear = body["rearM"] + clearance["rearM"]
    left = body["leftM"] + clearance["leftM"]
    right = body["rightM"] + clearance["rightM"]
    return [[front, left], [front, -right], [-rear, -right], [-rear, left]]


def fastlivo_extrinsics(config: dict[str, Any]) -> dict[str, list[float]]:
    imu_pose = effective_sensor_pose(sensor_by_type(config, "imu"))
    lidar_pose = effective_sensor_pose(sensor_by_type(config, "lidar"))
    camera_pose = effective_sensor_pose(sensor_by_type(config, "camera"))
    lidar_to_imu = relative_pose(lidar_pose, imu_pose)
    lidar_to_camera = relative_pose(lidar_pose, camera_pose)
    return {
        "extrinsic_T": lidar_to_imu["xyz"],
        "extrinsic_R": [value for row in matrix_from_rpy(lidar_to_imu["rpy"]) for value in row],
        "Pcl": lidar_to_camera["xyz"],
        "Rcl": [value for row in matrix_from_rpy(lidar_to_camera["rpy"]) for value in row],
    }


def validate(config: dict[str, Any]) -> list[str]:
    _require_keys(
        config,
        (
            "schemaVersion",
            "vehicleId",
            "displayName",
            "vehicleType",
            "coordinateConvention",
            "baseFrame",
            "geometry",
            "navigation",
            "chassis",
            "sensors",
        ),
        "vehicle configuration",
    )
    if config["schemaVersion"] != "1.0":
        raise ConfigError("schemaVersion must be 1.0")
    if config["coordinateConvention"] != "ROS_FLU":
        raise ConfigError("coordinateConvention must be ROS_FLU")
    if config["vehicleType"] not in {"ackermann", "differential", "tracked"}:
        raise ConfigError("vehicleType is unsupported")
    if not isinstance(config["vehicleId"], str) or not config["vehicleId"]:
        raise ConfigError("vehicleId must not be empty")

    base_frame = config["baseFrame"]
    _require_keys(base_frame, ("frameId", "reference"), "baseFrame")
    if base_frame["frameId"] != "base_link":
        raise ConfigError("baseFrame.frameId must be base_link")

    geometry = config["geometry"]
    _require_keys(
        geometry,
        (
            "bodyExtents",
            "wheelbaseM",
            "frontTrackM",
            "rearTrackM",
            "wheelRadiusM",
            "wheelWidthM",
            "maxSteeringAngleRad",
            "minTurningRadiusM",
            "wheelCenters",
        ),
        "geometry",
    )
    for key in (
        "wheelbaseM",
        "frontTrackM",
        "rearTrackM",
        "wheelRadiusM",
        "wheelWidthM",
        "maxSteeringAngleRad",
        "minTurningRadiusM",
    ):
        _positive(geometry[key], f"geometry.{key}")
    extents = geometry["bodyExtents"]
    _require_keys(extents, ("frontM", "rearM", "leftM", "rightM"), "geometry.bodyExtents")
    for key in ("frontM", "rearM", "leftM", "rightM"):
        _positive(extents[key], f"geometry.bodyExtents.{key}")

    wheels = geometry["wheelCenters"]
    if not isinstance(wheels, list) or len(wheels) != 4:
        raise ConfigError("geometry.wheelCenters must contain four wheels")
    required_wheels = {"front_left", "front_right", "rear_left", "rear_right"}
    wheel_ids = set()
    for index, wheel in enumerate(wheels):
        _require_keys(wheel, ("id", "position", "steering", "driven"), f"wheelCenters[{index}]")
        wheel_ids.add(wheel["id"])
        _vector(wheel["position"], 3, f"wheelCenters[{index}].position")
    if wheel_ids != required_wheels:
        raise ConfigError("wheel IDs must be front_left, front_right, rear_left and rear_right")

    navigation = config["navigation"]
    _require_keys(
        navigation,
        (
            "clearance",
            "maxLinearSpeedMps",
            "maxAngularSpeedRadps",
            "allowReverse",
            "inflationRadiusM",
            "costScalingFactor",
        ),
        "navigation",
    )
    clearance = navigation["clearance"]
    _require_keys(clearance, ("frontM", "rearM", "leftM", "rightM"), "navigation.clearance")
    for key in ("frontM", "rearM", "leftM", "rightM"):
        _positive(clearance[key], f"navigation.clearance.{key}", allow_zero=True)
    _positive(navigation["maxLinearSpeedMps"], "navigation.maxLinearSpeedMps")
    _positive(navigation["maxAngularSpeedRadps"], "navigation.maxAngularSpeedRadps")
    _positive(navigation["inflationRadiusM"], "navigation.inflationRadiusM", allow_zero=True)
    _positive(navigation["costScalingFactor"], "navigation.costScalingFactor")

    chassis = config["chassis"]
    _require_keys(
        chassis,
        (
            "commandMaxLinearSpeedMps",
            "commandMaxAngularSpeedRadps",
            "minimumMotionSpeedMps",
        ),
        "chassis",
    )
    _positive(chassis["commandMaxLinearSpeedMps"], "chassis.commandMaxLinearSpeedMps")
    _positive(chassis["commandMaxAngularSpeedRadps"], "chassis.commandMaxAngularSpeedRadps")
    _positive(chassis["minimumMotionSpeedMps"], "chassis.minimumMotionSpeedMps", allow_zero=True)
    if navigation["maxLinearSpeedMps"] > chassis["commandMaxLinearSpeedMps"]:
        raise ConfigError("navigation max speed exceeds the chassis command limit")
    if navigation["maxAngularSpeedRadps"] > chassis["commandMaxAngularSpeedRadps"]:
        raise ConfigError("navigation max angular speed exceeds the chassis command limit")

    sensors = config["sensors"]
    if not isinstance(sensors, list) or not sensors:
        raise ConfigError("sensors must not be empty")
    sensor_ids = set()
    frame_ids = set()
    for index, sensor in enumerate(sensors):
        path = f"sensors[{index}]"
        _require_keys(
            sensor,
            ("id", "sensorType", "model", "parentFrameId", "frameId", "pose"),
            path,
        )
        if sensor["id"] in sensor_ids:
            raise ConfigError(f"duplicate sensor ID: {sensor['id']}")
        sensor_ids.add(sensor["id"])
        for frame_key in ("frameId", "measurementFrameId"):
            frame_id = sensor.get(frame_key)
            if not frame_id:
                continue
            if frame_id in frame_ids:
                raise ConfigError(f"duplicate sensor frame: {frame_id}")
            frame_ids.add(frame_id)
        _validate_pose(sensor["pose"], f"{path}.pose")
        if sensor.get("measurementFrameId"):
            if sensor.get("measurementFramePose") is None:
                raise ConfigError(f"{path}.measurementFramePose is required with measurementFrameId")
            _validate_pose(sensor["measurementFramePose"], f"{path}.measurementFramePose")

    warnings = []
    if config["vehicleType"] == "ackermann":
        calculated_radius = geometry["wheelbaseM"] / math.tan(geometry["maxSteeringAngleRad"])
        if abs(calculated_radius - geometry["minTurningRadiusM"]) > 0.01:
            raise ConfigError(
                "geometry.minTurningRadiusM disagrees with wheelbase/max steering "
                f"({calculated_radius:.6f} m expected)"
            )
        curvature_limit = navigation["maxLinearSpeedMps"] / geometry["minTurningRadiusM"]
        if navigation["maxAngularSpeedRadps"] > curvature_limit + 1e-3:
            warnings.append(
                "navigation.maxAngularSpeedRadps exceeds the Ackermann curvature limit "
                f"at max speed ({curvature_limit:.6f} rad/s)"
            )
        sensor_by_type(config, "imu")
        sensor_by_type(config, "lidar")
        sensor_by_type(config, "camera")
        sensor_by_role(config, "master_left")
        sensor_by_role(config, "secondary_right")

    return warnings


def _validate_pose(pose: Any, path: str) -> None:
    _require_keys(pose, ("xyz", "quaternion", "rpy"), path)
    _vector(pose["xyz"], 3, f"{path}.xyz")
    quaternion = _vector(pose["quaternion"], 4, f"{path}.quaternion")
    rpy = _vector(pose["rpy"], 3, f"{path}.rpy")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.0) > 1e-4:
        raise ConfigError(f"{path}.quaternion is not normalized")
    expected = quaternion_from_rpy(rpy)
    direct_error = max(abs(left - right) for left, right in zip(quaternion, expected))
    negated_error = max(abs(left + right) for left, right in zip(quaternion, expected))
    if min(direct_error, negated_error) > 1e-5:
        raise ConfigError(f"{path}.quaternion and rpy describe different rotations")


def load_and_validate(path: str | Path) -> tuple[dict[str, Any], list[str]]:
    source = Path(path)
    try:
        config = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ConfigError(f"cannot read {source}: {error}") from error
    warnings = validate(config)
    return config, warnings
