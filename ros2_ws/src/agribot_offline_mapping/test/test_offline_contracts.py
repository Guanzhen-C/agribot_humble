import math
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
HARDWARE_ROOT = PACKAGE_ROOT.parent / "agribot_hardware_bringup"


def parameters(path, node_name="/**"):
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node_name][
        "ros__parameters"
    ]


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def matmul(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def matvec(matrix, vector):
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def flatten(matrix):
    return [value for row in matrix for value in row]


def test_lio_sam_translation_uses_lidar_to_imu_convention():
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )
    lio_sam = parameters(PACKAGE_ROOT / "config" / "lio_sam_c16.yaml")
    lidar_to_imu_base = [
        mounts["imu"]["xyz"][index] - mounts["lidar"]["xyz"][index]
        for index in range(3)
    ]
    lidar_from_base = transpose(rpy_matrix(mounts["lidar"]["rpy"]))

    assert lio_sam["extrinsicTrans"] == pytest.approx(
        matvec(lidar_from_base, lidar_to_imu_base), abs=1.0e-8
    )
    assert lio_sam["N_SCAN"] == 16
    assert lio_sam["downsampleRate"] == 1
    assert lio_sam["loopClosureEnableFlag"] is True
    assert lio_sam["poseCovThreshold"] == pytest.approx(0.0)


def test_lio_sam_rotation_uses_calibrated_vector_and_pose_conventions():
    lio_sam = parameters(PACKAGE_ROOT / "config" / "lio_sam_c16.yaml")
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )

    def matrix(values):
        return [
            [values[row * 3 + column] for column in range(3)]
            for row in range(3)
        ]

    vector_rotation = matrix(lio_sam["extrinsicRot"])
    pose_rotation = matrix(lio_sam["extrinsicRPY"])
    expected_pose_rotation = matmul(
        transpose(rpy_matrix(mounts["imu"]["rpy"])),
        rpy_matrix(mounts["lidar"]["rpy"]),
    )

    assert vector_rotation != [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    for row in range(3):
        for column in range(3):
            assert pose_rotation[row][column] == pytest.approx(
                expected_pose_rotation[row][column], abs=1.0e-8
            )
            assert pose_rotation[row][column] == pytest.approx(
                vector_rotation[column][row], abs=1.0e-9
            )

    for left in range(3):
        for right in range(3):
            dot_product = sum(
                vector_rotation[left][index]
                * vector_rotation[right][index]
                for index in range(3)
            )
            assert dot_product == pytest.approx(
                1.0 if left == right else 0.0, abs=1.0e-8
            )


def test_official_lio_sam_uses_only_rtk_horizontal_position():
    adapter = parameters(
        PACKAGE_ROOT / "config" / "rtk_odometry_adapter.yaml",
        "rtk_odometry_adapter",
    )
    lio_sam = parameters(PACKAGE_ROOT / "config" / "lio_sam_c16.yaml")
    lio_source = (
        PACKAGE_ROOT.parent / "LIO-SAM" / "src" / "mapOptmization.cpp"
    ).read_text(encoding="utf-8")
    utility_source = (
        PACKAGE_ROOT.parent / "LIO-SAM" / "include" / "lio_sam" / "utility.hpp"
    ).read_text(encoding="utf-8")

    assert adapter["position_output_topic"] == "/lio_sam/odometry/gps"
    assert adapter["antenna_frame"] == "rtk_master_antenna"
    assert adapter["required_fix_quality"] == 4
    assert lio_sam["useGpsElevation"] is False
    assert lio_sam["gpsFactorMinDistance"] == pytest.approx(1.0)
    assert "headingTopic" not in lio_sam
    assert "positionFactorMinDistance" not in lio_sam
    assert "void addGPSFactor()" in lio_source
    assert "gtsam::GPSFactor gps_factor" in lio_source
    assert "headingHandler" not in lio_source
    assert "YawFactor" not in lio_source
    assert "AntennaPositionFactor" not in lio_source
    assert (
        "pointDistance(curGPSPoint, lastGPSPoint) < gpsFactorMinDistance"
        in lio_source
    )
    assert 'declare_parameter("gpsFactorMinDistance", 5.0)' in utility_source


def test_lio_sam_adapter_uses_measured_mount_and_mapping_rear_mask():
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )
    mapping = parameters(
        HARDWARE_ROOT / "config" / "pcd_mapping.yaml", "pcd_map_builder"
    )
    c16 = parameters(
        HARDWARE_ROOT / "config" / "c16.yaml", "lslidar_driver_node"
    )
    adapter = parameters(
        PACKAGE_ROOT / "config" / "lslidar_lio_sam_adapter.yaml",
        "lslidar_lio_sam_adapter",
    )

    assert adapter["base_to_lidar_xyz"] == pytest.approx(
        mounts["lidar"]["xyz"]
    )
    assert adapter["base_to_lidar_rpy"] == pytest.approx(
        mounts["lidar"]["rpy"]
    )
    assert c16["use_first_point_time"] is False
    assert adapter["input_stamp_is_scan_end"] is True
    assert adapter["rear_exclusion_enabled"] is True
    for suffix in ("min_x", "max_x", "half_width"):
        assert adapter[f"rear_exclusion_{suffix}"] == pytest.approx(
            mapping[f"rear_exclusion_{suffix}"]
        )


def test_georeference_uses_final_optimized_key_pose_path():
    exporter = parameters(
        PACKAGE_ROOT / "config" / "map_georeference_exporter.yaml",
        "map_georeference_exporter",
    )
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert exporter["optimized_path_topic"] == "/lio_sam/mapping/path"
    assert exporter["rtk_heading_topic"] == "/lio_sam/odometry/heading"
    assert exporter["antenna_frame"] == "rtk_master_antenna"
    lidar_from_base = transpose(rpy_matrix(mounts["lidar"]["rpy"]))
    assert exporter["lidar_to_antenna_m"] == pytest.approx(
        matvec(
            lidar_from_base,
            [
                mounts["rtk"]["xyz"][index] - mounts["lidar"]["xyz"][index]
                for index in range(3)
            ],
        ),
        abs=1.0e-8,
    )
    assert "map_odometry_topic" not in exporter
    assert exporter["maximum_horizontal_rmse_m"] == pytest.approx(0.20)
    assert exporter["maximum_yaw_rmse_deg"] == pytest.approx(2.0)
    assert exporter["require_yaw_validation"] is False

    exporter_source = (
        PACKAGE_ROOT / "src" / "map_georeference_exporter.cpp"
    ).read_text(encoding="utf-8")
    assert "Horizontal validation warning" in exporter_source
    assert (
        "horizontal georeference RMSE exceeds the acceptance limit"
        not in exporter_source
    )


def test_upstream_lio_sam_revision_is_pinned():
    repositories = yaml.safe_load(
        (PACKAGE_ROOT / "third_party.repos").read_text(encoding="utf-8")
    )
    revision = repositories["repositories"]["LIO-SAM"]["version"]

    assert revision == "08af3f32f01725372d4269838dc44c19c6d9e76b"
