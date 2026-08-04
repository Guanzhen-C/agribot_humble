from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
HARDWARE_ROOT = PACKAGE_ROOT.parent / "agribot_hardware_bringup"


def parameters(path, node_name="/**"):
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node_name][
        "ros__parameters"
    ]


def test_lio_sam_translation_uses_lidar_to_imu_convention():
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )
    lio_sam = parameters(PACKAGE_ROOT / "config" / "lio_sam_c16.yaml")
    imu_to_lidar = [
        mounts["lidar"]["xyz"][index] - mounts["imu"]["xyz"][index]
        for index in range(3)
    ]

    assert lio_sam["extrinsicTrans"] == pytest.approx(
        [-value for value in imu_to_lidar]
    )
    assert lio_sam["N_SCAN"] == 16
    assert lio_sam["downsampleRate"] == 1
    assert lio_sam["loopClosureEnableFlag"] is True
    assert lio_sam["poseCovThreshold"] == pytest.approx(0.0)


def test_lio_sam_rotation_uses_calibrated_vector_and_pose_conventions():
    lio_sam = parameters(PACKAGE_ROOT / "config" / "lio_sam_c16.yaml")

    def matrix(values):
        return [
            [values[row * 3 + column] for column in range(3)]
            for row in range(3)
        ]

    vector_rotation = matrix(lio_sam["extrinsicRot"])
    pose_rotation = matrix(lio_sam["extrinsicRPY"])

    assert vector_rotation != [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    for row in range(3):
        for column in range(3):
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


def test_rtk_factors_target_the_lidar_optical_center():
    mounts = yaml.safe_load(
        (HARDWARE_ROOT / "config" / "sensor_mounts.yaml").read_text(
            encoding="utf-8"
        )
    )
    adapter = parameters(
        PACKAGE_ROOT / "config" / "rtk_odometry_adapter.yaml",
        "rtk_odometry_adapter",
    )

    assert adapter["base_to_master_antenna_m"] == pytest.approx(
        mounts["rtk"]["xyz"]
    )
    assert adapter["base_to_target_m"] == pytest.approx(mounts["lidar"]["xyz"])
    assert adapter["required_fix_quality"] == 4
    assert set(adapter["allowed_heading_solutions"]) == {
        "L1_INT",
        "NARROW_INT",
    }


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
    lio_sam = parameters(
        PACKAGE_ROOT / "config" / "lio_sam_c16.yaml", "/**"
    )

    assert exporter["optimized_path_topic"] == "/lio_sam/mapping/path"
    assert "map_odometry_topic" not in exporter
    assert exporter["maximum_horizontal_rmse_m"] == pytest.approx(0.20)
    assert exporter["maximum_yaw_rmse_deg"] == pytest.approx(2.0)
    assert lio_sam["useGpsElevation"] is False


def test_upstream_lio_sam_revision_is_pinned():
    repositories = yaml.safe_load(
        (PACKAGE_ROOT / "third_party.repos").read_text(encoding="utf-8")
    )
    revision = repositories["repositories"]["LIO-SAM"]["version"]

    assert revision == "08af3f32f01725372d4269838dc44c19c6d9e76b"
