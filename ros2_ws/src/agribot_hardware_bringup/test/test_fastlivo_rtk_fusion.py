from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_parameters(filename, node_name):
    document = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    return document[node_name]["ros__parameters"]


def test_fixed_rtk_position_only_policy_and_gravity_constraint():
    parameters = load_parameters(
        "fastlivo_rtk_fusion.yaml", "fastlivo_rtk_fusion"
    )
    source = (
        PACKAGE_ROOT
        / "localization/fusion/src/fastlivo_rtk_fusion_node.cpp"
    ).read_text()

    assert parameters["required_fix_quality"] == 4
    assert parameters["auto_initialize_from_fixed_rtk"] is False
    assert parameters["rtk_horizontal_sigma_floor_m"] == pytest.approx(0.10)
    assert parameters["gravity_sigma_rad"] == pytest.approx(0.017453292519943295)
    assert "HorizontalAntennaFactor" in source
    assert "Pose3AttitudeFactor" in source
    assert "heading_topic" not in source
    assert "position_covariance[8]" not in source


def test_fusion_mounts_match_physical_sensor_mounts():
    fusion = load_parameters(
        "fastlivo_rtk_fusion.yaml", "fastlivo_rtk_fusion"
    )
    mounts = yaml.safe_load(
        (PACKAGE_ROOT / "config/sensor_mounts.yaml").read_text()
    )
    assert fusion["base_to_antenna_xyz"] == mounts["rtk"]["xyz"]
    assert fusion["base_from_imu_rpy"] == mounts["imu"]["rpy"]


def test_live_launch_starts_fastlivo_rtk_fusion_and_disables_localizer_tf():
    launch_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_fastlivo_rtk_localization.launch.py"
    ).read_text()
    assert 'executable="fastlivo_rtk_fusion"' in launch_source
    assert '"publish_tf": False' in launch_source
    assert '"cloud_topic": "/lidar/points"' in launch_source
    assert '"odom_topic": "/fastlivo/odometry"' in launch_source
    assert '"start_rtk": "true"' in launch_source
    assert '"start_camera", default_value="true"' in launch_source


def test_rviz_shows_fused_local_and_accepted_fixed_paths():
    rviz = (
        PACKAGE_ROOT / "rviz/fastlivo_rtk_localization.rviz"
    ).read_text()
    for topic in (
        "/fastlivo_rtk/path",
        "/fastlivo_rtk/fastlivo_path",
        "/fastlivo_rtk/fixed_rtk_path",
        "/fastlivo_rtk/pose",
    ):
        assert f"Value: {topic}" in rviz
