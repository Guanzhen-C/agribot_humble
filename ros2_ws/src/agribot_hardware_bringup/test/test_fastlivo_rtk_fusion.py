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
    assert parameters["correction_time_constant_sec"] == pytest.approx(2.0)
    assert "HorizontalAntennaFactor" in source
    assert "Pose3AttitudeFactor" in source
    assert "georeference_horizontal_rmse_m_" in source
    assert "target_map_from_base" in source
    assert "-std::expm1(-dt / correction_time_constant_sec_)" in source
    assert "refreshLatestOptimizedPose();" in source
    assert "if (!fixedRecentlyActive())" in source
    assert '"global_correction_frozen"' in source
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
    assert 'DeclareLaunchArgument("start_rtk", default_value="true")' in launch_source
    assert '"start_rtk": LaunchConfiguration("start_rtk")' in launch_source
    assert '"start_camera", default_value="true"' in launch_source
    assert '"initialization_source",\n                default_value="rtk"' in launch_source
    assert 'executable="rtk_map_initializer"' in launch_source
    assert '"/localization/rtk_initialpose"' in launch_source
    assert '"localizer_ready_topic"' in launch_source
    assert '"auto_initialize_from_fixed_rtk": False' in launch_source
    assert (
        'DeclareLaunchArgument(\n                "auto_initialize_from_fixed_rtk"'
        not in launch_source
    )
    assert '"allow_missing_georeference"' in launch_source


def test_no_rtk_mode_keeps_manual_ndt_gicp_and_freezes_global_correction():
    launch_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_fastlivo_rtk_localization.launch.py"
    ).read_text()
    fusion_source = (
        PACKAGE_ROOT
        / "localization/fusion/src/fastlivo_rtk_fusion_node.cpp"
    ).read_text()

    assert "initialization_source" in launch_source
    assert '"manual"' in launch_source
    assert 'executable="pcd_initial_localizer"' in launch_source
    assert '"initial_pose_topic"' in launch_source
    assert '"pose_topic": "/localization_pose"' in launch_source
    assert '"seed_pose_topic", "/localization_pose"' in fusion_source
    assert '"allow_missing_georeference", false' in fusion_source
    assert "if (!fixedRecentlyActive())" in fusion_source
    assert "global_correction_frozen" in fusion_source


def test_full_vehicle_launch_uses_fused_odometry_for_navigation_and_safety():
    launch_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_mppi_fastlivo_rtk_mapped.launch.py"
    ).read_text()

    assert "ackermann_fastlivo_rtk_localization.launch.py" in launch_source
    assert 'DeclareLaunchArgument("start_rtk", default_value="true")' in launch_source
    assert '"allow_missing_georeference",\n                default_value="true"' in launch_source
    assert '"odom_topic": "/fastlivo_rtk/odometry"' in launch_source
    assert '"command_topic": "/nav2/cmd_vel"' in launch_source
    assert '"require_localization_ready": True' in launch_source
    assert '"localization_ready_topic": "/fastlivo_rtk/ready"' in launch_source
    assert '"enable_chassis_output",\n                default_value="false"' in launch_source


def test_rtk_initial_pose_uses_position_heading_and_lidar_refinement():
    initializer = (
        PACKAGE_ROOT
        / "localization/navsat/src/rtk_map_initializer.cpp"
    ).read_text()
    launch_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_fastlivo_rtk_localization.launch.py"
    ).read_text()

    assert 'required_fix_quality", 4' in initializer
    assert '"L1_INT", "NARROW_INT"' in initializer
    assert "base_to_antenna_" in initializer
    assert "map_from_enu_ * enu_to_base" in initializer
    assert "map_to_base * latest_odom_to_base_->inverse()" in initializer
    assert '"pose_topic": "/localization_pose"' in launch_source
    assert '"seed_pose_topic", "/localization_pose"' in (
        PACKAGE_ROOT
        / "localization/fusion/src/fastlivo_rtk_fusion_node.cpp"
    ).read_text()


def test_rviz_shows_fused_local_and_accepted_fixed_paths():
    parameters = load_parameters(
        "fastlivo_rtk_fusion.yaml", "fastlivo_rtk_fusion"
    )
    rviz = (
        PACKAGE_ROOT / "rviz/fastlivo_rtk_localization.rviz"
    ).read_text()
    for topic in (
        "/fastlivo_rtk/path",
        "/fastlivo_rtk/fastlivo_path",
        "/fastlivo_rtk/fixed_rtk_path",
        "/rgb_img",
    ):
        assert f"Value: {topic}" in rviz
    assert "Value: /camera/rgb/image_raw" not in rviz
    assert parameters["flatten_visualization_paths"] is True
