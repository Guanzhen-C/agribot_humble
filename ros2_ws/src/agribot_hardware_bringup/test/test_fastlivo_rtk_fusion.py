from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COMMON_LOCALIZATION_LAUNCH = (
    PACKAGE_ROOT / "launch/include/fastlivo_rtk_localization.launch.py"
)
ACKERMANN_LOCALIZATION_LAUNCH = (
    PACKAGE_ROOT
    / "ackermann/launch/ackermann_fastlivo_rtk_localization.launch.py"
)
DIFFERENTIAL_LOCALIZATION_LAUNCH = (
    PACKAGE_ROOT
    / "differential/launch/differential_fastlivo_rtk_localization.launch.py"
)
DIFFERENTIAL_FULL_LAUNCH = (
    PACKAGE_ROOT
    / "differential/launch/differential_mppi_fastlivo_rtk_mapped.launch.py"
)
DIFFERENTIAL_OUTDOOR_LAUNCH = (
    PACKAGE_ROOT / "differential/launch/differential_outdoor_experiment.launch.py"
)


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
    assert parameters["preserve_local_vertical"] is True
    assert parameters["correction_time_constant_sec"] == pytest.approx(2.0)
    assert parameters["odom_ready_timeout_sec"] == pytest.approx(0.50)
    assert "HorizontalAntennaFactor" in source
    assert "Pose3AttitudeFactor" in source
    assert "georeference_horizontal_rmse_m_" in source
    assert "target_map_from_base" in source
    assert "-std::expm1(-dt / correction_time_constant_sec_)" in source
    assert "refreshLatestOptimizedPose();" in source
    assert "if (!fixedRecentlyActive())" in source
    assert "preserveLocalVertical(odom_from_base);" in source
    assert "localVerticalZ(odom_from_base)" in source
    assert "translation_delta.z() = 0.0;" in source
    assert "localizationHealthy()" in source
    assert '"odometry_fresh"' in source
    assert "std::chrono::milliseconds(100)" in source
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
    common_source = COMMON_LOCALIZATION_LAUNCH.read_text()
    wrapper_source = ACKERMANN_LOCALIZATION_LAUNCH.read_text()
    assert 'executable="fastlivo_rtk_fusion"' in common_source
    assert '"publish_tf": False' in common_source
    assert '"cloud_topic": "/lidar/points"' in common_source
    assert '"odom_topic": "/fastlivo/odometry"' in common_source
    assert 'DeclareLaunchArgument("start_rtk", default_value="true")' in wrapper_source
    assert '"start_rtk": LaunchConfiguration("start_rtk")' in wrapper_source
    assert '"start_camera", default_value="true"' in wrapper_source
    assert '"initialization_source", default_value="auto"' in wrapper_source
    assert 'executable="rtk_map_initializer"' in common_source
    assert 'DeclareLaunchArgument("fastlivo_dense_map", default_value="false")' in (
        common_source
    )
    assert '"publish.dense_map_en": ParameterValue(' in common_source
    assert (
        'DeclareLaunchArgument(\n'
        '                "fastlivo_map_sliding_en", default_value="true"'
        in common_source
    )
    assert '"local_map.map_sliding_en": ParameterValue(' in common_source
    assert 'DeclareLaunchArgument("fastlivo_dense_map", default_value="false")' in (
        wrapper_source
    )
    assert '"fastlivo_dense_map": LaunchConfiguration(' in wrapper_source
    assert '"fastlivo_map_sliding_en": LaunchConfiguration(' in wrapper_source
    assert 'executable="visual_place_recognizer.py"' in common_source
    assert 'executable="initialization_coordinator.py"' in common_source
    assert '"initial_pose_topic": PythonExpression(' in common_source
    assert "/localization/initialpose_prior" in common_source
    assert '"/localization/attempt_result"' in (
        PACKAGE_ROOT / "localization/pcd/src/pcd_initial_localizer.cpp"
    ).read_text()
    assert '"/localization/rtk_initialpose"' in common_source
    assert '"localizer_ready_topic"' in common_source
    assert '"auto_initialize_from_fixed_rtk": False' in common_source
    assert (
        'DeclareLaunchArgument(\n                "auto_initialize_from_fixed_rtk"'
        not in common_source
    )
    assert '"allow_missing_georeference"' in common_source
    assert "fastlivo_rtk_localization.launch.py" in wrapper_source
    # Delayed actions lose child launch configurations after a scoped include.
    # These nodes already wait for their input topics, so they must start directly.
    assert "TimerAction" not in common_source


def test_differential_outdoor_restores_rtk_visual_manual_priority():
    localization_source = DIFFERENTIAL_LOCALIZATION_LAUNCH.read_text()
    full_source = DIFFERENTIAL_FULL_LAUNCH.read_text()
    outdoor_source = DIFFERENTIAL_OUTDOOR_LAUNCH.read_text()

    for source in (localization_source, full_source):
        assert '"enable_rtk_initialization": LaunchConfiguration(' in source
        assert '"enable_visual_initialization": LaunchConfiguration(' in source
        assert '"visual_database_file": LaunchConfiguration(' in source
    assert '"initialization_source": "auto"' in outdoor_source
    assert 'DeclareLaunchArgument("initialization_source", default_value="auto")' in (
        outdoor_source
    )
    assert '"enable_rtk_initialization": "true"' in outdoor_source
    assert '"enable_visual_initialization": "true"' in outdoor_source


def test_differential_navigation_uses_continuous_control_behavior_trees():
    source = DIFFERENTIAL_FULL_LAUNCH.read_text()

    assert (
        "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml"
        in source
    )
    assert "navigate_through_poses_w_replanning_differential.xml" in source
    assert '"localization_ready_timeout_sec": 1.0' in source


def test_no_rtk_mode_keeps_manual_ndt_gicp_and_freezes_global_correction():
    launch_source = COMMON_LOCALIZATION_LAUNCH.read_text()
    fusion_source = (
        PACKAGE_ROOT
        / "localization/fusion/src/fastlivo_rtk_fusion_node.cpp"
    ).read_text()

    assert "initialization_source" in launch_source
    assert '"manual"' in launch_source
    assert '"auto"' in launch_source
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
    assert '"localization_ready_timeout_sec": 0.5' in launch_source
    assert '"enable_chassis_output",\n                default_value="false"' in launch_source
    assert 'LaunchConfiguration("rviz_config")' in launch_source


def test_nested_launches_cannot_override_parent_rviz_selection():
    localization_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_fastlivo_rtk_localization.launch.py"
    ).read_text()
    vehicle_source = (
        PACKAGE_ROOT
        / "ackermann/launch/ackermann_mppi_fastlivo_rtk_mapped.launch.py"
    ).read_text()

    assert "GroupAction(\n                scoped=True" in localization_source
    assert "GroupAction(\n                scoped=True" in vehicle_source
    assert '"rviz": LaunchConfiguration("rviz")' in localization_source
    assert '"rviz": "false"' in vehicle_source


def test_rtk_initial_pose_uses_position_heading_and_lidar_refinement():
    initializer = (
        PACKAGE_ROOT
        / "localization/navsat/src/rtk_map_initializer.cpp"
    ).read_text()
    launch_source = COMMON_LOCALIZATION_LAUNCH.read_text()

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
