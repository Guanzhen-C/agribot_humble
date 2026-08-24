import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
VEHICLE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "vehicle_autonomy.launch.py"
NAVSAT_LAUNCH_PATH = (
    PACKAGE_ROOT / "ackermann" / "launch" / "ackermann_mppi_navsat.launch.py"
)
ACKERMANN_LAUNCH_PATHS = (
    PACKAGE_ROOT / "ackermann" / "launch" / "ackermann_mppi_navsat.launch.py",
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlio_mapped.launch.py",
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlio_local.launch.py",
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlio_3d_mapping.launch.py",
)
SENSOR_COLLECTION_LAUNCH_PATH = (
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_sensor_data_collection.launch.py"
)
SENSOR_COLLECTION_QOS_PATH = (
    PACKAGE_ROOT
    / "ackermann"
    / "config"
    / "sensor_data_recording_qos.yaml"
)
GEOREFERENCE_VALIDATION_LAUNCH_PATH = (
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_georeference_validation.launch.py"
)
DIFFERENTIAL_FULL_LAUNCH_PATH = (
    PACKAGE_ROOT
    / "differential"
    / "launch"
    / "differential_mppi_fastlivo_rtk_mapped.launch.py"
)
ACKERMANN_FASTLIVO_FULL_LAUNCH_PATH = (
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlivo_rtk_mapped.launch.py"
)


def load_vehicle_launch():
    spec = importlib.util.spec_from_file_location(
        "vehicle_autonomy_launch", VEHICLE_LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LAUNCH = load_vehicle_launch()


def load_launch(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


NAVSAT_LAUNCH = load_launch(NAVSAT_LAUNCH_PATH, "ackermann_mppi_navsat_launch")
DIFFERENTIAL_FULL_LAUNCH = load_launch(
    DIFFERENTIAL_FULL_LAUNCH_PATH,
    "differential_mppi_fastlivo_rtk_mapped_launch",
)
ACKERMANN_FASTLIVO_FULL_LAUNCH = load_launch(
    ACKERMANN_FASTLIVO_FULL_LAUNCH_PATH,
    "ackermann_mppi_fastlivo_rtk_mapped_launch",
)


def context_with(**overrides):
    values = {
        "localization": "navsat",
        "navigation_mode": "static",
        "vehicle_type": "differential",
        "controller": "mppi",
        "chassis_driver": "differential_can",
        "can_transport": "socketcan",
        "enable_can_output": "false",
        "enable_chassis_output": "false",
        "use_detailed_vehicle_model": "false",
        "map": "/tmp/real_map.yaml",
        "pcd_map_base": "/tmp/real_map",
        "pcd_map_file": "/tmp/real_map.pcd",
        "initialization_source": "manual",
        "map_georeference_file": "",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def differential_full_context(**overrides):
    values = {
        "chassis_driver": "differential_can",
        "can_transport": "socketcan",
        "enable_chassis_output": "false",
        "allow_uncalibrated_camera": "false",
        "use_sim_time": "false",
        "motion_authorization": "",
        "vehicle_calibration": str(
            PACKAGE_ROOT / "differential" / "config" / "vehicle_calibration.yaml"
        ),
        "zqwl_port": "/dev/does-not-exist",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def ackermann_fastlivo_full_context(**overrides):
    values = {
        "chassis_driver": "ackermann_can",
        "can_transport": "socketcan",
        "enable_chassis_output": "false",
        "allow_uncalibrated_camera": "false",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_valid_differential_selection():
    assert LAUNCH._validate_arguments(context_with()) == []


def test_differential_rejects_removed_dwb_controller():
    with pytest.raises(RuntimeError, match="legacy DWB configuration has been removed"):
        LAUNCH._validate_arguments(context_with(controller="dwb"))


def test_differential_full_stack_is_read_only_by_default():
    assert DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
        differential_full_context()
    ) == []


def test_uncalibrated_camera_is_allowed_only_without_chassis_output():
    assert ACKERMANN_FASTLIVO_FULL_LAUNCH._validate_arguments(
        ackermann_fastlivo_full_context(allow_uncalibrated_camera="true")
    ) == []
    with pytest.raises(RuntimeError, match="禁止绕过"):
        ACKERMANN_FASTLIVO_FULL_LAUNCH._validate_arguments(
            ackermann_fastlivo_full_context(
                enable_chassis_output="true",
                allow_uncalibrated_camera="true",
            )
        )
    with pytest.raises(RuntimeError, match="禁止绕过"):
        DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
            differential_full_context(
                enable_chassis_output="true",
                allow_uncalibrated_camera="true",
            )
        )


def test_differential_full_stack_requires_explicit_motion_authorization():
    with pytest.raises(RuntimeError, match="motion_authorization"):
        DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
            differential_full_context(enable_chassis_output="true")
        )


def test_differential_full_stack_rejects_provisional_calibration():
    with pytest.raises(RuntimeError, match="尚未标定"):
        DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
            differential_full_context(
                enable_chassis_output="true",
                motion_authorization=(
                    DIFFERENTIAL_FULL_LAUNCH.MOTION_AUTHORIZATION
                ),
            )
        )


def test_differential_full_stack_accepts_completed_calibration(tmp_path):
    calibration = tmp_path / "vehicle_calibration.yaml"
    calibration.write_text(
        "vehicle_type: differential\ncalibration_complete: true\n"
    )
    assert DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
        differential_full_context(
            enable_chassis_output="true",
            motion_authorization=DIFFERENTIAL_FULL_LAUNCH.MOTION_AUTHORIZATION,
            vehicle_calibration=str(calibration),
        )
    ) == []


def test_differential_full_stack_rejects_simulated_time_for_motion(tmp_path):
    calibration = tmp_path / "vehicle_calibration.yaml"
    calibration.write_text(
        "vehicle_type: differential\ncalibration_complete: true\n"
    )
    with pytest.raises(RuntimeError, match="use_sim_time"):
        DIFFERENTIAL_FULL_LAUNCH._validate_arguments(
            differential_full_context(
                enable_chassis_output="true",
                use_sim_time="true",
                motion_authorization=(
                    DIFFERENTIAL_FULL_LAUNCH.MOTION_AUTHORIZATION
                ),
                vehicle_calibration=str(calibration),
            )
        )


def test_ackermann_can_accepts_verified_driver():
    context = context_with(
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_can",
        enable_chassis_output="true",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_unknown_can_transport_is_rejected():
    with pytest.raises(RuntimeError, match="can_transport must be"):
        LAUNCH._validate_arguments(context_with(can_transport="unknown"))


def test_ackermann_serial_accepts_verified_driver():
    context = context_with(
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_serial",
        enable_chassis_output="true",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_removed_simulated_driver_is_rejected():
    context = context_with(
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="simulated",
        enable_chassis_output="true",
    )
    with pytest.raises(RuntimeError, match="chassis_driver must be"):
        LAUNCH._validate_arguments(context)


def test_local_navigation_accepts_fastlio_without_map():
    context = context_with(
        localization="fastlio",
        navigation_mode="local",
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_serial",
        map="",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_local_navigation_rejects_navsat():
    with pytest.raises(RuntimeError, match="requires localization:=fastlio"):
        LAUNCH._validate_arguments(
            context_with(localization="navsat", navigation_mode="local", map="")
        )


def test_mapping_navigation_accepts_fastlio_without_map():
    context = context_with(
        localization="fastlio",
        navigation_mode="mapping",
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_can",
        map="",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_mapping_navigation_rejects_navsat():
    with pytest.raises(RuntimeError, match="requires localization:=fastlio"):
        LAUNCH._validate_arguments(
            context_with(localization="navsat", navigation_mode="mapping", map="")
        )


def test_mapping_navigation_requires_output_base_path():
    with pytest.raises(RuntimeError, match="3D mapping requires pcd_map_base"):
        LAUNCH._validate_arguments(
            context_with(
                localization="fastlio",
                navigation_mode="mapping",
                vehicle_type="ackermann",
                controller="mppi",
                map="",
                pcd_map_base="",
            )
        )


def test_mapped_localization_accepts_fastlio():
    context = context_with(
        localization="fastlio",
        navigation_mode="localization",
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_can",
        map="/tmp/real_map.yaml",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_mapped_localization_accepts_navsat():
    context = context_with(
        localization="navsat",
        navigation_mode="localization",
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="ackermann_can",
        map="/tmp/real_map.yaml",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_mapped_localization_requires_nav2_map():
    with pytest.raises(RuntimeError, match="localization navigation requires map"):
        LAUNCH._validate_arguments(
            context_with(
                localization="fastlio",
                navigation_mode="localization",
                vehicle_type="ackermann",
                controller="mppi",
                map="",
            )
        )


def test_mapped_localization_requires_pcd_map():
    with pytest.raises(RuntimeError, match="pcd_map_file"):
        LAUNCH._validate_arguments(
            context_with(
                localization="fastlio",
                navigation_mode="localization",
                vehicle_type="ackermann",
                controller="mppi",
                pcd_map_file="",
            )
        )


def test_mapped_rtk_initialization_requires_georeference():
    with pytest.raises(RuntimeError, match="requires map_georeference_file"):
        LAUNCH._validate_arguments(
            context_with(
                localization="fastlio",
                navigation_mode="localization",
                vehicle_type="ackermann",
                controller="mppi",
                initialization_source="rtk",
                map_georeference_file="",
            )
        )


def test_mapped_rtk_initialization_accepts_georeference():
    assert LAUNCH._validate_arguments(
        context_with(
            localization="fastlio",
            navigation_mode="localization",
            vehicle_type="ackermann",
            controller="mppi",
            initialization_source="rtk",
            map_georeference_file="/tmp/real_map_georeference.yaml",
        )
    ) == []


def test_unknown_initialization_source_is_rejected():
    with pytest.raises(RuntimeError, match="initialization_source must be"):
        LAUNCH._validate_arguments(
            context_with(initialization_source="unsupported")
        )


def test_removed_ackermann_fastlio_static_mode_is_rejected():
    with pytest.raises(RuntimeError, match="static mode was removed"):
        LAUNCH._validate_arguments(
            context_with(
                localization="fastlio",
                navigation_mode="static",
                vehicle_type="ackermann",
                controller="mppi",
            )
        )


def test_static_navigation_requires_map():
    with pytest.raises(RuntimeError, match="static navigation requires map"):
        LAUNCH._validate_arguments(context_with(map=""))


def test_unknown_navigation_mode_is_rejected():
    with pytest.raises(RuntimeError, match="navigation_mode must be"):
        LAUNCH._validate_arguments(context_with(navigation_mode="unknown"))


def write_test_georeference(
    tmp_path, fingerprint, yaw_validation_passed=True
):
    georeference = tmp_path / "map_georeference.yaml"
    georeference.write_text(
        "\n".join(
            [
                "schema_version: 1",
                "map:",
                "  id: map",
                f"  fingerprint_fnv1a64: {fingerprint}",
                "reference:",
                "  latitude_deg: 30.0",
                "  longitude_deg: 114.0",
                "  altitude_m: 20.0",
                "map_from_enu:",
                "  xyz: [1.0, 2.0, 0.0]",
                "  rpy: [0.0, 0.0, 0.1]",
                "calibration:",
                "  horizontal_rmse_m: 0.05",
                "  yaw_rmse_deg: 0.5",
                "  yaw_validation_passed: "
                f"{'true' if yaw_validation_passed else 'false'}",
                "  sample_count: 20",
                "  version: test-v1",
                "  hash: test-hash",
            ]
        )
    )
    return georeference


def test_navsat_entry_accepts_matching_pcd_fingerprint(tmp_path):
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\n")
    pcd_path = tmp_path / "map.pcd"
    pcd_path.write_bytes(b"optimized-map")
    georeference = write_test_georeference(
        tmp_path, NAVSAT_LAUNCH._fingerprint_file(pcd_path)
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {"map": str(map_path), "map_georeference": str(georeference)}
    )

    actions = NAVSAT_LAUNCH._launch_georeferenced_navsat(
        context, hardware_share=str(PACKAGE_ROOT)
    )

    assert len(actions) == 1


def test_navsat_entry_rejects_mismatched_pcd_fingerprint(tmp_path):
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\n")
    (tmp_path / "map.pcd").write_bytes(b"optimized-map")
    georeference = write_test_georeference(tmp_path, "0000000000000000")
    context = LaunchContext()
    context.launch_configurations.update(
        {"map": str(map_path), "map_georeference": str(georeference)}
    )

    with pytest.raises(RuntimeError, match="fingerprint"):
        NAVSAT_LAUNCH._launch_georeferenced_navsat(
            context, hardware_share=str(PACKAGE_ROOT)
        )


def test_navsat_entry_accepts_position_only_yaw_for_lidar_refinement(tmp_path):
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\n")
    pcd_path = tmp_path / "map.pcd"
    pcd_path.write_bytes(b"optimized-map")
    georeference = write_test_georeference(
        tmp_path,
        NAVSAT_LAUNCH._fingerprint_file(pcd_path),
        yaw_validation_passed=False,
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {"map": str(map_path), "map_georeference": str(georeference)}
    )

    actions = NAVSAT_LAUNCH._launch_georeferenced_navsat(
        context, hardware_share=str(PACKAGE_ROOT)
    )

    assert len(actions) == 1


def test_navsat_entry_uses_kf_gins_with_one_shot_pcd_refinement():
    source = NAVSAT_LAUNCH_PATH.read_text()
    assert '"localization": "navsat"' in source
    assert '"navigation_mode": "localization"' in source
    assert '"pcd_map_file": str(pcd_path)' in source
    assert '"mapped_odometry_topic": "/odometry/filtered_navsat"' in source
    assert '"navsat_output_frame": "odom"' in source
    assert '"navsat_tf_mode": "odom_to_base_only"' in source
    assert '"navsat_ready_topic": "/localization/navsat_ready"' in source
    assert '"require_localization_ready": "true"' in source
    assert '"enable_fpfh": "false"' in source


def test_chassis_uses_nav2_output_with_only_localization_readiness_inhibition():
    source = VEHICLE_LAUNCH_PATH.read_text()
    assert "vehicle_command_gate" not in source
    assert "vehicle_preflight" not in source
    assert "nav2_collision_monitor" not in source
    assert 'default_value="/nav2/cmd_vel"' in source
    assert source.count(
        '"require_localization_ready": LaunchConfiguration('
    ) == 3
    assert source.count(
        '"command_topic": LaunchConfiguration(\n'
        '                                    "command_input_topic"\n'
        "                                ),"
    ) == 3


def test_fastlio_local_ackermann_uses_mppi_command_directly():
    source = ACKERMANN_LAUNCH_PATHS[2].read_text()
    assert (
        'DeclareLaunchArgument(\n'
        '                "command_input_topic", default_value="/nav2/cmd_vel"\n'
        "            )"
    ) in source


def test_mapping_entry_uses_mapped_config_without_owning_chassis_by_default():
    source = ACKERMANN_LAUNCH_PATHS[3].read_text()
    assert '"navigation_mode": "mapping"' in source
    assert '"start_navigation": "false"' in source
    assert '"pcd_map_base": LaunchConfiguration("map_base")' in source
    assert "pcd_mapping.rviz" in source
    assert 'DeclareLaunchArgument("map_start_delay", default_value="5.0")' in source
    assert 'DeclareLaunchArgument("enable_chassis_output", default_value="false")' in source


def test_sensor_collection_records_raw_sensor_and_camera_data_only():
    source = SENSOR_COLLECTION_LAUNCH_PATH.read_text()
    assert '"launch", "sensors.launch.py"' in source
    assert '"launch", "include", "right_camera.launch.py"' in source
    assert 'DeclareLaunchArgument("camera_driver", default_value="hikrobot_mvs")' in source
    assert '"hikrobot_camera_serial", default_value="DB0447659"' in source
    assert 'default_value="/dev/agribot_right_camera"' in source
    assert 'DeclareLaunchArgument("start_rtk", default_value="true")' in source
    assert 'DeclareLaunchArgument("start_camera", default_value="true")' in source
    assert 'DeclareLaunchArgument("record_bag", default_value="true")' in source
    assert '"bag_max_cache_size", default_value="536870912"' in source
    assert '"bag_qos_overrides", default_value=recording_qos' in source
    assert '"--max-cache-size"' in source
    assert '"--qos-profile-overrides-path"' in source
    for topic in (
        "/lidar/points",
        "/lslidar_device_info",
        "/time_topic",
        "/imu/data",
        "/imu/magnetic_field",
        "/imu/temperature",
        "/rtk/raw_sentence",
        "/rtk/fix",
        "/rtk/fix_quality",
        "/rtk/gga_utc",
        "/rtk/satellite_count",
        "/rtk/hdop",
        "/rtk/differential_age",
        "/rtk/reference_station_id",
        "/rtk/heading_with_covariance",
        "/camera/rgb/image_raw",
        "/camera/rgb/camera_info",
        "/tf_static",
    ):
        assert f'"{topic}"' in source
    assert "fast_lio" not in source.lower()
    assert "vehicle_autonomy" not in source
    assert "pcd_map_builder" not in source
    assert "ackermann_chassis_can_node" not in source
    assert "/teleop/cmd_vel" not in source
    qos = SENSOR_COLLECTION_QOS_PATH.read_text()
    for topic in ("/camera/rgb/image_raw", "/camera/rgb/camera_info"):
        assert f"{topic}:" in qos
    assert qos.count("reliability: reliable") == 2
    assert qos.count("depth: 16") == 2
    manifest = (PACKAGE_ROOT / "package.xml").read_text()
    assert "<exec_depend>hikrobot_mvs_ros2</exec_depend>" in manifest
    assert "<exec_depend>usb_cam</exec_depend>" in manifest
    assert "<exec_depend>openni2_camera</exec_depend>" not in manifest


def test_georeference_validation_reuses_production_localizer_without_motion_nodes():
    source = GEOREFERENCE_VALIDATION_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"start_navigation": "false"' in source
    assert '"enable_chassis_output": "false"' in source
    assert '"chassis_driver": "none"' in source
    assert '"start_rtk": "false"' in source
    assert '"mapped_initial_pose_topic": test_initial_pose_topic' in source
    assert 'executable="georeference_test_bridge"' in source
    assert '"base_to_master_antenna_m"' in source
    assert 'rtk_xyz = mounts["rtk"]["xyz"]' in source
    assert "ackermann_chassis_can_node" not in source
    assert "ackermann_chassis_serial_node" not in source


def test_mapped_entry_uses_nav2_and_pcd_maps_with_optional_fpfh_localization():
    source = ACKERMANN_LAUNCH_PATHS[1].read_text()
    assert '"navigation_mode": "localization"' in source
    assert '"map": PythonExpression(' in source
    assert "map_base, \".yaml'\"]" in source
    assert '"pcd_map_file": PythonExpression(' in source
    assert "map_base, \".pcd'\"]" in source
    assert '"require_localization_ready": "true"' in source
    assert 'DeclareLaunchArgument("enable_fpfh", default_value="false")' in source
    assert '"initialization_source": LaunchConfiguration(' in source
    assert '"mapped_initial_pose_topic": PythonExpression(' in source
    assert '"enable_fpfh": PythonExpression(' in source
    assert '"automatic_global_localization": PythonExpression(' in source
    assert '"map_georeference_file": LaunchConfiguration(' in source
    assert "nav2_params_ackermann_fastlio_mapped.yaml" in source
    assert 'DeclareLaunchArgument("map_start_delay", default_value="5.0")' in source
    assert "posegraph" not in source


def test_vehicle_launch_uses_one_shot_pcl_localization_only_in_mapped_mode():
    source = VEHICLE_LAUNCH_PATH.read_text()
    assert 'executable="fastlio_map_anchor.py"' not in source
    assert 'executable="pcd_initial_localizer"' in source
    assert '"map_file_path": LaunchConfiguration("pcd_map_file")' in source
    assert 'pointcloud_tf_adapter' not in source
    assert 'mrpt_map_server' not in source
    assert 'mrpt_pf_localization' not in source
    assert 'executable="pcd_map_builder"' in source
    assert "slam_toolbox" not in source
    assert "pointcloud_to_laserscan" not in source
    assert "pcd_ndt_localizer" not in source
    assert "map_to_fastlio_odom" in source
    assert "odom_to_fastlio_world" in source
    assert '"cloud_topic": "/lidar/points"' in source
    assert '"cloud_frame": "lidar_link"' in source
    assert '"odom_topic": "/odometry/filtered_navsat"' in source
    assert '"base_to_body_xyz": [0.48, 0.0, 0.233]' in source
    assert '"external_ready_topic": LaunchConfiguration(' in source
    assert (
        'condition=LaunchConfigurationEquals("navigation_mode", "static")'
        in source
    )


def test_navsat_bridge_can_leave_map_to_odom_to_pcd_localizer():
    source = (
        PACKAGE_ROOT
        / "localization"
        / "navsat"
        / "scripts"
        / "navsat_pose_bridge.py"
    ).read_text()
    assert '"odom_to_base_only"' in source
    assert 'if self.tf_mode == "odom_to_base_only":' in source


def test_ackermann_vehicle_launch_publishes_robot_description():
    source = VEHICLE_LAUNCH_PATH.read_text()
    assert 'package="robot_state_publisher"' in source
    assert 'name="ackermann_robot_state_publisher"' in source
    assert '"urdf", "ackermann_vehicle.urdf"' in source
    assert '"robot_description": robot_description' in source
    assert 'LaunchConfiguration("vehicle_type").perform(context)' in source


def test_detailed_ackermann_vehicle_model_is_opt_in():
    disabled_context = context_with(
        vehicle_type="ackermann", use_detailed_vehicle_model="false"
    )
    assert LAUNCH._launch_ackermann_robot_state_publisher(
        disabled_context,
        hardware_share=str(PACKAGE_ROOT),
        use_sim_time=False,
    ) == []

    enabled_context = context_with(
        vehicle_type="ackermann", use_detailed_vehicle_model="true"
    )
    actions = LAUNCH._launch_ackermann_robot_state_publisher(
        enabled_context,
        hardware_share=str(PACKAGE_ROOT),
        use_sim_time=False,
    )
    assert len(actions) == 1


def test_ackermann_entry_points_expose_optional_detailed_vehicle_model():
    for launch_path in ACKERMANN_LAUNCH_PATHS:
        source = launch_path.read_text()
        assert (
            '"use_detailed_vehicle_model", default_value="false"' in source
        )
        assert (
            '"use_detailed_vehicle_model": LaunchConfiguration(' in source
        )


def test_sensor_launch_is_scoped_to_preserve_parent_rviz_selection():
    source = VEHICLE_LAUNCH_PATH.read_text()
    sensor_block = source[source.index("    sensors = GroupAction("):]
    sensor_block = sensor_block[:sensor_block.index("    navsat_localization")]
    assert "scoped=True" in sensor_block
    assert '"rviz": "false"' in sensor_block


def test_ackermann_entry_points_default_to_verified_can_transport():
    for launch_path in ACKERMANN_LAUNCH_PATHS:
        source = launch_path.read_text()
        assert (
            'DeclareLaunchArgument("chassis_driver", '
            'default_value="ackermann_can")'
        ) in source
        assert (
            'DeclareLaunchArgument("can_transport", default_value="zqwl_cdc")'
            in source
        )
        assert 'DeclareLaunchArgument("can_interface", default_value="can0")' in source
        assert "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00" in source
        assert 'DeclareLaunchArgument("zqwl_channel", default_value="0")' in source
        assert (
            'DeclareLaunchArgument("zqwl_bitrate", default_value="1000000")'
            in source
        )
