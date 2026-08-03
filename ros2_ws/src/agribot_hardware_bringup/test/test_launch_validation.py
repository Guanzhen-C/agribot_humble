import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
VEHICLE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "vehicle_autonomy.launch.py"
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


def load_vehicle_launch():
    spec = importlib.util.spec_from_file_location(
        "vehicle_autonomy_launch", VEHICLE_LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LAUNCH = load_vehicle_launch()


def context_with(**overrides):
    values = {
        "localization": "navsat",
        "navigation_mode": "static",
        "vehicle_type": "differential",
        "controller": "dwb",
        "chassis_driver": "differential_can",
        "can_transport": "socketcan",
        "enable_can_output": "false",
        "enable_chassis_output": "false",
        "use_detailed_vehicle_model": "false",
        "map": "/tmp/real_map.yaml",
        "pcd_map_base": "/tmp/real_map",
        "pcd_map_file": "/tmp/real_map.pcd",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_valid_differential_selection():
    assert LAUNCH._validate_arguments(context_with()) == []


def test_differential_rejects_mppi():
    with pytest.raises(RuntimeError, match="requires controller:=dwb"):
        LAUNCH._validate_arguments(context_with(controller="mppi"))


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


def test_mapped_entry_uses_nav2_and_pcd_maps_with_optional_fpfh_localization():
    source = ACKERMANN_LAUNCH_PATHS[1].read_text()
    assert '"navigation_mode": "localization"' in source
    assert '"map": PythonExpression(' in source
    assert "map_base, \".yaml'\"]" in source
    assert '"pcd_map_file": PythonExpression(' in source
    assert "map_base, \".pcd'\"]" in source
    assert '"require_localization_ready": "true"' in source
    assert 'DeclareLaunchArgument("enable_fpfh", default_value="false")' in source
    assert '"enable_fpfh": LaunchConfiguration("enable_fpfh")' in source
    assert "initial_pose" not in source
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
    assert (
        'condition=LaunchConfigurationEquals("navigation_mode", "static")'
        in source
    )


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
