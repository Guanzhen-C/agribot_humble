import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
VEHICLE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "vehicle_autonomy.launch.py"
ACKERMANN_LAUNCH_PATHS = (
    PACKAGE_ROOT / "ackermann" / "launch" / "ackermann_mppi_navsat.launch.py",
    PACKAGE_ROOT / "ackermann" / "launch" / "ackermann_mppi_fastlio.launch.py",
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlio_local.launch.py",
    PACKAGE_ROOT
    / "ackermann"
    / "launch"
    / "ackermann_mppi_fastlio_mapping.launch.py",
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
        "map": "/tmp/real_map.yaml",
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


def test_static_navigation_requires_map():
    with pytest.raises(RuntimeError, match="static navigation requires map"):
        LAUNCH._validate_arguments(context_with(map=""))


def test_unknown_navigation_mode_is_rejected():
    with pytest.raises(RuntimeError, match="navigation_mode must be"):
        LAUNCH._validate_arguments(context_with(navigation_mode="unknown"))


def test_chassis_uses_nav2_controller_output_without_manual_gate():
    source = VEHICLE_LAUNCH_PATH.read_text()
    assert "vehicle_command_gate" not in source
    assert "vehicle_preflight" not in source
    assert "nav2_collision_monitor" not in source
    assert 'default_value="/nav2/cmd_vel"' in source
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
