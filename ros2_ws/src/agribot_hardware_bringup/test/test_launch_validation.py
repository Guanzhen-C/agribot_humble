import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]


def load_vehicle_launch():
    path = PACKAGE_ROOT / "launch" / "vehicle_autonomy.launch.py"
    spec = importlib.util.spec_from_file_location("vehicle_autonomy_launch", path)
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


def test_static_navigation_requires_map():
    with pytest.raises(RuntimeError, match="static navigation requires map"):
        LAUNCH._validate_arguments(context_with(map=""))


def test_unknown_navigation_mode_is_rejected():
    with pytest.raises(RuntimeError, match="navigation_mode must be"):
        LAUNCH._validate_arguments(context_with(navigation_mode="unknown"))
