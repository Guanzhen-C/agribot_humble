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
        "vehicle_type": "differential",
        "controller": "dwb",
        "chassis_driver": "differential_can",
        "enable_can_output": "false",
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
        enable_can_output="true",
    )
    assert LAUNCH._validate_arguments(context) == []


def test_removed_simulated_driver_is_rejected():
    context = context_with(
        vehicle_type="ackermann",
        controller="mppi",
        chassis_driver="simulated",
        enable_can_output="true",
    )
    with pytest.raises(RuntimeError, match="chassis_driver must be"):
        LAUNCH._validate_arguments(context)
