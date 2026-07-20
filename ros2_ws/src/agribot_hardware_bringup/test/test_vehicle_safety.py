import importlib.util
import math
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


PACKAGE_ROOT = Path(__file__).parents[1]


def load_script(name):
    path = PACKAGE_ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load_script("vehicle_command_gate.py")
PREFLIGHT = load_script("vehicle_preflight.py")


def test_command_clamping_and_invalid_values():
    assert GATE.clamp_command(1.2, -0.9, 0.8, 0.65) == (0.8, -0.65)
    assert GATE.clamp_command(-0.2, 0.3, 0.8, 0.65) == (-0.2, 0.3)
    with pytest.raises(ValueError):
        GATE.clamp_command(math.nan, 0.0, 0.8, 0.65)
    with pytest.raises(ValueError):
        GATE.clamp_command(0.0, 0.0, 0.0, 0.65)


def test_quaternion_validation():
    assert PREFLIGHT.quaternion_is_valid(0.0, 0.0, 0.0, 1.0)
    assert not PREFLIGHT.quaternion_is_valid(0.0, 0.0, 0.0, 0.0)
    assert not PREFLIGHT.quaternion_is_valid(math.nan, 0.0, 0.0, 1.0)


def test_missing_can_interface_is_not_up():
    assert not PREFLIGHT.interface_is_up("definitely_missing")


def test_gate_requires_preflight_enable_and_clear_estop():
    rclpy.init()
    node = GATE.VehicleCommandGate(
        parameter_overrides=[
            Parameter("require_preflight", value=True),
            Parameter("initially_enabled", value=True),
            Parameter("input_timeout_sec", value=1.0),
        ]
    )
    try:
        command = Twist()
        command.linear.x = 0.4
        node.handle_command(command)
        assert not node.output_is_active()

        node.handle_preflight(Bool(data=True))
        assert node.output_is_active()

        node.handle_e_stop(Bool(data=True))
        assert not node.output_is_active()

        node.handle_e_stop(Bool(data=False))
        node.handle_drive_enable(Bool(data=False))
        assert not node.output_is_active()
    finally:
        node.destroy_node()
        rclpy.shutdown()
