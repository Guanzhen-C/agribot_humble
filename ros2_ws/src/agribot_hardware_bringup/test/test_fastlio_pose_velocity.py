import importlib.util
from pathlib import Path

import numpy as np
import pytest


PATH = Path(__file__).resolve().parents[1] / "localization/fastlio/scripts/fastlio_odom_bridge.py"
spec = importlib.util.spec_from_file_location("fastlio_pose_velocity", PATH)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_velocity_is_in_current_base_frame_not_map_frame():
    previous = bridge.compose_matrix([0, 0, 0], [0, 0, np.pi / 2])
    current = bridge.compose_matrix([0, .03, .01], [0, 0, np.pi / 2])
    linear, angular = bridge.pose_delta_twist(previous, current, .1)
    assert linear == pytest.approx([.3, 0, .1])
    assert angular == pytest.approx([0, 0, 0])


def test_rotation_wrap_does_not_produce_a_full_turn():
    previous = bridge.compose_matrix([0, 0, 0], [0, 0, np.pi - .01])
    current = bridge.compose_matrix([0, 0, 0], [0, 0, -np.pi + .01])
    linear, angular = bridge.pose_delta_twist(previous, current, .1)
    assert linear == pytest.approx([0, 0, 0])
    assert angular == pytest.approx([0, 0, .2])


@pytest.mark.parametrize("dt", [0, -1, .00001, .6, float("nan")])
def test_clock_jump_gap_and_duplicate_stamp_reset_velocity(dt):
    linear, angular = bridge.pose_delta_twist(np.eye(4), np.eye(4), dt)
    assert linear == pytest.approx([0, 0, 0])
    assert angular == pytest.approx([0, 0, 0])


def test_imu_lever_arm_is_removed_before_differentiation():
    base_to_imu = bridge.compose_matrix([1, 0, 1], [0, 0, 0])
    old_base = np.eye(4)
    new_base = bridge.compose_matrix([0, 0, 0], [.01, -.02, .03])
    old_imu = old_base @ base_to_imu
    new_imu = new_base @ base_to_imu
    linear, angular = bridge.pose_delta_twist(old_imu @ np.linalg.inv(base_to_imu),
                                             new_imu @ np.linalg.inv(base_to_imu), .1)
    assert linear == pytest.approx([0, 0, 0], abs=1e-10)
    assert np.linalg.norm(angular) > .3
