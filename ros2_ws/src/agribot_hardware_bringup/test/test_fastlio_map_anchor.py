import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / "localization"
    / "fastlio"
    / "scripts"
    / "fastlio_map_anchor.py"
)
SPEC = importlib.util.spec_from_file_location("fastlio_map_anchor", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_configured_pose_accepts_planar_and_six_dof_values():
    planar = MODULE.configured_pose([2.0, -1.0, math.pi / 2.0])
    assert planar[:3, 3] == pytest.approx([2.0, -1.0, 0.0])
    assert planar[:3, :3] @ [1.0, 0.0, 0.0] == pytest.approx(
        [0.0, 1.0, 0.0]
    )

    six_dof = MODULE.configured_pose([1.0, 2.0, 3.0, 0.1, -0.2, 0.3])
    assert six_dof[:3, 3] == pytest.approx([1.0, 2.0, 3.0])
    assert np.linalg.det(six_dof[:3, :3]) == pytest.approx(1.0)


def test_configured_pose_rejects_ambiguous_parameter_length():
    with pytest.raises(ValueError, match="initial_pose"):
        MODULE.configured_pose([0.0, 0.0])


def test_quaternion_matrix_round_trip_preserves_rotation():
    original = MODULE.configured_pose([0.0, 0.0, 0.0, 0.4, -0.3, 1.2])
    quaternion = MODULE.quaternion_from_matrix(original)
    recovered = MODULE.quaternion_matrix(quaternion)
    assert recovered[:3, :3] == pytest.approx(original[:3, :3], abs=1.0e-9)


def test_map_anchor_keeps_fastlio_relative_motion_unchanged():
    initial_map_to_base = MODULE.configured_pose([5.0, -2.0, 0.7])
    initial_odom_to_base = MODULE.configured_pose(
        [8.0, 3.0, 0.4, 0.05, -0.02, -1.1]
    )
    map_to_odom = initial_map_to_base @ np.linalg.inv(initial_odom_to_base)

    odom_motion = MODULE.configured_pose([1.5, -0.2, 0.1])
    later_odom_to_base = initial_odom_to_base @ odom_motion
    later_map_to_base = map_to_odom @ later_odom_to_base

    assert later_map_to_base == pytest.approx(
        initial_map_to_base @ odom_motion, abs=1.0e-9
    )
