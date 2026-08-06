import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "mapping_result_trajectory_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("mapping_result_publisher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_pose_composition_and_inverse_preserve_rigid_transform():
    pose = (
        (2.0, -3.0, 0.5),
        MODULE.quaternion_from_rpy(0.1, -0.2, 0.7),
    )
    identity = MODULE.compose_pose(*pose, *MODULE.inverse_pose(*pose))

    assert identity[0] == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert identity[1] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)


def test_pose_composition_rotates_translation_before_adding_it():
    quarter_turn = MODULE.quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
    result = MODULE.compose_pose(
        (1.0, 2.0, 0.0), quarter_turn,
        (2.0, 0.0, 0.0), quarter_turn,
    )

    assert result[0] == pytest.approx((1.0, 4.0, 0.0), abs=1.0e-12)
    assert abs(result[1][2]) == pytest.approx(1.0, abs=1.0e-12)
    assert abs(result[1][3]) == pytest.approx(0.0, abs=1.0e-12)
