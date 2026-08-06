import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "odometry_reference_aligner.py"
)
SPEC = importlib.util.spec_from_file_location("odometry_reference_aligner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_reference_alignment_recovers_known_rigid_transform():
    source_from_base = (
        (2.0, -1.0, 0.2),
        MODULE.normalize((0.0, 0.0, math.sin(0.2), math.cos(0.2))),
    )
    reference_from_source = (
        (10.0, 4.0, -0.2),
        MODULE.normalize((0.0, 0.0, math.sin(0.35), math.cos(0.35))),
    )
    reference_from_base = MODULE.compose_pose(
        *reference_from_source, *source_from_base
    )

    recovered = MODULE.compose_pose(
        *reference_from_base, *MODULE.inverse_pose(*source_from_base)
    )

    assert recovered[0] == pytest.approx(reference_from_source[0], abs=1.0e-12)
    assert recovered[1] == pytest.approx(reference_from_source[1], abs=1.0e-12)


def test_inverse_pose_round_trip_is_identity():
    pose = (
        (1.5, -3.0, 0.7),
        MODULE.normalize((0.1, -0.2, 0.3, 0.9)),
    )

    identity = MODULE.compose_pose(*pose, *MODULE.inverse_pose(*pose))

    assert identity[0] == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert identity[1] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)
