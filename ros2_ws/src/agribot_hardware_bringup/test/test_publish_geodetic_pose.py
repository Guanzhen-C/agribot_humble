import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "publish_geodetic_pose.py"
SPEC = importlib.util.spec_from_file_location("publish_geodetic_pose", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(
    "heading_degrees, expected_yaw",
    ((0.0, math.pi / 2.0), (90.0, 0.0), (180.0, -math.pi / 2.0)),
)
def test_converts_clockwise_north_heading_to_enu_yaw(
    heading_degrees, expected_yaw
):
    assert MODULE.enu_yaw_from_heading_degrees(heading_degrees) == pytest.approx(
        expected_yaw
    )


def test_parser_accepts_required_geodetic_pose_values():
    parsed = MODULE.parse_arguments(["39.9", "116.3", "271.5"])
    assert parsed.latitude == 39.9
    assert parsed.longitude == 116.3
    assert parsed.heading_degrees == 271.5
    assert math.isnan(parsed.altitude)
