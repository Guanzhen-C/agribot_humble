import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

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


def test_fastlivo_imu_pose_is_converted_to_the_rear_axle_center():
    message = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=10.0, y=2.0, z=1.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
    )
    imu_mount = {
        "xyz": [0.1425, 0.0, 0.143],
        "rpy": [0.0, 0.0, 0.0],
    }

    position, orientation = MODULE.sensor_odometry_pose_to_base(
        message, imu_mount
    )

    assert position == pytest.approx((9.8575, 2.0, 0.857), abs=1.0e-12)
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)


def test_fastlivo_imu_mount_rotation_is_removed_from_vehicle_orientation():
    quarter_turn = MODULE.quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
    message = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=10.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(
                    x=quarter_turn[0],
                    y=quarter_turn[1],
                    z=quarter_turn[2],
                    w=quarter_turn[3],
                ),
            )
        )
    )
    imu_mount = {
        "xyz": [1.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, math.pi / 2.0],
    }

    position, orientation = MODULE.sensor_odometry_pose_to_base(
        message, imu_mount
    )

    assert position == pytest.approx((9.0, 0.0, 0.0), abs=1.0e-12)
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)
