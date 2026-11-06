import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from agribot_hil_apple.gravity_reference import base_to_level_quaternion


@pytest.mark.parametrize('rpy', [[0., .13, 0.], [.15, -.2, 2.1], [0., 0., -1.4]])
def test_level_frame_uses_gravity_and_preserves_heading_with_mounted_imu(rpy):
    mounting = [.0005724, -.0091395, -.0000026]
    world_base = Rotation.from_euler('xyz', rpy)
    world_imu = world_base*Rotation.from_euler('xyz', mounting)
    relative = Rotation.from_quat(base_to_level_quaternion(world_imu.as_quat(), mounting))
    world_level = world_base*relative
    np.testing.assert_allclose(world_level.apply([0., 0., 1.]), [0., 0., 1.], atol=1e-12)
    expected = world_base.apply([1., 0., 0.]); expected[2] = 0.; expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(world_level.apply([1., 0., 0.]), expected, atol=1e-12)


def test_pitch_does_not_turn_distant_ground_into_obstacle_but_retains_trunk():
    world_base = Rotation.from_euler('xyz', [0., .13, 0.])
    world_points = [[3., 0., -.1], [3., 0., .3]]
    body_points = world_base.inv().apply(world_points)
    assert .08 < body_points[0, 2] < .55
    relative = Rotation.from_quat(base_to_level_quaternion(world_base.as_quat(), [0., 0., 0.]))
    level_points = relative.inv().apply(body_points)
    np.testing.assert_allclose(level_points, world_points, atol=1e-12)
    assert level_points[0, 2] < .08
    assert .08 < level_points[1, 2] < .55


@pytest.mark.parametrize('quaternion', [[0., 0., 0., 0.], [float('nan'), 0., 0., 1.]])
def test_invalid_orientation_is_rejected(quaternion):
    with pytest.raises(ValueError):
        base_to_level_quaternion(quaternion, [0., 0., 0.])
