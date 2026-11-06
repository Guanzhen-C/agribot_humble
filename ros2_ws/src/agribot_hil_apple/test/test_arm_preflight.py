import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from agribot_hil_apple.arm_preflight import approach_orientation


def test_grasp_approach_aligns_tcp_and_keeps_jaw_closure_horizontal():
    shoulder = np.array([.252, .001, .384])
    target = np.array([0., .23, .834])
    rotation = Rotation.from_quat(approach_orientation(target, shoulder)).as_matrix()
    expected = (target - shoulder) / np.linalg.norm(target - shoulder)
    assert np.allclose(rotation[:, 2], expected)
    assert abs(rotation[2, 1]) < 1e-12
    assert np.allclose(rotation.T @ rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1.)


@pytest.mark.parametrize('target', [[0., 0., 0.], [0., 0., 1.]])
def test_degenerate_approach_is_rejected(target):
    with pytest.raises(ValueError):
        approach_orientation(target, [0., 0., 0.])
