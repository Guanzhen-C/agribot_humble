import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from agribot_localization_switch.core import Handover, Sample


def pose(x=0., y=0., yaw=0.):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[:3, 3] = [x, y, 0.]
    return value


def test_no_switch_without_ordered_request_and_stable_overlap():
    core = Handover()
    assert not core.consider(pose(), pose(.1), True, 0.)
    assert core.mode == "navsat"
    assert core.request() and not core.request()
    assert not core.consider(pose(), pose(.1), True, 0.)
    assert not core.consider(pose(), pose(.1), True, 1.)
    assert core.consider(pose(), pose(.1), True, 2., last_output=pose(.02))
    assert core.mode == "fastlivo_rtk"
    assert np.allclose(core.corrected(pose(.1)), pose(.02))
    assert not core.consider(pose(), pose(), False, 3.)


@pytest.mark.parametrize("other", [pose(.6), pose(yaw=math.radians(11)), np.full((4, 4), np.nan)])
def test_bad_overlap_never_takes_over(other):
    core = Handover();core.request()
    for t in range(10):
        assert not core.consider(pose(), other, True, float(t))
    assert core.mode == "navsat"


def test_unhealthy_or_disagreeing_sample_restarts_stability_window():
    core = Handover();core.request()
    assert not core.consider(pose(), pose(), True, 0.)
    assert not core.consider(pose(), pose(), False, 1.9)
    assert not core.consider(pose(), pose(), True, 2.)
    assert not core.consider(pose(), pose(), True, 3.9)
    assert core.consider(pose(), pose(), True, 4.)


def test_valid_delayed_overlap_waits_for_fresh_takeover_and_aligns_current_pose():
    core = Handover(stable_sec=0.); core.request()
    assert not core.consider(pose(), pose(.1), True, 0., can_take_over=False)
    assert core.mode == "navsat"
    assert core.consider(pose(), pose(.1), True, .1,
                         last_output=pose(.2), output_fused_pose=pose(.3))
    assert np.allclose(core.corrected(pose(.3)), pose(.2))


def test_offset_slew_is_bounded_at_vehicle_not_at_far_map_origin():
    nav = pose(1000., 2000., math.radians(1))
    fused = pose(1000.1, 2000.1, math.radians(3))
    core = Handover(stable_sec=0.);core.request()
    assert core.consider(nav, fused, True, 0., last_output=nav)
    before = core.corrected(fused)
    core.relax_offset(.1, reference=fused)
    after = core.corrected(fused)
    assert np.linalg.norm(after[:3, 3]-before[:3, 3]) <= .00500001
    angle = Rotation.from_matrix(before[:3, :3].T @ after[:3, :3]).magnitude()
    assert angle <= math.radians(.2)+1e-8


def test_prediction_uses_body_velocity_and_refuses_old_or_future_samples():
    sample = Sample(100., 100., pose(yaw=math.pi/2), np.array([1., 0., 0.]),
                    np.array([0., 0., 1.]), np.eye(6))
    result = sample.at(100.25)
    assert result[0, 3] == pytest.approx(0., abs=1e-10)
    assert result[1, 3] == pytest.approx(.25)
    for target in (99.9, 100.36):
        with pytest.raises(ValueError):sample.at(target)
