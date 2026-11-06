from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import Bool

from agribot_hil_apple.gripper_audit import joint_metrics
from agribot_hil_apple.harvest import Harvester


def test_feedback_audit_rejects_the_previously_measured_four_mm_drift():
    samples = [[float(t), 0., 0., 1.57, 0., 0., 0., .0089, .0089] for t in np.arange(0., 1.01, .05)]
    assert joint_metrics(samples, 0., 1., .0089)['maximum_opening_error_m'] < 1e-10
    samples[4][-1] -= .004
    assert joint_metrics(samples, 0., 1., .0089)['maximum_opening_error_m'] > .00025
    assert not joint_metrics(samples[:4], 0., 1., .0089)['complete']


def test_transit_restraint_cannot_force_open_a_held_fruit():
    node = SimpleNamespace(report={'held_fruit': 'apple'})
    with pytest.raises(RuntimeError, match='holding fruit'):
        Harvester.transit_hold(node, True)


def test_acknowledged_restraint_and_release_are_recorded(monkeypatch):
    from agribot_hil_apple import harvest
    import time
    messages, events = [], []
    node = SimpleNamespace(report={}, transit_status=True, transit_status_received=time.monotonic(),
        transit_publisher=SimpleNamespace(publish=messages.append),
        event=lambda name, **values: events.append((name, values)))
    Harvester.transit_hold(node, True)
    def spin(_node, timeout_sec):
        Harvester.receive_transit_status(node, Bool(data=False))
    monkeypatch.setattr(harvest.rclpy, 'spin_once', spin)
    Harvester.transit_hold(node, False)
    assert [message.data for message in messages] == [True, False]
    assert events[-1] == ('transit_gripper_hold', {'engaged': False})
