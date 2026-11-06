from types import SimpleNamespace

import pytest

from agribot_hil_apple import harvest, mission
from agribot_hil_apple.arm_preflight import JOINTS, FINGERS


def observation_node(monkeypatch, rows):
    wall = [0.]
    events, seen = [], []
    node = SimpleNamespace(observation=None, received=0., joint_receipt=0.,
        joints=dict.fromkeys(JOINTS+FINGERS, 0.),
        event=lambda name, **values: events.append((name, values)))
    rows = iter(rows)

    def spin(_node, timeout_sec):
        wall[0] += .1
        stamp, speed, yaw_rate = next(rows)
        seen.append(stamp)
        node.observation = {'stamp': stamp, 'base_speed': speed, 'base_yaw_rate': yaw_rate}
        node.received = node.joint_receipt = wall[0]

    monkeypatch.setattr(harvest.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(harvest.rclpy, 'spin_once', spin)
    return node, events, seen


def test_transient_motion_waits_for_sustained_stationarity_without_raising_threshold(monkeypatch):
    rows = [(0., .08, 0.), (.1, .001, 0.), (.2, .001, 0.),
            (.3, .001, 0.), (.4, .001, 0.)]
    node, events, _ = observation_node(monkeypatch, rows)
    result = harvest.Harvester.wait_observation(node)
    assert result['stamp'] == .4
    assert result is not node.observation
    assert events[-1][0] == 'vehicle_settled_after_transient_motion'


@pytest.mark.parametrize('speed,yaw_rate', [(.036, 0.), (0., .051), (float('nan'), 0.)])
def test_continuous_motion_or_invalid_speed_never_passes(monkeypatch, speed, yaw_rate):
    node, events, _ = observation_node(monkeypatch, ((i*.1, speed, yaw_rate) for i in range(200)))
    with pytest.raises(RuntimeError, match='settling timed out'):
        harvest.Harvester.wait_observation(node)
    assert events[-1][0] == 'manipulation_settle_timeout'


def test_duplicate_observation_timestamps_do_not_prove_stationarity(monkeypatch):
    node, events, _ = observation_node(monkeypatch, ((1., 0., 0.) for _ in range(200)))
    with pytest.raises(TimeoutError, match='advancing'):
        harvest.Harvester.wait_observation(node)
    assert not events


def test_clock_rewind_restarts_stationarity_window(monkeypatch):
    rows = [(5., 0., 0.), (5.1, 0., 0.), (0., 0., 0.),
            (.1, 0., 0.), (.2, 0., 0.), (.3, 0., 0.)]
    node, _, seen = observation_node(monkeypatch, rows)
    assert harvest.Harvester.wait_observation(node)['stamp'] == .3
    assert seen == [row[0] for row in rows]


def test_mission_settling_refreshes_stale_clock_before_starting_interval(monkeypatch):
    sim = [10.]
    wall = [0.]
    navigator = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(sim[0]*1e9))))
    consumed = []
    updates = iter([100., 100.5, 101., 101.5, 102.])

    def spin(_node, timeout_sec):
        wall[0] += .1
        sim[0] = next(updates)
        consumed.append(sim[0])

    monkeypatch.setattr(mission.rclpy, 'spin_once', spin)
    monkeypatch.setattr(mission.time, 'monotonic', lambda: wall[0])
    mission.Mission.wait_sim(SimpleNamespace(navigator=navigator), 2.)
    assert consumed == [100., 100.5, 101., 101.5, 102.]


def test_mission_event_uses_latest_clock_from_the_active_subsystem():
    def node(stamp):
        return SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(stamp*1e9))))

    current = SimpleNamespace(navigator=node(10.), harvester=node(100.), started=0.,
                              report={'events': []}, save=lambda: None)
    mission.Mission.event(current, 'picking_finished')
    assert current.report['events'][-1]['sim_time'] == 100.


def test_closed_stdout_does_not_abort_a_mission_or_lose_persisted_events(monkeypatch):
    clock = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1000000000)))
    saved, printed = [], []
    current = SimpleNamespace(navigator=clock, harvester=clock, started=0.,
        report={'events': []}, save=lambda: saved.append(True))

    def closed_pipe(*args, **kwargs):
        printed.append(True)
        raise BrokenPipeError('Disconnected SSH output')

    monkeypatch.setattr('builtins.print', closed_pipe)
    mission.Mission.event(current, 'following')
    mission.Mission.event(current, 'navigation_completed')
    assert len(current.report['events']) == len(saved) == 2
    assert len(printed) == 1
    assert current.stdout_available is False


def test_mission_storage_error_is_not_hidden_as_an_optional_logging_failure():
    clock = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1000000000)))
    current = SimpleNamespace(navigator=clock, harvester=clock, started=0.,
        report={'events': []}, save=lambda: (_ for _ in ()).throw(OSError('Disk full')))
    with pytest.raises(OSError, match='Disk full'):
        mission.Mission.event(current, 'following')
