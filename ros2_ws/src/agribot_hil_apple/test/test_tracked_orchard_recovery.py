from types import SimpleNamespace

import pytest

from agribot_hil_apple.navigation import ControllerPathError
from agribot_hil_apple.tracked_orchard_mission import OrchardMission


def fixture(failures):
    points = [[41., 36., 0.], [41., 32.5, 3.14159], [3., 32.5, 3.14159]]
    node = SimpleNamespace(manifest={'fixed_route_points': points}, report={'navigation': []},
                           route_index=0, executing=False, plans=[], stops=0, follows=0)
    node.begin_route = lambda p: None
    node.persist = lambda: None
    def plan(p):
        node.plans.append(p)
        return SimpleNamespace(poses=[])
    def follow(path, timeout):
        node.follows += 1
        if node.follows <= failures:
            if node.follows == 1:
                node.route_index = 1
            raise ControllerPathError('Native MPPI aborted')
    def stop():
        node.stops += 1
    node.plan, node.follow, node.stop = plan, follow, stop
    return node


def test_replan_only_after_failure_preserves_remaining_vias():
    node = fixture(1)
    OrchardMission.navigate_route(node)
    assert node.plans == [node.manifest['fixed_route_points'], node.manifest['fixed_route_points'][1:]]
    assert node.stops == 1 and node.follows == 2
    assert node.report['navigation'][0]['executed'] is False
    assert node.report['navigation'][1]['executed'] is True


def test_no_periodic_replanning_or_unbounded_retries():
    node = fixture(0)
    OrchardMission.navigate_route(node)
    assert len(node.plans) == 1 and node.stops == 0
    node = fixture(100)
    with pytest.raises(ControllerPathError):
        OrchardMission.navigate_route(node)
    assert len(node.plans) == 7 and node.stops == 7


def test_unrelated_failure_is_not_hidden_by_replanning():
    node = fixture(0)
    def failed(*args, **kwargs):
        raise ValueError('Unrelated error')
    node.follow = failed
    with pytest.raises(ValueError):
        OrchardMission.navigate_route(node)
    assert len(node.plans) == 1


def test_run06_baseline_does_not_enable_experimental_planner_retry():
    node = fixture(0)
    original_plan = node.plan
    def plan(points):
        path = original_plan(points)
        raise RuntimeError('Native start footprint occupied')
    node.plan = plan
    with pytest.raises(RuntimeError):
        OrchardMission.navigate_route(node)
    assert len(node.plans) == 1 and node.follows == 0
    assert all(points == node.manifest['fixed_route_points'] for points in node.plans)
