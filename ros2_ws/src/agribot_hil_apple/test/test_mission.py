import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import Odometry

from agribot_hil_apple.mission import Mission, ResourceSampler, resource_snapshot
from agribot_hil_apple.mission_record import Recorder
from agribot_hil_apple.navigation import Navigator, TerminalProgress, PathProgress, advance_via_index
from lifecycle_msgs.msg import State
from agribot_hil_apple.path_preflight import path_metrics


def test_terminal_progress_detects_circling_even_while_vehicle_is_moving():
    progress = TerminalProgress()
    for stamp in range(14):
        progress.update(0., .4+.01*(stamp % 2), float(stamp))
    assert progress.expired(13.)


def test_terminal_progress_uses_cross_track_improvement_after_path_remaining_is_zero():
    progress = TerminalProgress()
    for stamp in range(40):
        progress.update(0., 1.-.02*stamp, float(stamp))
        assert not progress.expired(float(stamp))


def test_terminal_progress_does_not_abort_a_planned_long_detour():
    progress = TerminalProgress()
    progress.update(.5, .5, 0.)
    progress.update(10., .6, 30.)
    assert not progress.expired(31.)
    progress.update(1.5, .5, 80.)
    assert not progress.expired(81.)


def test_terminal_progress_reinitializes_after_clock_reset_and_ignores_invalid_feedback():
    progress = TerminalProgress()
    progress.update(float('nan'), 1., 1.)
    assert not progress.expired(100.)
    progress.update(1., .5, 50.)
    progress.update(1., .5, 2.)
    assert not progress.expired(3.)


def test_route_metrics_detect_reverse_and_excessive_direction_switches():
    rows = [[0., 0., 0., 1.], [1., 0., 0., 1.], [.5, 0., 0., 1.], [1., 0., 0., 1.]]
    result = path_metrics(rows)
    assert result['direction_changes'] == 2
    assert result['reverse_segments'] == 1
    assert result['length_m'] == 2.


def test_lifecycle_activation_retries_a_busy_startup_service():
    cancelled = []
    pending = SimpleNamespace(cancel=lambda: cancelled.append(True))
    futures = [pending, SimpleNamespace()]
    client = SimpleNamespace(wait_for_service=lambda **kwargs: True,
                             call_async=lambda _: futures.pop(0))

    def wait(future, timeout):
        if future is pending:
            raise TimeoutError('Server still activating')
        return SimpleNamespace(current_state=State(id=State.PRIMARY_STATE_ACTIVE))

    destroyed = []
    node = SimpleNamespace(create_client=lambda *args: client, wait=wait,
                           destroy_client=destroyed.append)
    Navigator.wait_active(node, 'planner_server')
    assert cancelled == [True]
    assert destroyed == [client]


def test_route_resume_retires_only_reached_intermediate_points():
    points = [[41., 36., 0.], [41., 32.5, 3.14], [28.3, 31.95, 3.14]]
    assert advance_via_index(points, 0, 14., 36.) == 0
    assert advance_via_index(points, 0, 41., 35.8) == 1
    assert advance_via_index(points, 1, 41.1, 32.6) == 2
    assert advance_via_index(points, 2, 28.3, 31.95) == 2


def test_resource_probe_reads_current_system_and_sampler_stops():
    sample = resource_snapshot()
    assert sample['MemAvailable_kib'] > 0
    assert sample['MemTotal_kib'] >= sample['MemAvailable_kib']
    sampler = ResourceSampler()
    result = sampler.close()
    assert not sampler.worker.is_alive()
    assert not result['errors']


def test_navigation_recovery_counts_stalls_not_successful_progress():
    odometry = Odometry()
    events = []
    calls = []

    def follow(_path):
        calls.append(True)
        odometry.pose.pose.position.x += 1.
        if len(calls) < 7:
            raise RuntimeError('Transient optimizer failure')

    navigator = SimpleNamespace(pose=odometry, route_index=0,
        begin_route=lambda _: None, plan=lambda _: SimpleNamespace(poses=[]),
        follow=follow, stop=lambda: None)
    node = SimpleNamespace(navigator=navigator, report={'navigation': []},
        event=lambda name, **values: events.append((name, values)), wait_sim=lambda _: None)
    Mission.navigate(node, 'outbound', [[10., 0., 0.]])
    assert len(calls) == 7
    assert all(value['consecutive_stalled_attempts'] == 0 for name, value in events if name == 'navigation_attempt_failed')
    assert node.report['navigation'][-1]['status'] == 'succeeded'


def test_navigation_recovery_stops_after_four_attempts_without_progress():
    navigator = SimpleNamespace(pose=Odometry(), route_index=0,
        begin_route=lambda _: None, plan=lambda _: (_ for _ in ()).throw(RuntimeError('No path')),
        stop=lambda: None)
    node = SimpleNamespace(navigator=navigator, report={'navigation': []},
        event=lambda *args, **kwargs: None, wait_sim=lambda _: None)
    with pytest.raises(RuntimeError, match='No path'):
        Mission.navigate(node, 'outbound', [[10., 0., 0.]])
    assert len(node.report['navigation']) == 4


def test_return_uses_static_planner_configuration_without_a_blocking_rebuild():
    calls = []
    route = [[41., 32.5, 3.141592653589793], [41., 36., 0.], [2., 36., 0.]]
    node = SimpleNamespace(manifest={'return_via': route},
        navigate=lambda *args: calls.append(args),
        navigate_profile=lambda *args: pytest.fail('Return must not rebuild Smac between phases'))
    Mission.finish_route(node)
    assert calls == [('return', route)]


@pytest.mark.parametrize('route', [
    [[3., 32.5, 3.14159]],
    [[3., 32.5, 3.14159], [3., 29., 0.]],
])
def test_completion_uses_only_requested_points_without_adding_home_or_point_one(route):
    calls = []
    node = SimpleNamespace(manifest={'completion_via': route}, navigate=lambda *args: calls.append(args))
    Mission.finish_route(node)
    assert calls == [('completion', route)]


@pytest.mark.parametrize('fail', [False, True])
def test_navigation_profile_is_restored_even_when_navigation_fails(fail, monkeypatch):
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    from rclpy.parameter import Parameter
    old = [Parameter('GridBased.reverse_penalty', value=2.).to_parameter_msg()]
    restored, closed, timeouts = [], [], []
    client = SimpleNamespace(call_async=lambda request: restored.extend(request.parameters))
    nav = SimpleNamespace(parameter_profile=lambda *args: (client, old),
        wait=lambda _, timeout: (timeouts.append(timeout),
            SimpleNamespace(result=SimpleNamespace(successful=True)))[1],
        destroy_client=closed.append)

    def navigate(*args):
        if fail:
            raise RuntimeError('Test failure')

    node = SimpleNamespace(navigator=nav, manifest={'return_via': [[2., 36., 0.]]},
        event=lambda *args, **kwargs: None, navigate=navigate)
    if fail:
        with pytest.raises(RuntimeError, match='Test failure'):
            Mission.navigate_profile(node, 'return', node.manifest['return_via'], '/planner_server', {})
    else:
        Mission.navigate_profile(node, 'return', node.manifest['return_via'], '/planner_server', {})
    assert restored == old
    assert closed == [client]
    assert timeouts == [120.]


def test_shutdown_does_not_mask_cancellation_with_a_restore_timeout(monkeypatch):
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    events, closed = [], []
    client = SimpleNamespace(call_async=lambda _: pytest.fail('Context is already shut down'))
    nav = SimpleNamespace(parameter_profile=lambda *args: (client, []), destroy_client=closed.append)
    node = SimpleNamespace(navigator=nav, event=lambda name, **_: events.append(name),
        navigate=lambda *args: (_ for _ in ()).throw(KeyboardInterrupt()))
    with pytest.raises(KeyboardInterrupt):
        Mission.navigate_profile(node, 'return', [], '/planner_server', {})
    assert events[-1] == 'navigation_profile_restore_deferred'
    assert closed == [client]


def test_path_progress_detects_oscillation_far_from_endpoint():
    progress = PathProgress()
    for now in range(46):
        progress.update(50.+.1*(now % 2), float(now))
    assert progress.expired(46.)


def test_path_progress_allows_long_detours_with_decreasing_path_length():
    progress = PathProgress()
    for now in range(100):
        progress.update(80.-now*.1, float(now))
        assert not progress.expired(float(now))
    progress.update(float('nan'), 120.)
    progress.update(75., 1.)
    assert not progress.expired(2.)


@pytest.mark.parametrize('first_stop_count,expected_stops', [(1, 2), (2, 1)])
@pytest.mark.parametrize('pickup_tolerance', [None, .08, .10])
def test_mission_uses_backup_parking_only_when_more_fruit_are_needed(first_stop_count, expected_stops, pickup_tolerance):
    from agribot_hil_apple.arm_preflight import JOINTS, STOW
    odometry = Odometry()
    odometry.pose.pose.position.x, odometry.pose.pose.position.y = 2., 36.
    models = ['harvest_apple_1', 'harvest_apple_2', 'harvest_apple_5']
    observation = {'base_in_world': [2., 36., .04, 0., 0., 0., 1.], 'fruits': [
        {'model': model, 'socket_in_world': [1.95, 36., .317],
         'orientation_in_world': [0., 0., 0., 1.]} for model in models]}
    arm = SimpleNamespace(report={'harvested': []}, joints=dict(zip(JOINTS, STOW)),
                          parking_engaged=False, wait_observation=lambda: observation,
                          event=lambda *args, **kwargs: None)

    def candidates(models, maximum):
        reachable = ['harvest_apple_1', 'harvest_apple_2'][:first_stop_count]+['harvest_apple_5']
        return [name for name in reachable if name in models and name not in arm.report['harvested']][:maximum]

    arm.candidates = candidates
    arm.pick = lambda candidate, stow_after: arm.report['harvested'].append(candidate)
    arm.stow = lambda: None
    hold_modes = []
    arm.transit_hold = hold_modes.append
    arm.parking_fixture = lambda engaged: setattr(arm, 'parking_engaged', engaged)
    navigator = SimpleNamespace(pose=odometry, wait_ready=lambda **kwargs: None,
                                wait_active=lambda *args: None)
    stops = [{'park': [28., 32., 3.14], 'target_models': models[:2]},
             {'park': [30., 32., 3.14], 'target_models': models[2:]}]
    visited = []
    node = SimpleNamespace(navigator=navigator, harvester=arm, report={},
        manifest={'spawn': [2., 36., .1325, 0.], 'park': stops[0]['park'],
                  'pickup_stops': stops, 'outbound_via': [[41., 36., 0.], stops[0]['park']]},
        event=lambda *args, **kwargs: None, wait_sim=lambda *args: None,
        navigate_profile=lambda phase, points, *args: visited.append((phase, points, args[-1])),
        finish_route=lambda: None)
    if pickup_tolerance is not None:
        node.manifest['pickup_position_tolerance_m'] = pickup_tolerance
    Mission.run(node, True, 2, 2)
    assert node.report['status'] == 'succeeded'
    assert len(visited) == expected_stops
    assert len(node.report['harvested']) == 2
    assert hold_modes == [True]+[mode for _ in range(expected_stops) for mode in (False, True)]
    assert all(profile['goal_checker.xy_goal_tolerance'] == (.10 if pickup_tolerance is None else pickup_tolerance) and
               profile['goal_checker.yaw_goal_tolerance'] == .3 and
               profile['FollowPath.GoalAngleCritic.enabled'] is True and
               profile['FollowPath.PathAlignCritic.use_path_orientations'] is True
               for _, _, profile in visited)


def test_recorder_constructs_and_records_measurements_without_command_publishers():
    rclpy.init()
    node = None
    try:
        node = Recorder()
        message = Odometry()
        message.header.stamp.sec = 1
        message.pose.pose.position.x = 2.
        message.twist.twist.linear.x = -.2
        node.receive('/hil/ground_truth', message)
        node.receive('/hil/ground_truth', message)
        assert node.report['counts']['/hil/ground_truth'] == 2
        assert len(node.report['traces']['/hil/ground_truth']) == 1
        assert node.report['traces']['/hil/ground_truth'][0][1] == 2.
        assert node.report['twists']['/hil/ground_truth'][0][1] == -.2
        assert not any('cmd_vel' in p.topic_name for p in node.publishers)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
