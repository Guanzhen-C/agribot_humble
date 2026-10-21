import importlib.util
from pathlib import Path
import math
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/verify_algorithm_runtime.py"
spec = importlib.util.spec_from_file_location("runtime_acceptance", SCRIPT)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)

ANCHOR_SCRIPT = SCRIPT.parents[2] / "agribot_autonomy/scripts/kiss_localization.py"
anchor_spec = importlib.util.spec_from_file_location("simulation_map_anchor", ANCHOR_SCRIPT)
anchor = importlib.util.module_from_spec(anchor_spec)
anchor_spec.loader.exec_module(anchor)


def test_settled_spawn_anchor_is_applied_once_not_as_motion_ground_truth():
    frame = SimpleNamespace(planar_mode=False)
    node = SimpleNamespace(
        auto_initial_pose_applied=False, map_to_odom=None,
        allow_reinitialization=False, pending_initial_pose=None, auto_initial_pose_topic="/odom",
        transform_from_pose=lambda pose: anchor.KissLocalization.transform_from_pose(frame, pose),
        get_logger=lambda: Mock(), try_initialize_transform=Mock())
    first = runtime.Odometry()
    first.pose.pose = runtime.pose(2.01, 36.0, -0.013).pose
    first.pose.pose.position.z = 0.04
    anchor.KissLocalization.handle_auto_initial_odom(node, first)
    initial = node.pending_initial_pose
    assert initial[0] == pytest.approx((2.01, 36.0, 0.04))
    assert anchor.yaw_from_quaternion(initial[1]) == pytest.approx(-0.013)
    moved = runtime.Odometry()
    moved.pose.pose = runtime.pose(20, 30, 1).pose
    anchor.KissLocalization.handle_auto_initial_odom(node, moved)
    assert node.pending_initial_pose == initial
    node.try_initialize_transform.assert_called_once_with()


def test_full_pose_map_anchor_preserves_roll_pitch_and_yaw():
    frame = SimpleNamespace(planar_mode=False)
    initial = ((2.01, 36.0, 0.04), anchor.quaternion_from_rpy(0.02, -0.015, -0.013))
    odom = runtime.Odometry()
    odom.pose.pose = runtime.pose(-0.1425, 0, 0).pose
    node = SimpleNamespace(
        latest_odom=odom, pending_initial_pose=initial,
        transform_from_pose=lambda pose: anchor.KissLocalization.transform_from_pose(frame, pose),
        get_logger=lambda: Mock())
    anchor.KissLocalization.try_initialize_transform(node)
    reconstructed = anchor.compose_transform(node.map_to_odom, node.transform_from_pose(odom.pose.pose))
    assert reconstructed[0] == pytest.approx(initial[0])
    assert reconstructed[1] == pytest.approx(initial[1])
    assert node.pending_initial_pose is None


def test_coverage_exercises_every_component_without_claiming_full_matrix():
    cases = runtime.coverage_cases()
    assert len(cases) == len(set(cases)) == 14
    for index, choices in enumerate((runtime.PERCEPTIONS, runtime.VISIONS,
                                    runtime.LOCALIZATIONS, runtime.PLANNERS,
                                    runtime.CONTROLLERS)):
        assert {case[index] for case in cases} == set(choices)


def test_prefix_keeps_original_geometry_and_order():
    path = runtime.PathMessage()
    path.header.frame_id = "map"
    path.poses = [runtime.pose(i / 10, 0, 0) for i in range(101)]
    prefix = runtime.path_prefix(path, 2)
    assert prefix.header.frame_id == "map"
    assert prefix.poses == path.poses[:21]


def test_terminal_heading_error_wraps_at_pi():
    assert runtime.heading_error(runtime.pose(0, 0, math.pi - 0.1).pose, -math.pi) == pytest.approx(0.1)
    assert runtime.heading_error(runtime.pose(0, 0, math.pi / 2).pose, math.pi / 2) == pytest.approx(0)


def test_full_route_only_final_waypoint_uses_half_wheelbase():
    assert runtime.waypoint_acceptance_radii(3, 2.8, True) == [.75, .75, 1.4]
    assert runtime.waypoint_acceptance_radii(3, .4, True) == [.75, .75, .2]
    assert runtime.waypoint_acceptance_radii(3, 2.8, False) == [.75, .75, .75]
    assert runtime.waypoint_acceptance_radii(0, 2.8, True) == []


@pytest.mark.parametrize("command,twist,expected", [
    ([0., 0.], [0., 0.], True),
    ([0., 0.], [.019, -.019], True),
    ([.01, 0.], [0., 0.], False),
    ([0., 0.], [.02, 0.], False),
    ([0., 0.], [0., .02], False),
    ([float("nan"), 0.], [0., 0.], False),
    ([0., 0.], [float("inf"), 0.], False),
    ([], [0., 0.], False),
])
def test_terminal_motion_requires_zero_command_and_finite_stationary_truth(command, twist, expected):
    assert runtime.terminal_motion_stopped(command, twist) is expected


def test_native_stop_records_fresh_truth_without_sending_control(monkeypatch):
    now = [0.]
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now[0])
    probe = SimpleNamespace(trace=[{"wall_monotonic": -1.}], latest_command=[0., 0.],
                            capture_terminal=False, terminal_observation=None)
    def until(predicate, seconds):
        assert seconds == 10 and probe.capture_terminal
        assert predicate() is False
        for stamp in (0., .5, 1.):
            now[0] = stamp
            probe.trace.append({"wall_monotonic": stamp, "chassis_twist": [0., 0.],
                                "truth": [4.81, -.04, .22]})
            if predicate():
                return True
        return False
    probe.until = until
    runtime.Probe.observe_native_stop(probe)
    assert probe.capture_terminal is False
    assert probe.terminal_observation["truth"] == [4.81, -.04, .22]
    assert probe.terminal_observation["fresh_terminal_samples"] == 3
    assert probe.terminal_observation["stationary_hold_s"] == 1.


@pytest.mark.parametrize("reason", ["stale", "commanded", "moving", "no_fresh_sample"])
def test_native_success_alone_cannot_pass_terminal_observation(monkeypatch, reason):
    now = [0.]
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now[0])
    probe = SimpleNamespace(trace=[], latest_command=[0., 0.], capture_terminal=False)
    def until(predicate, seconds):
        for stamp in (0., .5, 1., 2.):
            now[0] = stamp
            if reason != "no_fresh_sample":
                probe.trace.append({"wall_monotonic": stamp - 1 if reason == "stale" else stamp,
                                    "chassis_twist": [.1 if reason == "moving" else 0., 0.],
                                    "truth": [4.7, 0., .2]})
            probe.latest_command = [.1 if reason == "commanded" else 0., 0.]
            assert predicate() is False
        return False
    probe.until = until
    with pytest.raises(RuntimeError, match="stationary odometry"):
        runtime.Probe.observe_native_stop(probe)
    assert probe.capture_terminal is False


def test_native_stop_stability_timer_resets_when_motion_resumes(monkeypatch):
    now = [0.]
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now[0])
    probe = SimpleNamespace(trace=[], latest_command=[0., 0.], capture_terminal=False)
    def until(predicate, seconds):
        for stamp, speed, expected in ((0., 0., False), (.5, .1, False),
                                       (1., 0., False), (1.5, 0., False), (2., 0., True)):
            now[0] = stamp
            probe.trace.append({"wall_monotonic": stamp, "chassis_twist": [speed, 0.],
                                "truth": [4.8, 0., .2]})
            assert predicate() is expected
        return True
    probe.until = until
    runtime.Probe.observe_native_stop(probe)
    assert probe.terminal_observation["stationary_hold_s"] == 1.


def test_final_stop_uses_requested_position_and_keeps_the_original_allowance():
    observation = {"truth": [4.81, 0., 100.]}
    assert runtime.check_settled_position(observation, runtime.pose(5., 0., math.pi), .201, .03) == pytest.approx(.19)
    assert runtime.check_settled_position({"truth": [4.78, 0., 0.]}, runtime.pose(5., 0., 0.), .201, .03) == pytest.approx(.22)


@pytest.mark.parametrize("truth", [[4.7, 0., 0.], [5., .24, 0.], [float("nan"), 0., 0.]])
def test_passing_near_goal_before_stopping_outside_it_does_not_pass(truth):
    with pytest.raises(RuntimeError, match="stopped outside"):
        runtime.check_settled_position({"truth": truth}, runtime.pose(5., 0., 0.), .201, .03)


def test_optional_stall_observer_reports_failure_without_sending_control(tmp_path, monkeypatch):
    log = tmp_path / "route.log"
    log.write_text("")
    now = [0.]
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now[0])
    probe = SimpleNamespace(initial_truth=runtime.pose(0, 0, 0).pose,
        truth=runtime.pose(0, 0, 0).pose, stall_timeout=10., plan_updates=1)
    def until(predicate, seconds):
        predicate()
        now[0] = 11.
        return predicate()
    probe.until = until
    process = SimpleNamespace(poll=lambda: None)
    with pytest.raises(RuntimeError, match="No physical route progress"):
        runtime.Probe.observe_production_route(probe, log, process, 100)


def test_waypoint_check_rejects_missing_and_out_of_order_points():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(i, 0, 0) for i in range(6)]
    assert runtime.ordered_waypoint_distances(path, [runtime.pose(2, 0, 0),
                                                    runtime.pose(4, 0, 0)]) == [0, 0]
    assert max(runtime.ordered_waypoint_distances(path, [runtime.pose(4, 0, 0),
                                                       runtime.pose(2, 0, 0)])) > 0.35
    assert max(runtime.ordered_waypoint_distances(path, [runtime.pose(9, 0, 0)])) > 0.35


def test_empty_planner_path_is_never_accepted():
    with pytest.raises(ValueError, match="empty path"):
        runtime.ordered_waypoint_distances(runtime.PathMessage(), [runtime.pose(0, 0, 0)])


def test_waypoint_check_does_not_match_outbound_point_to_a_closer_return_visit():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(x, y, 0) for x, y in
                  [(0, 0.1), (2, 0.1), (4, 0.1), (2, 0), (0, 0)]]
    waypoints = [runtime.pose(x, 0, 0) for x in (0, 2, 4, 2, 0)]
    assert max(runtime.ordered_waypoint_distances(path, waypoints)) == pytest.approx(0.1)
    assert runtime.ordered_waypoint_distances(path, []) == []


def test_ordered_waypoint_check_matches_exhaustive_minimax_assignment():
    from itertools import combinations_with_replacement

    path = runtime.PathMessage()
    path.poses = [runtime.pose(x, 0, 0) for x in (0, 1, 3, 4, 2, 0)]
    targets = [runtime.pose(x, 0, 0) for x in (1.2, 3.8, 0.2)]
    expected = min(max(abs(path.poses[i].pose.position.x - target.pose.position.x)
                       for i, target in zip(indices, targets))
                   for indices in combinations_with_replacement(range(len(path.poses)), len(targets)))
    assert max(runtime.ordered_waypoint_distances(path, targets)) == pytest.approx(expected)


def test_final_stationary_error_is_not_an_arbitrary_minimax_tie():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(0, .6, 0), runtime.pose(5, .45, 0), runtime.pose(5, .18, 0)]
    targets = [runtime.pose(0, 0, 0), runtime.pose(5, 0, 0)]
    assert runtime.ordered_waypoint_distances(path, targets)[-1] == pytest.approx(.45)
    errors = runtime.driven_waypoint_distances(path, targets, {"truth": [5, .18, 0]}, True)
    assert errors == pytest.approx([.6, .18])


def test_large_final_radius_does_not_weaken_intermediate_neighborhoods():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(0, .3, 0), runtime.pose(5, 1.2, 0)]
    targets = [runtime.pose(0, 0, 0), runtime.pose(5, 0, 0)]
    assert runtime.driven_waypoint_distances(path, targets, {"truth": [5, 1.2, 0]}, True) == pytest.approx([.3, 1.2])


def test_terminal_check_does_not_accept_an_earlier_goal_crossing():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(0, 0, 0), runtime.pose(5, 0, 0), runtime.pose(7, 0, 0)]
    targets = [runtime.pose(0, 0, 0), runtime.pose(5, 0, 0)]
    assert runtime.driven_waypoint_distances(path, targets, {"truth": [7, 0, 0]}, True) == [0, 2]


def test_full_route_single_goal_and_nonfinite_endpoint():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(0, 0, 0)]
    targets = [runtime.pose(1, 0, 0)]
    assert runtime.driven_waypoint_distances(path, targets, {"truth": [.9, 0, 0]}, True) == pytest.approx([.1])
    with pytest.raises(ValueError, match="finite"):
        runtime.driven_waypoint_distances(path, targets, {"truth": [float("nan"), 0, 0]}, True)
    assert runtime.driven_waypoint_distances(path, targets, None, False) == [1]


def test_pose_error_uses_matching_sensor_time_and_does_not_duplicate_pairs():
    node = SimpleNamespace(
        truth_history=[(1.0, runtime.pose(1.0, 0, 0).pose),
                       (1.03, runtime.pose(1.03, 0, 0).pose),
                       (1.5, runtime.pose(1.5, 0, 0).pose)],
        estimate_stamp=1.03, last_error_stamp=None,
        estimate=runtime.pose(1.04, 0, 0).pose,
        position_errors=[], position_error_time_deltas=[])
    runtime.Probe.record_pose_error(node)
    assert node.position_errors == pytest.approx([0.01])
    assert node.position_error_time_deltas == [0.0]
    runtime.Probe.record_pose_error(node)
    assert len(node.position_errors) == 1
    node.estimate_stamp = 2.0
    runtime.Probe.record_pose_error(node)
    assert len(node.position_errors) == 1


def test_domain_rotation_avoids_physical_unity_and_ephemeral_port_ranges():
    domains = {runtime.case_domain(93, index) for index in range(480)}
    assert domains == set(runtime.SAFE_DOMAINS)
    assert 0 not in domains and 37 not in domains
    assert not domains.intersection(range(102, 215))
    assert runtime.case_domain(93, 20, reuse=True) == 93


def test_abnormal_shutdown_is_not_hidden_by_successful_motion():
    log = "[ERROR] [fastlivo_mapping-6]: process has died [pid 123, exit code -11, cmd 'test']."
    assert runtime.abnormal_exits(log) == [{"process": "fastlivo_mapping-6", "code": -11}]
    assert runtime.abnormal_exits("process has finished cleanly") == []


def test_startup_fails_immediately_when_launch_has_exited():
    probe = SimpleNamespace(until=lambda predicate, seconds: predicate())
    process = SimpleNamespace(poll=lambda: 0, returncode=0)
    with pytest.raises(RuntimeError, match="launch exited during startup"):
        runtime.Probe.ready(probe, 120, process)


def test_compatibility_library_is_identified_from_actual_server_mappings():
    assert runtime.gazebo_pid("[INFO] [gzserver-2]: process started with pid [123]") == 123
    with pytest.raises(RuntimeError, match="PID"):
        runtime.gazebo_pid("[INFO] [controller_server-3]: process started with pid [123]")
    maps = "7f-8f r-xp 0 103:02 9 /tmp/compat/libgazebo_transport.so.11.10.2\n" \
           "8f-9f r--p 0 103:02 9 /tmp/compat/libgazebo_transport.so.11.10.2\n" \
           "9f-af r-xp 0 103:02 10 /usr/lib/libtbb.so.12\n"
    assert runtime.loaded_library_paths(maps, "libgazebo_transport.so") == [
        "/tmp/compat/libgazebo_transport.so.11.10.2"]


def test_truncated_path_endpoint_has_valid_tangent_orientation():
    path = runtime.PathMessage()
    path.poses = [runtime.pose(0, i / 10, 0) for i in range(40)]
    for item in path.poses:
        item.pose.orientation.w = 0.0
    prefix = runtime.path_prefix(path, 2)
    q = prefix.poses[-1].pose.orientation
    assert q.z == pytest.approx(2 ** -0.5)
    assert q.w == pytest.approx(2 ** -0.5)
    assert path.poses[20].pose.orientation.w == 0.0


def test_direct_route_uses_production_sampling_and_preserves_final_heading():
    start = runtime.pose(0, 0, 1).pose
    path = runtime.production_direct_path(start, [runtime.pose(2, 0, math.pi / 2)])
    assert [p.pose.position.x for p in path.poses] == [0, 0.5, 1, 1.5, 2]
    assert runtime.ordered_waypoint_distances(path, [runtime.pose(2, 0, 0)]) == [0]
    assert path.poses[-1].pose.orientation.z == pytest.approx(2 ** -0.5)


def test_cyclone_participant_ports_fit_every_safe_domain():
    config = SCRIPT.parents[1] / "config/cyclonedds_simulation.xml"
    index = int(ET.parse(config).find(".//{https://cdds.io/config}MaxAutoParticipantIndex").text)
    assert index >= 20
    for domain in runtime.SAFE_DOMAINS:
        highest_port = 7400 + 250 * domain + 11 + 2 * index
        assert highest_port <= 65535
        if domain <= 101:
            assert highest_port < 32768
