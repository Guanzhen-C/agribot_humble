import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from types import SimpleNamespace
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, RobotState
from agribot_hil_apple.harvest import pose

from agribot_hil_apple.arm_preflight import JOINTS, FINGERS, STOW, approach_orientation
from agribot_hil_apple.harvest import Harvester, approach_options, resample, world_to_base
from agribot_hil_apple.arm_servo import command_times
from agribot_hil_apple.fruit_geometry import stem_dimensions


@pytest.mark.parametrize('offset', [.04, .07])
def test_variable_stem_reaches_wood_and_collision_stays_inside_visual(offset):
    shape = stem_dimensions(offset)
    assert shape['visual_center']+shape['visual_length']/2 == pytest.approx(offset+.003)
    assert shape['visual_center']-shape['visual_length']/2 == pytest.approx(-.003)
    assert shape['collision_center']-shape['collision_length']/2 == pytest.approx(.001)
    assert shape['collision_center']+shape['collision_length']/2 < offset


def test_harvest_node_constructs_without_overriding_ros_node_properties(tmp_path):
    import rclpy
    rclpy.init()
    node = None
    try:
        node = Harvester(tmp_path, tmp_path/'report.json')
        assert node.service_clients == {}
        assert node.report['executed'] is False
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_resample_converts_cumulative_moveit_times_to_plugin_step_delays():
    trajectory = JointTrajectory(joint_names=['joint1'], points=[
        JointTrajectoryPoint(positions=[0.], velocities=[0.], time_from_start=Duration()),
        JointTrajectoryPoint(positions=[.4], velocities=[0.], time_from_start=Duration(sec=2)),
        JointTrajectoryPoint(positions=[.6], velocities=[0.], time_from_start=Duration(sec=3))])
    samples, delays, duration = resample(trajectory)
    assert samples[0, 0] == pytest.approx(0.)
    assert samples[-1, 0] == pytest.approx(.6)
    assert sum(delays) == pytest.approx(3.)
    assert max(delays) <= .020001
    assert delays[-1] == 0.
    assert duration == 3.
    assert np.isfinite(samples).all()


def test_resample_rejects_duplicate_timestamps():
    with pytest.raises(ValueError):
        resample(JointTrajectory(points=[JointTrajectoryPoint(positions=[0.])]*2))


def test_world_coordinates_are_transformed_using_actual_base_roll_and_height():
    rotation = Rotation.from_euler('xyz', [.05, -.02, 2.9])
    base = [28.3, 31.95, .04, *rotation.as_quat()]
    relative = np.array([[.2, .34, .7], [.3, .4, .8]])
    world = rotation.apply(relative)+base[:3]
    assert np.allclose(world_to_base(world, base), relative)


def test_tilted_stem_and_jaw_closing_axis_are_perpendicular():
    stem = Rotation.from_euler('xy', [.05, -.02]).apply([0., 0., 1.])
    orientation = Rotation.from_quat(approach_orientation([.2, .34, .7], [.25, 0., .384], stem))
    assert abs(np.dot(orientation.apply([0., 1., 0.]), stem)) < 1e-12


def test_position_servo_accepts_standard_absolute_trajectory_timing():
    message = JointTrajectory(joint_names=['j'], points=[
        JointTrajectoryPoint(positions=[0.], time_from_start=Duration()),
        JointTrajectoryPoint(positions=[1.], time_from_start=Duration(sec=1))])
    assert command_times(message, ['j']) == [0., 1.]
    message.points[1].time_from_start = Duration()
    with pytest.raises(ValueError):
        command_times(message, ['j'])


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_servo_rejects_nonfinite_joint_targets(value):
    message = JointTrajectory(joint_names=['j'], points=[JointTrajectoryPoint(positions=[value])])
    with pytest.raises(ValueError):
        command_times(message, ['j'])


def test_attaching_a_world_fruit_does_not_also_remove_it_after_attachment():
    scenes = []
    observation = {'base_in_world': [0., 0., 0., 0., 0., 0., 1.], 'fruits': [{
        'model': 'apple', 'socket_in_world': [0., 0., .7],
        'orientation_in_world': [0., 0., 0., 1.]}]}
    node = SimpleNamespace(call=lambda *args: SimpleNamespace(scene=PlanningScene()),
        wait_observation=lambda: observation, observation=observation,
        fk=lambda **kwargs: pose([0., 0., .65]), apply_scene=scenes.append)
    Harvester.attach_planning_object(node, 'apple')
    assert len(scenes) == 1
    assert not scenes[0].world.collision_objects
    assert scenes[0].robot_state.attached_collision_objects[0].object.id == 'apple'


def test_recovering_an_already_attached_planning_object_is_idempotent():
    scene = PlanningScene()
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
        link_name='manipulator_gripper_shell', object=CollisionObject(id='apple'))]
    node = SimpleNamespace(call=lambda *args: SimpleNamespace(scene=scene))
    Harvester.attach_planning_object(node, 'apple')


def test_post_validation_rejected_motion_is_replanned_and_never_returned():
    valid = JointTrajectory(points=[JointTrajectoryPoint(positions=[0.])])
    responses = [SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=code), trajectory=SimpleNamespace(
            joint_trajectory=valid if code == 1 else JointTrajectory()))) for code in (-2, -2, 1)]
    events = []
    node = SimpleNamespace(call=lambda *args: responses.pop(0),
                           event=lambda name, **kwargs: events.append((name, kwargs)))
    state = RobotState()
    state.joint_state.name, state.joint_state.position = JOINTS+FINGERS, STOW
    assert Harvester.plan(node, state, state) is valid
    assert [name for name, _ in events] == ['replanning_rejected_motion', 'replanning_rejected_motion', 'motion_planned']
    assert events[-1][1]['velocity_scaling'] == .8
    assert not responses


def test_side_approaches_keep_the_grasp_axis_perpendicular_to_the_stem():
    target = np.array([.2, .3, .7])
    stem = Rotation.from_euler('xy', [.04, -.02]).apply([0., 0., 1.])
    options = list(approach_options(target, [.25, 0., .384], stem))
    assert len(options) == 19
    assert any(abs(a) > 1. for a, _, _, _ in options)
    for _, _, pre, quaternion in options:
        rotation = Rotation.from_quat(quaternion)
        assert np.linalg.norm(target-pre) == pytest.approx(.035)
        assert abs(np.dot(rotation.apply([0., 1., 0.]), stem)) < 1e-12
        assert np.allclose((target-pre)/.035, rotation.apply([0., 0., 1.]))


@pytest.mark.parametrize('limit,count', [(1, 1), (2, 2), (None, 2)])
def test_online_selection_stops_after_enough_reachable_candidates(limit, count):
    fruits = [{'model': str(i), 'grasp_in_base': [.2, .3+.1*i, .7],
               'stem_axis_in_base': [0., 0., 1.]} for i in range(2)]
    planned = []
    path = SimpleNamespace(points=[None])
    node = SimpleNamespace(report={'harvested': [], 'candidates': []},
        wait_observation=lambda: {'fruits': fruits}, environment=lambda _: None,
        fk=lambda **kwargs: pose([.25, 0., .384]), state=lambda: RobotState(),
        ik=lambda *args: RobotState(), cartesian=lambda *args: path,
        plan=lambda *args: planned.append(True) or path, event=lambda *args, **kwargs: None)
    accepted = Harvester.candidates(node, maximum=limit)
    assert len(accepted) == len(planned) == count


def test_selection_rejects_nonpositive_limit():
    with pytest.raises(ValueError):
        Harvester.candidates(SimpleNamespace(), maximum=0)


def test_precise_approach_retains_collision_check_and_lower_speed():
    trajectory = JointTrajectory(points=[JointTrajectoryPoint(positions=[0.]), JointTrajectoryPoint(positions=[.01])])
    requests = []
    node = SimpleNamespace(call=lambda _, __, request: requests.append(request) or
        SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.,
                        solution=SimpleNamespace(joint_trajectory=trajectory)))
    Harvester.cartesian(node, RobotState(), pose([.2, .3, .7]))
    request = requests[0]
    assert request.avoid_collisions is True
    assert request.max_step == .002
    assert request.max_velocity_scaling_factor == .35
    assert request.max_acceleration_scaling_factor == .5


def test_execution_waits_for_scheduled_end_without_two_second_padding():
    trajectory = JointTrajectory(joint_names=['j'], points=[
        JointTrajectoryPoint(positions=[0.], velocities=[0.], time_from_start=Duration()),
        JointTrajectoryPoint(positions=[.1], velocities=[0.], time_from_start=Duration(sec=1))])
    waits = []
    node = SimpleNamespace(wait_observation=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)),
        event=lambda *a, **k: None, send_samples=lambda *a: None,
        wait_joint_goal=lambda target, duration: waits.append((target, duration)))
    Harvester.execute(node, trajectory)
    assert waits[0][0]['j'] == pytest.approx(.1)
    assert waits[0][1] == pytest.approx(1.15)


@pytest.mark.parametrize('rejections', [0, 1, 4])
def test_retreat_executes_only_a_complete_validated_option(rejections):
    attempts, executed = [], []
    tcp = pose([.2, .3, .7], Rotation.from_euler('y', np.pi/2).as_quat())
    def cartesian(state, target):
        attempts.append(target)
        if len(attempts) <= rejections:
            raise RuntimeError('Incomplete collision-checked approach: 0.941')
        return 'validated'
    node = SimpleNamespace(wait_observation=lambda: {'fruits': [
        {'model': 'apple', 'stem_axis_in_base': [0., 0., 1.]}]},
        fk=lambda: tcp, state=lambda: RobotState(), cartesian=cartesian,
        execute=executed.append, event=lambda *a, **k: None)
    if rejections == 4:
        with pytest.raises(RuntimeError, match='No complete collision-checked retreat'):
            Harvester.retreat_held(node, 'apple')
        assert not executed
    else:
        Harvester.retreat_held(node, 'apple')
        assert executed == ['validated']
        assert len(attempts) == rejections+1


def test_short_cartesian_motion_rejects_slow_joint_winding_even_without_large_steps():
    trajectory = JointTrajectory(points=[JointTrajectoryPoint(positions=[i*.1]) for i in range(70)])
    node = SimpleNamespace(call=lambda *a: SimpleNamespace(error_code=SimpleNamespace(val=1),
        fraction=1., solution=SimpleNamespace(joint_trajectory=trajectory)))
    with pytest.raises(RuntimeError, match='winds a joint'):
        Harvester.cartesian(node, RobotState(), pose([.2, .3, .7]))


def test_prospective_timing_records_joint_travel_without_execution():
    from agribot_hil_apple.pickup_preflight import trajectory_metrics
    trajectory = JointTrajectory(points=[JointTrajectoryPoint(positions=[0., 0.]),
        JointTrajectoryPoint(positions=[.2, -.4], time_from_start=Duration(sec=2))])
    assert trajectory_metrics(trajectory) == {'points': 2, 'duration_sim_sec': 2.,
                                             'maximum_joint_travel_rad': .4}
