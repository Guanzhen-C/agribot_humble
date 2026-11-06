"""X5 MoveIt picking client with observed poses, contacts and physical fixtures.

Object poses are the explicitly labelled Gazebo manipulation baseline, not an
image detector. Chassis navigation remains a separate native-CAN operation.
"""
import argparse
import copy
import json
from pathlib import Path
import time

from builtin_interfaces.msg import Duration
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point, Pose
from linkattacher_msgs.srv import AttachLink, DetachLink
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                            CollisionObject, Constraints, JointConstraint,
                            PlanningScene, PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan,
                            GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_preflight import JOINTS, FINGERS, STOW, approach_orientation
from .fruit_geometry import stem_dimensions


def pose(position, quaternion=(0., 0., 0., 1.)):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, position)
    (result.orientation.x, result.orientation.y,
     result.orientation.z, result.orientation.w) = map(float, quaternion)
    return result


def resample(trajectory, period=.02):
    """Convert MoveIt absolute timing into this Gazebo plugin's step delays."""
    points = trajectory.points
    times = np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points])
    if len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('MoveIt trajectory must have increasing, finite timestamps')
    positions = np.asarray([p.positions for p in points])
    velocities = np.asarray([p.velocities for p in points])
    if velocities.shape != positions.shape:
        velocities = np.gradient(positions, times, axis=0)
    sample_times = np.r_[np.arange(times[0], times[-1], period), times[-1]]
    samples = CubicHermiteSpline(times, positions, velocities)(sample_times)
    return samples, np.r_[np.diff(sample_times), 0.], times[-1]-times[0]


def world_to_base(points, base):
    return Rotation.from_quat(base[3:]).inv().apply(np.asarray(points)-base[:3])


def approach_options(target, shoulder, stem_axis):
    target, shoulder = np.asarray(target), np.asarray(shoulder)
    stem = np.asarray(stem_axis)/np.linalg.norm(stem_axis)
    horizontal = target-shoulder
    horizontal -= np.dot(horizontal, stem)*stem
    if np.linalg.norm(horizontal) < 1e-8:
        raise ValueError('Target does not define an approach perpendicular to the stem')
    horizontal /= np.linalg.norm(horizontal)
    angles = [(0., e) for e in (0., -.2, -.4, -.6, .2, .45, .7)]
    angles += [(a, e) for a in (-.6, .6, -1.2, 1.2) for e in (-.2, -.4, 0.)]
    for azimuth, elevation in angles:
        lateral = Rotation.from_rotvec(stem*azimuth).apply(horizontal)
        direction = np.cos(elevation)*lateral+np.sin(elevation)*stem
        quaternion = approach_orientation(target, target-direction, stem)
        yield azimuth, elevation, target-.035*direction, quaternion


class Harvester(Node):
    def __init__(self, scenario, output):
        super().__init__('hil_harvester', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scenario, self.output = scenario, output
        self.observation, self.received, self.joints = None, 0., {}
        self.joint_receipt = 0.
        self.contacts = {1: [], 2: []}
        self.parking_engaged = False
        self.transit_status, self.transit_status_received = None, 0.
        self.report = {'status': 'initializing', 'executed': False,
                       'object_pose_source': 'Gazebo manipulation baseline, not visual detection',
                       'planner': 'X5 MoveIt2 KDL + OMPL RRTConnect',
                       'candidates': [], 'harvested': [], 'events': []}
        self.create_subscription(String, '/hil/fruit/observations', self.observe, 2)
        self.create_subscription(JointState, '/joint_states', self.receive_joints, 20)
        for index in (1, 2):
            self.create_subscription(ContactsState, f'/hil/gripper/finger{index}/contacts',
                                     lambda m, i=index: self.contact(i, m), 20)
        self.publisher = self.create_publisher(JointTrajectory, '/hil/arm/command', 2)
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool
        hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.transit_publisher = self.create_publisher(Bool, '/hil/gripper/transit_hold', hold_qos)
        self.create_subscription(Bool, '/hil/gripper/transit_hold/status', self.receive_transit_status, hold_qos)
        self.service_clients = {}

    def receive_transit_status(self, message):
        self.transit_status, self.transit_status_received = message.data, time.monotonic()

    def transit_hold(self, engaged):
        from std_msgs.msg import Bool
        if engaged and self.report.get('held_fruit'):
            raise RuntimeError('Cannot engage transit restraint while holding fruit')
        self.transit_publisher.publish(Bool(data=engaged))
        deadline = time.monotonic()+8.
        while self.transit_status is not engaged or time.monotonic()-self.transit_status_received > .5:
            if time.monotonic() > deadline:
                raise TimeoutError('Gazebo finger transit restraint did not acknowledge the requested state')
            rclpy.spin_once(self, timeout_sec=.05)
        self.event('transit_gripper_hold', engaged=engaged)

    def observe(self, message):
        self.observation, self.received = json.loads(message.data), time.monotonic()

    def receive_joints(self, message):
        self.joints.update(zip(message.name, message.position))
        if all(name in message.name for name in JOINTS+FINGERS):
            self.joint_receipt = time.monotonic()

    def contact(self, index, message):
        now = self.get_clock().now().nanoseconds/1e9
        self.contacts[index] = [(stamp, pair) for stamp, pair in self.contacts[index] if now-stamp < 2.]
        self.contacts[index].extend((now, (s.collision1_name, s.collision2_name)) for s in message.states)

    def event(self, name, **values):
        self.report['events'].append({'event': name, 'sim_time': self.get_clock().now().nanoseconds/1e9, **values})
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.report, indent=2)+'\n')
        if rclpy.ok():
            self.get_logger().info(name+': '+json.dumps(values))

    def call(self, service, kind, request, timeout=20.):
        if service not in self.service_clients:
            self.service_clients[service] = self.create_client(kind, service)
        client = self.service_clients[service]
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f'{service} unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise TimeoutError(service)
        return future.result()

    def wait_observation(self):
        deadline = time.monotonic()+10.
        stable_since, previous_stamp, saw_motion = None, None, False
        while True:
            if time.monotonic() > deadline:
                if saw_motion:
                    observation = self.observation or {}
                    self.event('manipulation_settle_timeout',
                        base_speed=observation.get('base_speed'),
                        base_yaw_rate=observation.get('base_yaw_rate'))
                    raise RuntimeError('Vehicle must be stopped before manipulating; settling timed out')
                raise TimeoutError('Fresh advancing fruit observations and stopped arm feedback unavailable')
            rclpy.spin_once(self, timeout_sec=.05)
            if (self.observation is None or not all(n in self.joints for n in JOINTS+FINGERS) or
                    time.monotonic()-self.received > 1.5 or time.monotonic()-self.joint_receipt > 1.5):
                stable_since = None
                continue
            observation = self.observation
            stamp = float(observation['stamp'])
            speed, yaw_rate = observation['base_speed'], observation['base_yaw_rate']
            if not np.isfinite([stamp, speed, yaw_rate]).all() or speed > .035 or abs(yaw_rate) > .05:
                stable_since, saw_motion = None, True
                continue
            if stable_since is None or (previous_stamp is not None and stamp < previous_stamp):
                stable_since = stamp
            previous_stamp = stamp
            if stamp-stable_since >= .3-1e-9:
                if saw_motion:
                    self.event('vehicle_settled_after_transient_motion',
                        stable_duration_sim_sec=stamp-stable_since, base_speed=speed, base_yaw_rate=yaw_rate)
                return copy.deepcopy(observation)

    def state(self, positions=None):
        result = RobotState()
        result.joint_state.name = JOINTS+FINGERS
        result.joint_state.position = (list(positions) if positions is not None else
                                       [self.joints[n] for n in JOINTS+FINGERS])
        result.is_diff = True
        return result

    def fk(self, state=None, link='manipulator_tcp'):
        request = GetPositionFK.Request(robot_state=state or self.state(), fk_link_names=[link])
        request.header.frame_id = 'base_link'
        result = self.call('/compute_fk', GetPositionFK, request)
        if result.error_code.val != 1 or not result.pose_stamped:
            raise RuntimeError('Mounted arm FK failed')
        return result.pose_stamped[0].pose

    def apply_scene(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.call('/apply_planning_scene', ApplyPlanningScene,
                         ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('MoveIt planning scene rejected')

    def environment(self, observation):
        data = json.loads((self.scenario/'arm_environment.json').read_text())
        vertices = world_to_base(data['vertices_in_world'], observation['base_in_world'])
        mesh = Mesh(vertices=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices],
                    triangles=[MeshTriangle(vertex_indices=t) for t in data['triangles']])
        wood = CollisionObject(id='orchard_branches', meshes=[mesh], mesh_poses=[pose([0., 0., 0.])])
        wood.header.frame_id, wood.operation = 'base_link', CollisionObject.ADD
        scene = PlanningScene()
        scene.world.collision_objects.append(wood)
        matrix_names = []
        for fruit in observation['fruits']:
            model = fruit['model']
            stem_shape = stem_dimensions(fruit.get('socket_to_branch_m', .04))
            if model in self.report['harvested']:
                continue
            socket = world_to_base(fruit['socket_in_world'], observation['base_in_world'])
            rotation = (Rotation.from_quat(observation['base_in_world'][3:]).inv()*
                        Rotation.from_quat(fruit['orientation_in_world']))
            body = CollisionObject(id=model)
            body.header.frame_id, body.operation = 'base_link', CollisionObject.ADD
            body.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.035]),
                               SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                              dimensions=[stem_shape['collision_length'], .0018])]
            body.primitive_poses = [pose(socket+rotation.apply([0., 0., -.032]), rotation.as_quat()),
                                    pose(socket+rotation.apply([0., 0., stem_shape['collision_center']]), rotation.as_quat())]
            scene.world.collision_objects.append(body)
            matrix_names.append(model)
        # Touch permission is limited to the two physical finger pads.
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix = self.call('/get_planning_scene', GetPlanningScene, request).scene.allowed_collision_matrix
        original = len(matrix.entry_names)
        names = list(matrix.entry_names)+[name for name in matrix_names if name not in matrix.entry_names]
        rows = [list(entry.enabled)+[False]*(len(names)-original) for entry in matrix.entry_values]
        rows.extend([[False]*len(names) for _ in range(len(names)-original)])
        for fruit in matrix_names:
            for finger in ('manipulator_finger1', 'manipulator_finger2'):
                a, b = names.index(fruit), names.index(finger)
                rows[a][b] = rows[b][a] = True
        matrix.entry_names = names
        matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        scene.allowed_collision_matrix = matrix
        self.apply_scene(scene)
        valid = self.call('/check_state_validity', GetStateValidity,
                          GetStateValidity.Request(robot_state=self.state(), group_name='arm'))
        if not valid.valid:
            pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
            raise RuntimeError(f'Current arm state collides with the observed environment: {pairs}')

    def ik(self, position, quaternion, start):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name, ik.robot_state = 'arm', 'manipulator_tcp', start
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.pose = pose(position, quaternion)
        ik.timeout.sec, ik.avoid_collisions = 1, True
        result = self.call('/compute_ik', GetPositionIK, request)
        if result.error_code.val != 1:
            ik.avoid_collisions = False
            geometric = self.call('/compute_ik', GetPositionIK, request)
            contacts = []
            if geometric.error_code.val == 1:
                valid = self.call('/check_state_validity', GetStateValidity,
                    GetStateValidity.Request(robot_state=geometric.solution, group_name='arm'))
                contacts = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
            raise RuntimeError(f'Collision-free IK unavailable: {result.error_code.val}; '
                               f'geometric_code={geometric.error_code.val}, contacts={contacts}')
        values = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        return self.state([values[n] if n in JOINTS else start.joint_state.position[start.joint_state.name.index(n)]
                           for n in JOINTS+FINGERS])

    def plan(self, start, goal):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name, motion.pipeline_id, motion.planner_id = 'arm', 'ompl', 'RRTConnectkConfigDefault'
        motion.start_state = start
        motion.allowed_planning_time, motion.num_planning_attempts = 5., 3
        motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = .8
        constraint = Constraints(joint_constraints=[JointConstraint(joint_name=n, position=float(v),
            tolerance_above=.002, tolerance_below=.002, weight=1.)
            for n, v in zip(goal.joint_state.name, goal.joint_state.position) if n in JOINTS])
        motion.goal_constraints = [constraint]
        started = time.monotonic()
        for attempt in range(3):
            result = self.call('/plan_kinematic_path', GetMotionPlan, request).motion_plan_response
            if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                trajectory = result.trajectory.joint_trajectory
                self.event('motion_planned', planning_wall_sec=time.monotonic()-started,
                    trajectory_duration_sim_sec=trajectory.points[-1].time_from_start.sec+
                    trajectory.points[-1].time_from_start.nanosec*1e-9,
                    velocity_scaling=.8, acceleration_scaling=.8)
                return trajectory
            if result.error_code.val != -2:
                break
            self.event('replanning_rejected_motion', attempt=attempt+1)
        raise RuntimeError(f'OMPL planning failed: {result.error_code.val}')

    def cartesian(self, start, target):
        request = GetCartesianPath.Request(start_state=start, group_name='arm', link_name='manipulator_tcp',
                                           waypoints=[target], max_step=.002, jump_threshold=0.,
                                           avoid_collisions=True)
        request.header.frame_id = 'base_link'
        request.max_velocity_scaling_factor = .35
        request.max_acceleration_scaling_factor = .5
        result = self.call('/compute_cartesian_path', GetCartesianPath, request)
        if result.error_code.val != 1 or result.fraction < .99:
            raise RuntimeError(f'Incomplete collision-checked approach: {result.fraction:.3f}')
        positions = np.asarray([point.positions for point in result.solution.joint_trajectory.points])
        if len(positions) > 1 and np.max(np.abs(np.diff(positions, axis=0))) > .2:
            raise RuntimeError('Cartesian approach contains an excessive absolute joint step')
        if len(positions) > 1 and np.max(np.abs(np.diff(positions, axis=0)).sum(axis=0)) > np.pi:
            raise RuntimeError('Short Cartesian approach winds a joint by more than pi radians')
        return result.solution.joint_trajectory

    def execute(self, trajectory):
        self.wait_observation()
        samples, delays, duration = resample(trajectory)
        started, stamp = time.monotonic(), self.get_clock().now().nanoseconds/1e9
        self.event('motion_execution_started', trajectory_duration_sim_sec=float(duration))
        self.send_samples(trajectory.joint_names, samples, delays)
        end = dict(zip(trajectory.joint_names, samples[-1]))
        self.wait_joint_goal(end, duration+.15)
        self.event('motion_execution_completed', wall_duration_sec=time.monotonic()-started,
                   sim_duration_sec=self.get_clock().now().nanoseconds/1e9-stamp)

    def send_samples(self, names, samples, delays):
        message = JointTrajectory(joint_names=list(names))
        message.header.frame_id = 'hil_ackermann::base_link'
        message.header.stamp = (self.get_clock().now()+rclpy.duration.Duration(seconds=.15)).to_msg()
        elapsed = 0.
        for sample, delay in zip(samples, delays):
            ns = int(round(elapsed*1e9))
            message.points.append(JointTrajectoryPoint(positions=list(map(float, sample)),
                                  time_from_start=Duration(sec=ns//10**9, nanosec=ns%10**9)))
            elapsed += float(delay)
        self.publisher.publish(message)

    def wait_joint_goal(self, target, sim_duration):
        end = self.get_clock().now().nanoseconds/1e9+sim_duration
        deadline = time.monotonic()+max(20., sim_duration*5.)
        while True:
            rclpy.spin_once(self, timeout_sec=.02)
            now = self.get_clock().now().nanoseconds/1e9
            if now >= end and all(abs(self.joints[n]-v) < (.0004 if n in FINGERS else .012)
                                  for n, v in target.items()):
                return
            if time.monotonic() > deadline or now > end+5.:
                raise TimeoutError('Arm trajectory did not converge to the commanded joint state')

    def fingers(self, opening):
        names = JOINTS+FINGERS
        start = np.array([self.joints[n] for n in names])
        target = start.copy()
        target[-2:] = opening
        samples = np.linspace(start, target, 41)
        self.send_samples(names, samples, np.r_[np.full(40, .02), 0.])
        self.wait_joint_goal(dict(zip(names, target)), 1.)

    def candidates(self, models=None, maximum=None):
        if maximum is not None and maximum < 1:
            raise ValueError('Candidate limit must be positive')
        observation = self.wait_observation()
        self.environment(observation)
        shoulder_pose = self.fk(link='manipulator_link1')
        shoulder = np.array([shoulder_pose.position.x, shoulder_pose.position.y, shoulder_pose.position.z])
        fruits = sorted(observation['fruits'], key=lambda f: np.linalg.norm(np.array(f['grasp_in_base'])-shoulder))
        accepted = []
        for fruit in fruits:
            if fruit['model'] in self.report['harvested'] or (models is not None and fruit['model'] not in models):
                continue
            entry = {'model': fruit['model'], 'target_in_base': fruit['grasp_in_base']}
            self.report['candidates'].append(entry)
            target = np.array(fruit['grasp_in_base'])
            stem = np.asarray(fruit['stem_axis_in_base'])
            entry['attempts'] = []
            for azimuth, elevation, pre, quaternion in approach_options(target, shoulder, stem):
                try:
                    state = self.ik(pre, quaternion, self.state())
                    approach = self.cartesian(state, pose(target, quaternion))
                    plan = self.plan(self.state(), state)
                    entry.update(status='reachable', azimuth_rad=azimuth, elevation_rad=elevation,
                                 pregrasp_points=len(plan.points), approach_points=len(approach.points))
                    accepted.append((fruit, plan, approach, pose(pre, quaternion)))
                    break
                except RuntimeError as error:
                    entry['attempts'].append({'azimuth_rad': azimuth, 'elevation_rad': elevation, 'reason': str(error)})
            else:
                entry['status'] = 'skipped'
            self.event('candidate_checked', **entry)
            if maximum is not None and len(accepted) >= maximum:
                break
        return accepted

    def transfer_fixture(self, model):
        self.call('/pause_physics', Empty, Empty.Request())
        attached = False
        try:
            request = AttachLink.Request(model1_name='hil_ackermann', link1_name='manipulator_gripper_shell',
                                         model2_name=model, link2_name='fruit')
            result = self.call('/ATTACHLINK', AttachLink, request, timeout=5.)
            if not result.success:
                raise RuntimeError(result.message)
            attached = True
            result = self.call('/DETACHLINK', DetachLink, DetachLink.Request(
                model1_name='orchard_geometry', link1_name='orchard_world_link', model2_name=model, link2_name='fruit'), timeout=5.)
            if not result.success:
                raise RuntimeError(result.message)
        except Exception:
            if attached:
                self.call('/DETACHLINK', DetachLink, DetachLink.Request(
                    model1_name='hil_ackermann', link1_name='manipulator_gripper_shell', model2_name=model, link2_name='fruit'), timeout=5.)
            raise
        finally:
            self.call('/unpause_physics', Empty, Empty.Request())

    def parking_fixture(self, engage):
        if engage:
            self.wait_observation()
            result = self.call('/ATTACHLINK', AttachLink, AttachLink.Request(
                model1_name='orchard_geometry', link1_name='orchard_world_link',
                model2_name='hil_ackermann', link2_name='base_link'))
            if not result.success and 'already been attached' not in result.message:
                raise RuntimeError(result.message)
        else:
            result = self.call('/DETACHLINK', DetachLink, DetachLink.Request(
                model1_name='orchard_geometry', link1_name='orchard_world_link',
                model2_name='hil_ackermann', link2_name='base_link'))
            if not result.success:
                raise RuntimeError(result.message)
        self.parking_engaged = engage
        self.report['parking_constraint_engaged'] = engage
        self.event('simulation_parking_constraint', engaged=engage)

    def attach_planning_object(self, model):
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        current = self.call('/get_planning_scene', GetPlanningScene, request).scene
        if any(obj.object.id == model for obj in current.robot_state.attached_collision_objects):
            return
        scene = PlanningScene()
        attached = AttachedCollisionObject(link_name='manipulator_gripper_shell',
            touch_links=['manipulator_finger1', 'manipulator_finger2', 'manipulator_gripper_shell', 'manipulator_link6'])
        fruit = next(f for f in self.wait_observation()['fruits'] if f['model'] == model)
        stem_shape = stem_dimensions(fruit.get('socket_to_branch_m', .04))
        socket = world_to_base(fruit['socket_in_world'], self.observation['base_in_world'])
        shell = self.fk(link='manipulator_gripper_shell')
        sr = Rotation.from_quat([shell.orientation.x, shell.orientation.y, shell.orientation.z, shell.orientation.w])
        sp = np.array([shell.position.x, shell.position.y, shell.position.z])
        fr = Rotation.from_quat(self.observation['base_in_world'][3:]).inv()*Rotation.from_quat(fruit['orientation_in_world'])
        obj = CollisionObject(id=model, operation=CollisionObject.ADD)
        obj.header.frame_id = 'manipulator_gripper_shell'
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.035]),
                          SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                         dimensions=[stem_shape['collision_length'], .0018])]
        obj.primitive_poses = [pose(sr.inv().apply(socket+fr.apply(offset)-sp), (sr.inv()*fr).as_quat())
                               for offset in ([0., 0., -.032], [0., 0., stem_shape['collision_center']])]
        attached.object = obj
        scene.robot_state.attached_collision_objects = [attached]
        self.apply_scene(scene)

    def pick(self, candidate, stow_after=True):
        fruit, preplan, approach, prepose = candidate
        model = fruit['model']
        self.event('moving_to_pregrasp', model=model)
        self.execute(preplan)
        self.execute(approach)
        for _ in range(4):
            actual = self.fk()
            observed = next(f for f in self.wait_observation()['fruits'] if f['model'] == model)
            error = np.linalg.norm(np.array([actual.position.x, actual.position.y, actual.position.z])-observed['grasp_in_base'])
            self.event('precontact_alignment', model=model, tcp_error_m=float(error))
            if error < .0007:
                break
            quaternion = [actual.orientation.x, actual.orientation.y, actual.orientation.z, actual.orientation.w]
            self.execute(self.cartesian(self.state(), pose(observed['grasp_in_base'], quaternion)))
        if error > .0007:
            raise RuntimeError(f'Actual TCP misses stem by {error:.4f} m')
        self.fingers(.0013)
        hold = [self.joints[n] for n in JOINTS]+[.0013, .0013]
        deadline = time.monotonic()+5.
        pairs = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            now = self.get_clock().now().nanoseconds/1e9
            pairs = {i: [pair for stamp, pair in self.contacts[i] if now-stamp < 1. and
                         any(model+'::fruit::grasp_stem' in part for part in pair)] for i in (1, 2)}
            if all(pairs.values()):
                break
            self.send_samples(JOINTS+FINGERS, [hold], [0.])
        if not all(pairs.values()):
            self.fingers(.0089)
            raise RuntimeError(f'Bilateral stem contact was not confirmed: {pairs}')
        self.event('stem_contacts_confirmed', model=model, tcp_error_m=float(error), contacts=pairs)
        self.transfer_fixture(model)
        self.report['held_fruit'] = model
        self.event('stem_detached_from_tree', model=model)
        self.attach_planning_object(model)
        self.retreat_held(model)
        self.deposit(model)
        self.report.pop('held_fruit', None)
        self.report['harvested'].append(model)
        self.event('fruit_deposited', model=model)
        if stow_after:
            self.stow()

    def stow(self):
        self.event('stowing_arm')
        self.execute(self.plan(self.state(), self.state(STOW)))
        self.event('arm_stowed')

    def retreat_held(self, model):
        observed = next(f for f in self.wait_observation()['fruits'] if f['model'] == model)
        tcp = self.fk()
        quaternion = [tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w]
        stem = np.asarray(observed['stem_axis_in_base'])
        horizontal = Rotation.from_quat(quaternion).apply([0., 0., 1.])
        horizontal -= np.dot(horizontal, stem)*stem
        horizontal /= np.linalg.norm(horizontal)
        origin = np.array([tcp.position.x, tcp.position.y, tcp.position.z])
        for distance, drop in ((.05, .03), (.04, .02), (.04, .04), (.035, -.007)):
            target = origin-distance*horizontal-drop*stem
            try:
                trajectory = self.cartesian(self.state(), pose(target, quaternion))
            except RuntimeError as error:
                self.event('retreat_option_rejected', model=model, horizontal_m=distance,
                           drop_m=drop, reason=str(error))
                continue
            self.event('retreat_selected', model=model, horizontal_m=distance, drop_m=drop)
            self.execute(trajectory)
            return
        raise RuntimeError('No complete collision-checked retreat with the held fruit')

    def deposit(self, model):
        observed = next(f for f in self.wait_observation()['fruits'] if f['model'] == model)
        socket = world_to_base(observed['socket_in_world'], self.observation['base_in_world'])
        fr = Rotation.from_quat(self.observation['base_in_world'][3:]).inv()*Rotation.from_quat(observed['orientation_in_world'])
        body = socket+fr.apply([0., 0., -.032])
        tcp = self.fk()
        tr = Rotation.from_quat([tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w])
        tp = np.array([tcp.position.x, tcp.position.y, tcp.position.z])
        offset = tr.inv().apply(body-tp)
        down = Rotation.from_euler('x', np.pi)
        slot = [-.06, -.08+.08*len(self.report['harvested']), .38]
        target = np.array(slot)-down.apply(offset)
        state = self.ik(target, down.as_quat(), self.state())
        self.execute(self.plan(self.state(), state))
        self.fingers(.0089)
        result = self.call('/DETACHLINK', DetachLink, DetachLink.Request(
            model1_name='hil_ackermann', link1_name='manipulator_gripper_shell', model2_name=model, link2_name='fruit'))
        if not result.success:
            raise RuntimeError(result.message)
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name='manipulator_gripper_shell', object=CollisionObject(id=model, operation=CollisionObject.REMOVE))]
        self.apply_scene(scene)
        end = self.get_clock().now().nanoseconds/1e9+3.
        deadline = time.monotonic()+20.
        while self.get_clock().now().nanoseconds/1e9 < end:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock stopped during fruit release')
            rclpy.spin_once(self, timeout_sec=.03)
        observation = self.wait_observation()
        fruit = next(f for f in observation['fruits'] if f['model'] == model)
        rotation = Rotation.from_quat(fruit['orientation_in_world'])
        center = world_to_base(np.array(fruit['socket_in_world'])+rotation.apply([0., 0., -.032]), observation['base_in_world'])
        self.event('released_fruit_position', model=model, center_in_base=center.tolist())
        if not (-.15 < center[0] < .03 and abs(center[1]) < .14 and .22 < center[2] < .32):
            raise RuntimeError('Released fruit is not inside the physical cargo tray')
        # A deposited fruit is carried by the base in the planning model only.
        # Gazebo still simulates its free rigid body against the real tray.
        obj = CollisionObject(id=model, operation=CollisionObject.ADD,
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.035])],
            primitive_poses=[pose(center)])
        obj.header.frame_id = 'base_link'
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name='base_link', object=obj, touch_links=['base_link'])]
        self.apply_scene(scene)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path.home()/'agribot_hil_ws/scenario')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--maximum', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.maximum <= 3:
        parser.error('--maximum must be 1..3 (tray capacity)')
    rclpy.init()
    node = Harvester(args.scenario, args.output)
    try:
        if args.execute:
            node.parking_fixture(True)
        node.report['status'] = 'selecting'
        while len(node.report['harvested']) < args.maximum:
            candidates = node.candidates()
            if not candidates:
                node.report['status'] = 'no_reachable_fruit'
                break
            if not args.execute:
                node.report.update(status='planned', reachable=[c[0]['model'] for c in candidates])
                break
            node.report['executed'] = True
            node.pick(candidates[0])
        if args.execute:
            node.report['status'] = 'succeeded' if node.report['harvested'] else 'no_reachable_fruit'
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.report['status'] = 'cancelled'
    except Exception as error:
        node.report.update(status='failed', error=str(error))
        raise
    finally:
        if node.parking_engaged and not node.report.get('held_fruit') and rclpy.ok():
            try:
                node.parking_fixture(False)
            except Exception as error:
                node.report.update(status='failed', parking_release_error=str(error))
        node.event('finished', status=node.report['status'])
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
