"""Prospective MoveIt checks with carried-fruit geometry, without physical commands."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from .arm_preflight import JOINTS, FINGERS
from .harvest import Harvester, pose, world_to_base


def trajectory_metrics(trajectory):
    positions = np.asarray([point.positions for point in trajectory.points])
    last = trajectory.points[-1].time_from_start
    return {'points': len(trajectory.points), 'duration_sim_sec': last.sec+last.nanosec*1e-9,
            'maximum_joint_travel_rad': float(np.abs(np.diff(positions, axis=0)).sum(axis=0).max(initial=0.))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path.home()/'agribot_hil_ws/scenario')
    parser.add_argument('--base-height', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stop-index', type=int, default=0)
    parser.add_argument('--x-offset', type=float, default=0.)
    parser.add_argument('--y-offset', type=float, default=0.)
    parser.add_argument('--yaw-offset', type=float, default=0.)
    args = parser.parse_args()
    if not np.isfinite([args.base_height, args.x_offset, args.y_offset, args.yaw_offset]).all():
        parser.error('Prospective pose must be finite')
    manifest = json.loads((args.scenario/'mission.json').read_text())
    if not 0 <= args.stop_index < len(manifest['pickup_stops']):
        parser.error('Pickup stop index is outside the mission')
    stop = manifest['pickup_stops'][args.stop_index]
    models = set(stop['target_models'])
    rclpy.init()
    node = Harvester(args.scenario, args.output)
    actual = None
    state, fk = node.state, node.fk
    results = []
    try:
        actual = node.wait_observation()
        for subscription in list(node.subscriptions):
            if subscription.topic_name == '/hil/fruit/observations':
                node.destroy_subscription(subscription)
        prospective = copy.deepcopy(actual)
        park = [stop['park'][0]+args.x_offset, stop['park'][1]+args.y_offset,
                stop['park'][2]+args.yaw_offset]
        rotation = Rotation.from_euler('z', park[2])
        prospective['base_in_world'] = [*park[:2], args.base_height, *rotation.as_quat()]
        for fruit in prospective['fruits']:
            orientation = Rotation.from_quat(fruit['orientation_in_world'])
            fruit['grasp_in_base'] = world_to_base(
                np.array(fruit['socket_in_world'])+orientation.apply([0., 0., .02]),
                prospective['base_in_world']).tolist()
            fruit['stem_axis_in_base'] = (rotation.inv()*orientation).apply([0., 0., 1.]).tolist()
        node.observation = prospective
        node.wait_observation = lambda: prospective
        for model in sorted(models):
            candidates = node.candidates(models={model}, maximum=1)
            if not candidates:
                results.append({'model': model, 'status': 'failed', 'error': 'No short collision-free approach'})
                continue
            fruit, plan, approach, _ = candidates[0]
            positions = dict(zip(approach.joint_names, approach.points[-1].positions))
            grasp = state([positions.get(j, node.joints[j]) if j in JOINTS else .0013
                           for j in JOINTS+FINGERS])
            captured = []
            node.state = lambda positions=None, s=grasp: s if positions is None else state(positions)
            node.fk = lambda state=None, link='manipulator_tcp', s=grasp: fk(state=state or s, link=link)
            node.execute = captured.append
            try:
                node.attach_planning_object(model)
                node.retreat_held(model)
                results.append({'model': model, 'status': 'passed', 'pregrasp': trajectory_metrics(plan),
                                'approach': trajectory_metrics(approach), 'retreat': trajectory_metrics(captured[0])})
            except RuntimeError as error:
                results.append({'model': model, 'status': 'failed', 'error': str(error)})
            finally:
                node.state, node.fk = state, fk
                scene = PlanningScene()
                scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                    link_name='manipulator_gripper_shell', object=CollisionObject(id=model, operation=CollisionObject.REMOVE))]
                node.apply_scene(scene)
                if results and results[-1].get('model') == model and results[-1]['status'] == 'passed':
                    slot = [-.06, -.08+.08*len(node.report['harvested']), .245]
                    node.report['harvested'].append(model)
                    obj = CollisionObject(id=model, operation=CollisionObject.ADD,
                        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.035])],
                        primitive_poses=[pose(slot)])
                    obj.header.frame_id = 'base_link'
                    scene = PlanningScene()
                    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                        link_name='base_link', object=obj, touch_links=['base_link'])]
                    node.apply_scene(scene)
                node.environment(prospective)
        passed = {row['model'] for row in results if row['status'] == 'passed'} == models
        node.report.update(status='passed' if passed else 'failed', executed=False,
                           prospective_base=prospective['base_in_world'], results=results,
                           prospectively_picked_models=list(node.report['harvested']),
                           scope='Prospective native collision checks only; no physical arm, attachment or chassis commands')
        node.event('preflight_finished')
        if not passed:
            raise RuntimeError('Not all target fruit passed approach and attached-fruit retreat')
    finally:
        node.state, node.fk = state, fk
        if actual is not None:
            scene = PlanningScene()
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name='base_link', object=CollisionObject(id=model, operation=CollisionObject.REMOVE))
                for model in models]
            node.apply_scene(scene)
            node.report['harvested'] = []
            node.observation = actual
            node.environment(actual)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
