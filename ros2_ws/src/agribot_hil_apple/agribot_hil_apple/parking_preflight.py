"""Collision-check prospective fruit parking poses without commanding motion."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from .harvest import Harvester, world_to_base
from .navigation import Navigator


def prospective_observation(manifest, park, base_height=.04):
    rotation = Rotation.from_euler('z', park[2])
    base = [park[0], park[1], base_height, *rotation.as_quat()]
    fruits = []
    for index, socket in enumerate(manifest['apples'], 1):
        model = f'harvest_apple_{index}'
        fruits.append({'model': model, 'socket_in_world': socket,
            'orientation_in_world': [0., 0., 0., 1.],
            'grasp_in_base': world_to_base(np.array(socket)+[0., 0., .02], base).tolist(),
            'stem_axis_in_base': rotation.inv().apply([0., 0., 1.]).tolist(),
            'socket_to_branch_m': manifest.get('fruit_socket_to_branch_m', {}).get(model, .04)})
    return {'base_in_world': base, 'base_speed': 0., 'base_yaw_rate': 0., 'fruits': fruits}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path.home()/'agribot_hil_ws/scenario')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stop', type=int, default=0)
    parser.add_argument('--models', nargs='+', help='Optional candidate subset for this read-only check')
    parser.add_argument('--park', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='Override the prospective parking pose; never moves the vehicle')
    parser.add_argument('--check-navigation', action='store_true',
                        help='Require a native Smac path from the live fused pose before checking the arm')
    args = parser.parse_args()
    manifest = json.loads((args.scenario/'mission.json').read_text())
    stop = manifest['pickup_stops'][args.stop]
    rclpy.init()
    node = Harvester(args.scenario, args.output)
    navigator = None
    try:
        node.wait_observation()
        park = args.park or stop['park']
        node.report.update(status='prospective_no_execution', prospective_park=park)
        if args.check_navigation:
            navigator = Navigator()
            try:
                path = navigator.plan([park])
            except (RuntimeError, TimeoutError) as error:
                node.report.update(status='prospective_navigation_failed', navigation_error=str(error))
                return
            node.report['native_smac_path'] = [[p.pose.position.x, p.pose.position.y,
                p.pose.orientation.z, p.pose.orientation.w] for p in path.poses]
        observation = prospective_observation(manifest, park)
        node.wait_observation = lambda: copy.deepcopy(observation)
        accepted = node.candidates(models=set(args.models or stop['target_models']))
        node.report['reachable'] = [candidate[0]['model'] for candidate in accepted]
    finally:
        node.event('finished', status=node.report['status'])
        if navigator is not None:
            navigator.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
