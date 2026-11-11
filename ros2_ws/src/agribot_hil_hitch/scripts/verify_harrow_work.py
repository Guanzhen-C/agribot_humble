"""Read-only verification of all disc bottoms and persistent worked-soil samples."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


def main(args):
    if args.output.exists(): raise FileExistsError(args.output)
    if os.environ.get('ROS_DOMAIN_ID') != '74': raise ValueError('Only the isolated hitch bench is permitted')
    rclpy.init(); node = Node('verify_harrow_work_read_only')
    state, depths, samples, roller = {}, {}, [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Float64MultiArray, '/hil_hitch/plant_state',
        lambda m: state.update(values=list(m.data), at=time.monotonic()), qos)
    def depth(message):
        values = list(message.data); now = time.monotonic()
        depths.update(values=values, at=now)
        if len(values) == 16 and state.get('values', [0, 0, 0])[2] == 1 and now-state['at'] < .15:
            samples.append(values)
    node.create_subscription(Float64MultiArray, '/hil_hitch/disc_bottom_z', depth, qos)
    def joints(message):
        for name, speed in zip(message.name, message.velocity):
            if name == 'hitch_cage_roller_joint' and state.get('values', [0, 0, 0])[2] == 1:
                roller.append(speed)
    node.create_subscription(JointState, '/hil_hitch/joint_states', joints, qos)
    result = dict(passed=False, scope='Read-only physical HIL telemetry; not real agronomic soil validation')
    try:
        end = time.monotonic()+args.timeout
        while time.monotonic() < end and len(samples) < 20:
            rclpy.spin_once(node, timeout_sec=.05)
        result['engaged_samples'] = len(samples)
        result['disc_bottom_z_m'] = samples[-1] if samples else depths.get('values')
        result['all_discs_in_soil'] = bool(samples and all(-.15 < z < -.025 for z in samples[-1]))
        result['physical_state'] = state.get('values')
        environment = dict(os.environ, GAZEBO_MASTER_URI='http://127.0.0.1:11358')
        data = subprocess.check_output(['gz', 'topic', '-e', '/gazebo/hitch_farmland/hil_hitch/worked_swaths',
                                        '-d', '2'], text=True, timeout=8, env=environment)
        result['history_poses_observed'] = data.count('pose {')
        result['roller_peak_speed_rad_s'] = max(map(abs, roller), default=0.)
        result['passed'] = len(samples) >= 20 and result['all_discs_in_soil'] and \
            result['history_poses_observed'] > 0 and result['roller_peak_speed_rad_s'] > .05
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(result, indent=2))
    if not result['passed']: raise SystemExit('Harrow work verification failed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=40.)
    main(parser.parse_args())
