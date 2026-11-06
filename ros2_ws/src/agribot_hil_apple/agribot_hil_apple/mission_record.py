"""Independent PC trajectory/CAN evidence; never republishes chassis commands."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .arm_preflight import JOINTS, FINGERS


class Recorder(Node):
    def __init__(self):
        super().__init__('hil_mission_recorder')
        self.clock = 0.
        self.started = time.monotonic()
        self.report = {'source': 'independent observations, not a localization or command source',
                       'counts': {}, 'traces': {}, 'twists': {}, 'diagnostics': {}, 'can': [], 'arm': [],
                       'gripper_hold': []}
        self.last_sample = {}
        self.last_pose = None
        self.report['local_costmap_near_vehicle'] = []
        for topic, kind in [('/clock', Clock), ('/hil/ground_truth', Odometry),
                            ('/fastlivo_rtk/odometry', Odometry), ('/wheel/odometry', Odometry),
                            ('/fastlivo_rtk/ready', Bool),
                            ('/hil/gripper/transit_hold/status', Bool),
                            ('/hil/can/status', String), ('/joint_states', JointState), ('/diagnostics', DiagnosticArray),
                            ('/local_costmap/costmap', OccupancyGrid)]:
            self.create_subscription(kind, topic, lambda msg, name=topic: self.receive(name, msg), qos_profile_sensor_data)

    def receive(self, name, message):
        counts = self.report['counts']
        counts[name] = counts.get(name, 0)+1
        if isinstance(message, Clock):
            self.clock = message.clock.sec+message.clock.nanosec*1e-9
        elif isinstance(message, Odometry):
            if name == '/fastlivo_rtk/odometry':
                self.last_pose = message.pose.pose.position
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if stamp-self.last_sample.get(name, -1.) < .2:
                return
            self.last_sample[name] = stamp
            p, q = message.pose.pose.position, message.pose.pose.orientation
            self.report['traces'].setdefault(name, []).append([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            v, w = message.twist.twist.linear, message.twist.twist.angular
            self.report['twists'].setdefault(name, []).append([stamp, v.x, v.y, v.z, w.x, w.y, w.z])
        elif isinstance(message, Bool):
            if name == '/hil/gripper/transit_hold/status':
                self.report['gripper_hold'].append([self.clock, message.data])
                return
            key = 'ready_true' if message.data else 'ready_false'
            counts[key] = counts.get(key, 0)+1
        elif isinstance(message, OccupancyGrid) and self.last_pose is not None:
            if self.clock-self.last_sample.get('grid', -1.) < .2:
                return
            self.last_sample['grid'] = self.clock
            info, p = message.info, self.last_pose
            grid = np.asarray(message.data).reshape(info.height, info.width)
            iy, ix = np.where(grid >= 99)
            dx = info.origin.position.x+(ix+.5)*info.resolution-p.x
            dy = info.origin.position.y+(iy+.5)*info.resolution-p.y
            near = np.flatnonzero(dx*dx+dy*dy < 4.)
            near = near[np.argsort(dx[near]*dx[near]+dy[near]*dy[near])][:40]
            self.report['local_costmap_near_vehicle'].append({'sim_time': self.clock,
                'pose_xy': [p.x, p.y], 'nearest_lethal_offsets': np.column_stack([dx[near], dy[near]]).tolist()})
        elif isinstance(message, String):
            self.report['can'].append({'sim_time': self.clock, **json.loads(message.data)})
        elif isinstance(message, JointState):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if not all(n in message.name for n in JOINTS+FINGERS) or stamp-self.last_sample.get('arm', -1.) < .04:
                return
            self.last_sample['arm'] = stamp
            values = dict(zip(message.name, message.position))
            self.report['arm'].append([stamp, *[values[n] for n in JOINTS+FINGERS]])
        elif isinstance(message, DiagnosticArray):
            for status in message.status:
                self.report['diagnostics'][status.name] = {'message': status.message,
                    'values': {value.key: value.value for value in status.values}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=1800.)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder()
    try:
        while time.monotonic()-node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        node.report.update(final_sim_time=node.clock, wall_duration_sec=time.monotonic()-node.started)
        args.output.write_text(json.dumps(node.report, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
