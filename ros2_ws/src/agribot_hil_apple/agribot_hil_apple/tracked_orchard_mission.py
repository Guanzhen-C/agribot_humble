"""Native Smac/MPPI orchard route with leased implement-ECU spray commands."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .navigation import ControllerPathError, Navigator


def spray_allowed(config, x, y, yaw, speed, ready):
    return bool(ready and abs(speed) > config['minimum_speed_mps'] and
                config['x_range'][0] <= x <= config['x_range'][1] and
                any(abs(y-row) <= config['row_half_width_m'] for row in config['row_y']) and
                abs(math.sin(yaw)) < .3)


class OrchardMission(Navigator):
    def __init__(self, manifest, output):
        super().__init__()
        self.manifest, self.output = manifest, output
        self.implement = manifest.get('implement_kind', 'sprayer')
        self.work_config = manifest.get('shredding') or manifest['spraying']
        self.work_joints = (['work_shredder_joint'] if self.implement == 'shredder' else
                            ['work_fan_joint', 'work_pump_joint'])
        self.work_pub = self.create_publisher(String, '/hil/work/command', 1)
        self.bridge = {}
        self.gateway_ready_at = self.bridge_at = self.wheel_at = -math.inf
        self.wheel_speed = 0.
        self.executing = False
        self.started = time.monotonic()
        self.last_report = 0.
        self.actual_work = [0.] * len(self.work_joints)
        self.truth = None
        self.report = dict(manifest=manifest, status='waiting_for_localization', navigation=[],
                           work_events=[], actual_work=[], truth=[], fused=[], actual_native_can_required=True,
                           actual_work_joint_names=self.work_joints)
        self.last_work = None
        self.create_subscription(String, '/hil/tracked_can/status', self.bridge_status, 10)
        self.create_subscription(String, '/hil/isobus/events', self.isobus_status, 100)
        self.create_subscription(Odometry, '/hil/wheel/odometry', self.wheel, 10)
        self.create_subscription(Odometry, '/hil/ground_truth', self.ground_truth, 5)
        self.create_subscription(JointState, '/hil/work/joint_states', self.work_state, 10)
        self.create_timer(.1, self.tick_work, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def bridge_status(self, msg):
        self.bridge = json.loads(msg.data)
        self.bridge_at = time.monotonic()

    def isobus_status(self, msg):
        data = json.loads(msg.data)
        if data.get('kind') == 'gateway':
            value = bytes.fromhex(data['data'])
            mask = 4 if self.implement == 'shredder' else 3
            if value[0] == 1 and value[1] and value[2] & mask == mask:
                self.gateway_ready_at = time.monotonic()

    def wheel(self, msg):
        self.wheel_speed, self.wheel_at = msg.twist.twist.linear.x, time.monotonic()

    def ground_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = [p.x, p.y, p.z]

    def work_state(self, msg):
        values = dict(zip(msg.name, msg.velocity))
        self.actual_work = [values.get(name, 0.) for name in self.work_joints]

    def tick_work(self):
        now = time.monotonic()
        ready = (self.executing and self.ready and now-self.last_pose_receipt < 1.5 and
                 now-self.gateway_ready_at < 1 and now-self.wheel_at < .6 and now-self.bridge_at < 2 and
                 self.bridge.get('work_authorized', False) and not self.bridge.get('fault', 'missing'))
        active = False
        if self.pose is not None:
            p, q = self.pose.pose.pose.position, self.pose.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            active = spray_allowed(self.work_config, p.x, p.y, yaw, self.wheel_speed, ready)
        config = self.work_config
        command = dict(fan_gear=config.get('fan_gear', 0) if active else 0,
                       pump_gear=config.get('pump_gear', 0) if active else 0,
                       shredder_level=config.get('shredder_level', 0) if active else 0)
        self.work_pub.publish(String(data=json.dumps(command)))
        if active != self.last_work:
            self.report['work_events'].append(dict(wall=now-self.started, active=active,
                xy=([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y] if self.pose else None)))
            self.last_work = active
        if now-self.last_report >= .5:
            self.last_report = now
            elapsed = now-self.started
            self.report['actual_work'].append([elapsed, *self.actual_work])
            if self.truth:
                self.report['truth'].append([elapsed, *self.truth])
            if self.pose:
                p = self.pose.pose.pose.position
                self.report['fused'].append([elapsed, p.x, p.y, p.z])
            self.persist()

    def persist(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, indent=2)+'\n')
        temporary.replace(self.output)

    def run(self):
        self.wait_ready(90.)
        deadline = time.monotonic()+30
        while time.monotonic()-self.gateway_ready_at > 1 or not self.bridge.get('motion_authorized', False):
            if time.monotonic() > deadline:
                raise TimeoutError('CAN implement ECU or PC motion authorization not ready')
            rclpy.spin_once(self, timeout_sec=.1)
        self.navigate_route()
        self.executing = False
        self.stop()
        end = self.manifest['finish']
        if self.truth is None:
            raise RuntimeError('Independent ground truth unavailable at finish')
        error = math.hypot(self.truth[0]-end[0], self.truth[1]-end[1])
        self.report.update(independent_endpoint_error_m=error,
                           route_passage_events=self.route_passage_events)
        if error > self.manifest['goal_tolerance_m']+.10:
            raise RuntimeError('Actual endpoint outside configured tolerance plus 10 cm localization margin')
        if not any(all(speed > 50 for speed in sample[1:]) for sample in self.report['actual_work']):
            raise RuntimeError('No actual mounted-implement shaft rotation recorded')
        self.report['status'] = 'succeeded'
        self.persist()

    def navigate_route(self):
        points = self.manifest['fixed_route_points']
        self.begin_route(points)
        # No periodic replanning: only an ended/explicitly cancelled native action
        # can request a new Smac path, retaining every not-yet-passed original via.
        for attempt in range(7):
            self.executing = False
            self.report['status'] = 'planning' if attempt == 0 else 'replanning_after_failure'
            self.persist()
            remaining = points[self.route_index:]
            path = self.plan(remaining)
            entry = dict(phase='outbound', attempt=attempt, planner='Smac Hybrid-A* on X5',
                         controller='MPPI DiffDrive on X5',
                         path=[[p.pose.position.x, p.pose.position.y, p.pose.orientation.z, p.pose.orientation.w]
                               for p in path.poses], requested_goals=remaining)
            self.report['navigation'].append(entry)
            self.report['status'] = 'executing'
            self.persist()
            self.executing = True
            try:
                self.follow(path, timeout=900.)
            except ControllerPathError as error:
                self.executing = False
                self.stop()
                entry.update(executed=False, error=str(error))
                self.persist()
                if attempt == 6:
                    raise
            else:
                entry['executed'] = True
                return

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', type=Path, default=Path.home()/'agribot_isobus_hil_ws/scenario_orchard/mission.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OrchardMission(json.loads(args.mission.read_text()), args.output)
    try:
        node.run()
    except KeyboardInterrupt:
        node.report.update(status='interrupted', error='Operator interrupted the mission')
    except Exception as error:
        node.report.update(status='failed', error=str(error))
        raise
    finally:
        node.executing = False
        try:
            if rclpy.ok():
                node.stop()
                for _ in range(8):
                    node.work_pub.publish(String(data='{"fan_gear":0,"pump_gear":0,"shredder_level":0}'))
                    rclpy.spin_once(node, timeout_sec=.03)
        finally:
            node.persist()
            node.destroy_node()
            rclpy.try_shutdown()
