"""X5 Smac/MPPI work mission over the physical 6750 hitch and drive chain."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters


class Mission(Node):
    def __init__(self, output, transmission='off'):
        super().__init__('hitch_hil_mission', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output = output
        self.record = (output/'samples.jsonl').open('x')
        self.report = dict(scope='Physical X5/6750/USB-CAN HIL', passed=False, stages=[])
        self.pose = self.truth = self.plant = None
        self.power_plant = self.power_status = None
        self.pose_at = -math.inf
        self.hitch_target = dict(mode='hold', position=0, speed=0)
        self.transmission = transmission
        self.record_powertrain = transmission != 'off'
        self.power_publisher = self.create_publisher(String, '/hil_hitch/powertrain/command', 1)
        self.hitch_publisher = self.create_publisher(String, '/hil_hitch/command', 1)
        self.path_publisher = self.create_publisher(NavPath, '/hil_hitch/planned_path', 1)
        self.create_subscription(Odometry, '/odometry/filtered_navsat', self.odometry, 10)
        self.create_subscription(Odometry, '/hil_hitch/ground_truth', lambda msg: setattr(self, 'truth', msg), 10)
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64MultiArray, '/hil_hitch/plant_state', self.measurement, feedback_qos)
        self.create_subscription(JointState, '/hil_hitch/joint_states', self.joints, 10)
        self.create_subscription(Float64MultiArray, '/hil_hitch/powertrain/plant_state',
            lambda msg: setattr(self, 'power_plant', list(msg.data)), feedback_qos)
        self.create_subscription(String, '/hil_hitch/powertrain/status',
            lambda msg: setattr(self, 'power_status', json.loads(msg.data)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.params = self.create_client(SetParameters, '/hil_hitch_rdk_client/set_parameters')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.follower = ActionClient(self, FollowPath, '/follow_path')
        self.active_goal = None
        self.disc_speed = 0.
        self.current_stage = None
        self.create_timer(.05, lambda: self.hitch_publisher.publish(String(data=json.dumps(self.hitch_target))))
        self.create_timer(.05, self.publish_powertrain)

    def publish_powertrain(self):
        self.power_publisher.publish(String(data=json.dumps(dict(mode=self.transmission, gear=0,
            engine_rpm=1600, ratio=0, shift_ms=600, ratio_rate=150))))

    def odometry(self, msg):
        self.pose, self.pose_at = msg, time.monotonic()

    def joints(self, msg):
        self.disc_speed = max((abs(v) for n, v in zip(msg.name, msg.velocity) if n.startswith('hitch_disc_')), default=0.)

    def measurement(self, msg):
        self.plant = list(msg.data)
        sample = dict(at=time.monotonic(), plant=self.plant, disc_speed=self.disc_speed,
                      commanded_transmission=self.transmission)
        if self.record_powertrain:
            sample.update(powertrain=self.power_plant, isobus=self.power_status)
        if self.pose is not None:
            p = self.pose.pose.pose.position
            sample['estimated_xy'] = [p.x, p.y]
        if self.truth is not None:
            p = self.truth.pose.pose.position
            sample['true_xy'] = [p.x, p.y]
        self.record.write(json.dumps(sample)+'\n')
        if self.current_stage is not None:
            self.current_stage['samples'] += 1
            if self.plant[2] == 1 and self.disc_speed > .1:
                self.current_stage['engaged_samples'] += 1
            if self.transmission != 'off' and self.power_plant and self.power_plant[19] == 1:
                self.current_stage['transmission_samples'] += 1

    def pump(self, seconds, predicate=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.025)
            if predicate is not None and predicate():
                return True
        return False

    def wait(self, future, seconds):
        if not self.pump(seconds, future.done):
            raise TimeoutError('Mission operation timed out')
        return future.result()

    def authorize(self, value):
        if not self.params.wait_for_service(timeout_sec=30.):
            raise RuntimeError('X5 physical client unavailable')
        request = SetParameters.Request(parameters=[Parameter(name, value=value).to_parameter_msg()
            for name in ('drive_authorized', 'work_authorized')])
        result = self.wait(self.params.call_async(request), 8.)
        if not all(item.successful for item in result.results):
            raise RuntimeError('Authorization refused')

    def lift(self, position):
        self.hitch_target = dict(mode='position', position=position, speed=80)
        if not self.pump(35., lambda: self.plant is not None and
                         abs(self.plant[0]-position) < 150 and abs(self.plant[1]) < 150):
            raise TimeoutError('6750 hitch did not reach requested position')

    def wait_active(self, name):
        client = self.create_client(GetState, '/'+name+'/get_state')
        try:
            if not client.wait_for_service(timeout_sec=15.):
                raise RuntimeError(name+' lifecycle service unavailable')
            deadline = time.monotonic()+90.
            while time.monotonic() < deadline:
                state = self.wait(client.call_async(GetState.Request()), 8.).current_state
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    return
                self.pump(.3)
            raise TimeoutError(name+' did not become active')
        finally:
            self.destroy_client(client)
    def navigate(self, goal, final):
        for client in (self.planner, self.follower):
            if not client.wait_for_server(timeout_sec=15.):
                raise RuntimeError('X5 Nav2 action unavailable')
        request = ComputePathToPose.Goal()
        request.planner_id = 'GridBased'
        request.use_start = False
        request.goal = PoseStamped()
        request.goal.header.frame_id = 'map'
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = goal['x'], goal['y']
        request.goal.pose.orientation.z = math.sin(goal['yaw']/2)
        request.goal.pose.orientation.w = math.cos(goal['yaw']/2)
        handle = self.wait(self.planner.send_goal_async(request), 10.)
        if not handle.accepted:
            raise RuntimeError('Smac rejected goal')
        self.active_goal = handle
        result = self.wait(handle.get_result_async(), 90.)
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
            raise RuntimeError(f'Smac failed: {result.status}')
        path = result.result.path
        points = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        (self.output/(goal['name']+'-smac-path.json')).write_text(json.dumps(points)+'\n')
        self.path_publisher.publish(path)
        length = sum(math.hypot(a[0]-b[0], a[1]-b[1]) for a, b in zip(points, points[1:]))
        self.current_stage.update(path_length_m=length, planned_points=len(points), action='following')
        self.save()
        request = FollowPath.Goal(path=path, controller_id='FollowPath',
                                 goal_checker_id='goal_checker' if final else 'work_goal_checker')
        handle = self.wait(self.follower.send_goal_async(request), 10.)
        if not handle.accepted:
            raise RuntimeError('MPPI rejected path')
        self.active_goal = handle
        result = self.wait(handle.get_result_async(), max(180., length/.15))
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'MPPI failed: {result.status}')
        p = self.pose.pose.pose.position
        self.current_stage.update(action='completed', endpoint_error_m=math.hypot(p.x-goal['x'], p.y-goal['y']))

    def save(self):
        (self.output/'result.json').write_text(json.dumps(self.report, indent=2)+'\n')
        self.record.flush()

    def run(self, manifest):
        self.report['manifest'] = manifest
        self.report['transmission'] = self.transmission
        self.authorize(False)
        client = self.create_client(GetParameters, '/hil_hitch_rdk_client/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError('X5 parameter service unavailable')
            result = self.wait(client.call_async(GetParameters.Request(names=['powertrain_enabled'])), 8.)
            if result.values[0].bool_value != (self.transmission != 'off'):
                raise RuntimeError('Mission transmission selection does not match the X5/scene profile')
        finally:
            self.destroy_client(client)
        client = self.create_client(GetParameters, '/hil_hitch_native_plant/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError('PC physical plant parameter service unavailable')
            values = self.wait(client.call_async(GetParameters.Request(names=['physical_can', 'powertrain_force_plant'])), 8.).values
            if not values[0].bool_value or values[1].bool_value != (self.transmission != 'off'):
                raise RuntimeError('PC scene must match the selected physical transmission profile')
        finally:
            self.destroy_client(client)
        if not self.pump(20., lambda: self.pose is not None and time.monotonic()-self.pose_at < .8 and self.plant is not None):
            raise RuntimeError('Fresh localization and plant state required')
        for name in ('map_server', 'planner_server', 'controller_server'):
            self.wait_active(name)
        self.authorize(True)
        self.pump(.6)
        for index, goal in enumerate(manifest['goals']):
            self.current_stage = dict(name=goal['name'], samples=0, engaged_samples=0, transmission_samples=0, action='positioning_hitch')
            self.report['stages'].append(self.current_stage)
            self.save()
            print(json.dumps(dict(stage=goal['name'], state='starting')), flush=True)
            self.lift(goal['height'])
            self.navigate(goal, index == len(manifest['goals'])-1)
            if goal['height'] == 0 and self.current_stage['engaged_samples'] < 10:
                raise AssertionError('Working row did not show physical engagement and disc rotation')
            if self.transmission != 'off' and self.current_stage['transmission_samples'] < 10:
                raise AssertionError('Stage did not show actual transmission engagement')
            self.save()
            print(json.dumps(self.current_stage), flush=True)
        self.report['passed'] = True

    def stop(self):
        if self.active_goal is not None:
            self.wait(self.active_goal.cancel_goal_async(), 8.)
            self.active_goal = None
        self.hitch_target = dict(mode='hold', position=0, speed=0)
        self.transmission = 'off'
        self.authorize(False)
        self.pump(1.)
        self.save()
        self.record.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start', action='store_true', help='Explicitly authorize this isolated physical HIL mission')
    parser.add_argument('--transmission', choices=('off', 'powershift_auto', 'cvt_auto'), default='off')
    args = parser.parse_args()
    if not args.start or os.environ.get('ROS_DOMAIN_ID') != '74':
        raise ValueError('Explicit --start and isolated ROS_DOMAIN_ID=74 required')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Mission(args.output, args.transmission)
    try:
        node.run(json.loads(args.manifest.read_text()))
    except BaseException as error:
        node.report['error'] = str(error)
        raise
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
