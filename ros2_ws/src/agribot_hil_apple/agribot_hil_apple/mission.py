"""Run the original-spawn, CAN-only orchard mission on the X5 and retain evidence."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from scipy.spatial.transform import Rotation

from .harvest import Harvester, world_to_base
from .arm_preflight import JOINTS, STOW
from .navigation import Navigator, PARAMETER_UPDATE_TIMEOUT


def resource_snapshot():
    memory = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemAvailable', 'MemTotal'):
            memory[key+'_kib'] = int(value.split()[0])
    processes = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            command = (directory/'cmdline').read_bytes().split(b'\0')
            name = Path(command[0].decode()).name
            if name not in ('fastlivo_mapping', 'move_group', 'controller_server', 'planner_server', 'republish'):
                continue
            fields = dict(line.split(':', 1) for line in (directory/'status').read_text().splitlines() if ':' in line)
            processes.append({'pid': int(directory.name), 'name': name,
                              'rss_kib': int(fields.get('VmRSS', '0 kB').split()[0])})
        except (OSError, ValueError, UnicodeError):
            continue
    return {**memory, 'processes': processes}


class ResourceSampler:
    def __init__(self):
        self.samples, self.errors = [], []
        self.start = time.monotonic()
        self.done = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def run(self):
        while not self.done.is_set():
            try:
                self.samples.append({'wall_elapsed_sec': time.monotonic()-self.start, **resource_snapshot()})
            except OSError as error:
                self.errors.append(str(error))
            self.done.wait(2.)

    def close(self):
        self.done.set()
        self.worker.join(timeout=5.)
        peaks = {}
        for sample in self.samples:
            for process in sample['processes']:
                name = process['name']
                peaks[name] = max(peaks.get(name, 0), process['rss_kib'])
        return {'samples': self.samples, 'peak_rss_kib_by_process_name': peaks, 'errors': self.errors}


def odometry_pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


class Mission:
    def __init__(self, scenario, output):
        self.scenario, self.output = scenario, output
        self.manifest = json.loads((scenario/'mission.json').read_text())
        self.navigator = Navigator()
        self.harvester = Harvester(scenario, output/'harvest.json')
        self.started = time.monotonic()
        self.resources = ResourceSampler()
        self.report = {'status': 'initializing', 'executed': False,
                       'manifest': self.manifest, 'events': [], 'navigation': []}

    def event(self, name, **values):
        sim_time = max(node.get_clock().now().nanoseconds/1e9
                       for node in (self.navigator, self.harvester))
        self.report['events'].append({'event': name, 'wall_elapsed_sec': time.monotonic()-self.started,
            'sim_time': sim_time, **values})
        self.save()
        if getattr(self, 'stdout_available', True):
            try:
                print(name+': '+json.dumps(values), flush=True)
            except BrokenPipeError:
                self.stdout_available = False

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output/'mission.json').write_text(json.dumps(self.report, indent=2)+'\n')

    def navigate(self, phase, points):
        nav = self.navigator
        nav.begin_route(points)
        last_progress = np.array([nav.pose.pose.pose.position.x, nav.pose.pose.pose.position.y])
        stalled = 0
        deadline = time.monotonic()+1800.
        for attempt in range(32):
            remaining = points[nav.route_index:]
            entry = {'phase': phase, 'attempt': attempt+1, 'first_remaining_index': nav.route_index,
                     'requested_goals': remaining, 'status': 'planning'}
            self.report['navigation'].append(entry)
            self.event('planning', phase=phase, attempt=attempt+1, first_remaining_index=nav.route_index)
            try:
                path = nav.plan(remaining)
                entry['path'] = [[p.pose.position.x, p.pose.position.y, p.pose.orientation.z,
                                  p.pose.orientation.w] for p in path.poses]
                entry['status'] = 'following'
                self.event('following', phase=phase, path_points=len(path.poses))
                nav.follow(path)
                nav.stop()
                entry.update(status='succeeded', final_fused_pose=odometry_pose(nav.pose),
                             completed_via_index=nav.route_index,
                             via_passage=list(getattr(nav, 'route_passage_events', [])))
                self.event('navigation_completed', phase=phase, final_fused_pose=entry['final_fused_pose'])
                return
            except (RuntimeError, TimeoutError) as error:
                nav.stop()
                entry.update(status='failed', error=str(error), completed_via_index=nav.route_index,
                             via_passage=list(getattr(nav, 'route_passage_events', [])))
                current = np.array([nav.pose.pose.pose.position.x, nav.pose.pose.pose.position.y])
                progress = float(np.linalg.norm(current-last_progress))
                stalled = 0 if progress >= .5 else stalled+1
                if progress >= .5:
                    last_progress = current
                self.event('navigation_attempt_failed', phase=phase, error=str(error),
                           progress_since_checkpoint_m=progress, consecutive_stalled_attempts=stalled)
                if stalled >= 4 or attempt == 31 or time.monotonic() >= deadline:
                    raise
                self.wait_sim(3.)

    def wait_sim(self, seconds):
        deadline = time.monotonic()+max(20., seconds*5.)
        initial = self.navigator.get_clock().now().nanoseconds
        # Manipulation spins a different node; consume a fresh clock before timing.
        while self.navigator.get_clock().now().nanoseconds == initial:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock stopped before mission settling')
            rclpy.spin_once(self.navigator, timeout_sec=.05)
        end = self.navigator.get_clock().now().nanoseconds/1e9+seconds
        while self.navigator.get_clock().now().nanoseconds/1e9 < end:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock stopped during mission settling')
            rclpy.spin_once(self.navigator, timeout_sec=.05)

    def navigate_profile(self, phase, points, server, profile):
        nav = self.navigator
        self.event('navigation_profile_requested', phase=phase, server=server, parameters=profile)
        client, previous = nav.parameter_profile(server, profile)
        try:
            self.event('navigation_profile', phase=phase, server=server, parameters=profile)
            self.navigate(phase, points)
        finally:
            try:
                if rclpy.ok():
                    result = nav.wait(client.call_async(SetParametersAtomically.Request(parameters=previous)),
                                      PARAMETER_UPDATE_TIMEOUT)
                    if not result.result.successful:
                        raise RuntimeError('Failed to restore navigation parameters: '+result.result.reason)
                    self.event('navigation_profile_restored', phase=phase, server=server)
                else:
                    self.event('navigation_profile_restore_deferred', phase=phase, server=server,
                               reason='ROS context shut down; restart with baseline configuration before reuse')
            finally:
                nav.destroy_client(client)

    def finish_route(self):
        phase = 'completion' if 'completion_via' in self.manifest else 'return'
        self.navigate(phase, self.manifest[phase+'_via'])

    def run(self, execute, maximum, minimum):
        nav, arm = self.navigator, self.harvester
        nav.wait_ready(timeout=90.)
        nav.wait_active('planner_server')
        nav.wait_active('controller_server')
        observation = arm.wait_observation()
        self.report['initial_fused_pose'] = odometry_pose(nav.pose)
        self.report['initial_observed_base'] = observation['base_in_world']
        spawn = self.manifest['spawn']
        if math.dist(observation['base_in_world'][:2], spawn[:2]) > .25:
            raise RuntimeError('Full mission must start at the original spawn, not at a commissioning parking pose')
        if not execute:
            path = nav.plan(self.manifest['outbound_via'])
            self.report.update(status='planned_no_execution', planned_outbound_points=len(path.poses))
            return
        self.report.update(status='running', executed=True)
        arm.report.update(status='picking', executed=True)
        arm.transit_hold(True)
        stops = self.manifest.get('pickup_stops', [{'park': self.manifest['park']}])
        for index, stop in enumerate(stops):
            if len(arm.report['harvested']) >= maximum:
                break
            points = self.manifest['outbound_via'][:-1]+[stop['park']] if index == 0 else [stop['park']]
            self.navigate_profile('outbound' if index == 0 else 'pickup_reposition', points,
                '/controller_server', {'goal_checker.xy_goal_tolerance':
                    self.manifest.get('pickup_position_tolerance_m', .10),
                    'goal_checker.yaw_goal_tolerance': self.manifest.get('pickup_yaw_tolerance_rad', .3),
                    'FollowPath.GoalAngleCritic.enabled': True,
                    'FollowPath.PathAlignCritic.use_path_orientations': True})
            self.wait_sim(2.)
            arm.wait_observation()
            arm.parking_fixture(True)
            try:
                arm.transit_hold(False)
                while len(arm.report['harvested']) < maximum:
                    if 'target_models' in stop and set(stop['target_models']) <= set(arm.report['harvested']):
                        break
                    candidates = arm.candidates(models=set(stop['target_models']) if 'target_models' in stop else None,
                                                maximum=1)
                    if not candidates:
                        break
                    arm.pick(candidates[0], stow_after=False)
                arm.stow()
                arm.transit_hold(True)
            finally:
                if arm.parking_engaged and not arm.report.get('held_fruit'):
                    arm.parking_fixture(False)
            self.event('pickup_stop_completed', index=index, harvested=list(arm.report['harvested']))
            self.wait_sim(6.)
        if not arm.report['harvested']:
            raise RuntimeError('No fruit could be picked from a collision-free reachable target')
        arm.report['status'] = 'succeeded'
        arm.event('finished', status=arm.report['status'])
        self.report['harvested'] = list(arm.report['harvested'])
        self.event('harvesting_completed', harvested=self.report['harvested'])
        # Let genuine short-lived arm self-returns decay before resuming travel.
        self.wait_sim(6.)
        self.finish_route()
        self.wait_sim(2.)
        final = arm.wait_observation()
        self.report['final_observed_base'] = final['base_in_world']
        self.report['final_fused_pose'] = odometry_pose(nav.pose)
        finish = self.manifest.get('finish', spawn[:2]+[spawn[3]])
        self.report['observed_finish_position_error_m'] = math.dist(final['base_in_world'][:2], finish[:2])
        yaw = Rotation.from_quat(final['base_in_world'][3:]).as_euler('xyz')[2]
        self.report['observed_finish_yaw_error_rad'] = abs(math.atan2(math.sin(yaw-finish[2]), math.cos(yaw-finish[2])))
        self.report['final_arm_positions'] = {name: arm.joints[name] for name in JOINTS}
        self.report['final_cargo'] = []
        for model in self.report['harvested']:
            fruit = next(f for f in final['fruits'] if f['model'] == model)
            center = world_to_base(np.array(fruit['socket_in_world'])+
                Rotation.from_quat(fruit['orientation_in_world']).apply([0., 0., -.032]), final['base_in_world'])
            inside = bool(-.15 < center[0] < .03 and abs(center[1]) < .14 and .22 < center[2] < .32)
            self.report['final_cargo'].append({'model': model, 'center_in_base': center.tolist(), 'inside_tray': inside})
        if self.report['observed_finish_position_error_m'] > .3:
            raise RuntimeError('Observed vehicle did not finish within 0.3 m of the requested final point')
        if max(abs(arm.joints[name]-value) for name, value in zip(JOINTS, STOW)) > .02:
            raise RuntimeError('Arm is not stowed at the end of the mission')
        if not all(fruit['inside_tray'] for fruit in self.report['final_cargo']):
            raise RuntimeError('A harvested fruit was lost from the physical tray during the return')
        if len(self.report['harvested']) < minimum:
            raise RuntimeError(f"Returned with only {len(self.report['harvested'])} fruit; required minimum is {minimum}")
        self.report['status'] = 'succeeded'

    def close(self):
        try:
            if rclpy.ok():
                self.navigator.stop()
        finally:
            self.report['resources'] = self.resources.close()
            self.save()
            self.harvester.destroy_node()
            self.navigator.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path.home()/'agribot_hil_ws/scenario')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--maximum-fruit', type=int, default=3)
    parser.add_argument('--minimum-fruit', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.minimum_fruit <= args.maximum_fruit <= 3:
        parser.error('Require 1 <= minimum-fruit <= maximum-fruit <= 3')
    rclpy.init()
    mission = Mission(args.scenario, args.output)
    try:
        mission.run(args.execute, args.maximum_fruit, args.minimum_fruit)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        mission.report['status'] = 'cancelled'
    except Exception as error:
        mission.report.update(status='failed', error=str(error))
        raise
    finally:
        mission.event('finished', status=mission.report['status'])
        mission.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
