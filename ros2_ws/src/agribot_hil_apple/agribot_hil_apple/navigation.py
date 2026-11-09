"""Request real X5 Smac paths and optionally execute them through MPPI/CAN."""
import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathThroughPoses, FollowPath
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import Bool


PARAMETER_UPDATE_TIMEOUT = 120.


class ControllerPathError(RuntimeError):
    """The native controller ended unsuccessfully; a fresh path may be tried."""


def advance_via_index(points, index, x, y, radius=.6):
    # Only intermediate points may be retired by proximity. The final point
    # always requires the controller's configured goal-checker result.
    while index < len(points)-1 and math.hypot(points[index][0]-x, points[index][1]-y) < radius:
        index += 1
    return index


class ViaPathProgress:
    """Locate intermediate vias on the real Smac path, using Nav2 path feedback."""
    def __init__(self, rows, points, start_index, match_tolerance=.15):
        if not rows or any(not all(math.isfinite(v) for v in row[:2]) for row in rows):
            raise ValueError('Cannot track vias on an empty or nonfinite path')
        remaining = [0.]*len(rows)
        for i in range(len(rows)-2, -1, -1):
            remaining[i] = remaining[i+1]+math.dist(rows[i][:2], rows[i+1][:2])
        self.thresholds = []
        first = 0
        for index in range(start_index, len(points)-1):
            error, first = min((math.dist(rows[i][:2], points[index][:2]), i)
                               for i in range(first, len(rows)))
            if error > match_tolerance:
                raise RuntimeError(f'Smac path does not include intermediate via {index}: {error:.3f} m')
            self.thresholds.append((index, remaining[first]))

    def advance(self, index, remaining):
        if not math.isfinite(remaining) or remaining < 0.:
            return index
        for via, threshold in self.thresholds:
            # Passage, not a relaxed goal: require progress beyond the via.
            if via == index and remaining < threshold-.05:
                index += 1
        return index


class TerminalProgress:
    """Detect terminal circling without rejecting planned detours farther away."""
    def __init__(self, patience=12., improvement=.03, terminal_distance=2.):
        self.patience, self.improvement, self.terminal_distance = patience, improvement, terminal_distance
        self.remaining = self.best = self.last_progress = None

    def update(self, remaining, endpoint_distance, now):
        if not all(math.isfinite(value) and value >= 0. for value in (remaining, endpoint_distance, now)):
            return
        metric = remaining+endpoint_distance
        if (self.last_progress is None or now < self.last_progress or remaining > self.terminal_distance or
                self.remaining > self.terminal_distance or metric < self.best-self.improvement):
            self.best, self.last_progress = metric, now
        self.remaining = remaining

    def expired(self, now):
        return (self.last_progress is not None and self.remaining <= self.terminal_distance and
                now-self.last_progress > self.patience)


class PathProgress:
    """Reject prolonged oscillation even if the vehicle moves within a small area."""
    def __init__(self, patience=45., improvement=.03):
        self.patience, self.improvement = patience, improvement
        self.best = self.last_progress = None

    def update(self, remaining, now):
        if not all(math.isfinite(v) and v >= 0. for v in (remaining, now)):
            return
        if self.best is None or now < self.last_progress or remaining < self.best-self.improvement:
            self.best, self.last_progress = remaining, now

    def expired(self, now):
        return self.last_progress is not None and now-self.last_progress > self.patience


class Navigator(Node):
    def __init__(self):
        super().__init__('hil_mission_navigation', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.ready = False
        self.pose = None
        self.active_goal = None
        self.route_points, self.route_index = [], 0
        self.route_passage_events = []
        self.last_pose_receipt = self.last_ready_receipt = 0.
        self.ready_sub = self.create_subscription(
            Bool, '/fastlivo_rtk/ready', self.receive_ready, 10)
        self.pose_sub = self.create_subscription(
            Odometry, '/fastlivo_rtk/odometry', self.receive_pose, 10)
        self.stop_publisher = self.create_publisher(Twist, '/hil/navigation/cmd_vel', 1)
        self.planner = ActionClient(self, ComputePathThroughPoses, '/compute_path_through_poses')
        self.controller = ActionClient(self, FollowPath, '/follow_path')

    def receive_ready(self, message):
        self.ready, self.last_ready_receipt = message.data, time.monotonic()

    def receive_pose(self, message):
        self.pose, self.last_pose_receipt = message, time.monotonic()
        if self.route_points:
            p = message.pose.pose.position
            self.route_index = advance_via_index(self.route_points, self.route_index, p.x, p.y)

    def begin_route(self, points):
        self.route_points, self.route_index = list(points), 0
        self.route_passage_events = []

    def wait_active(self, server, timeout=45.):
        client = self.create_client(GetState, '/' + server + '/get_state')
        deadline = time.monotonic()+timeout
        try:
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError(f'{server} lifecycle service unavailable')
            while time.monotonic() < deadline:
                future = client.call_async(GetState.Request())
                try:
                    state = self.wait(future, min(5., max(.1, deadline-time.monotonic()))).current_state
                except TimeoutError:
                    future.cancel()
                    continue
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    return
                rclpy.spin_once(self, timeout_sec=.1)
            raise TimeoutError(f'{server} is not active')
        finally:
            self.destroy_client(client)

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS action timed out')
        return future.result()

    def wait_ready(self, timeout=30.):
        deadline = time.monotonic() + timeout
        while not (self.ready and self.pose is not None and
                   time.monotonic()-self.last_pose_receipt < 2.5 and
                   time.monotonic()-self.last_ready_receipt < 2.5):
            if time.monotonic() >= deadline:
                raise TimeoutError('Healthy FAST-LIVO2/RTK localization is not available')
            rclpy.spin_once(self, timeout_sec=.1)

    def plan(self, points):
        self.wait_ready()
        self.wait_active('planner_server')
        if not self.planner.wait_for_server(timeout_sec=10.):
            raise RuntimeError('X5 Smac action server is unavailable')
        request = ComputePathThroughPoses.Goal()
        request.planner_id = 'GridBased'
        request.use_start = False
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw*.5)
            pose.pose.orientation.w = math.cos(yaw*.5)
            request.goals.append(pose)
        goal = self.wait(self.planner.send_goal_async(request), 10.)
        if not goal.accepted:
            raise RuntimeError('Smac rejected the planning request')
        self.active_goal = goal
        result = self.wait(goal.get_result_async(), 90.)
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
            raise RuntimeError(f'Smac planning failed, action status={result.status}')
        return result.result.path

    def follow(self, path, timeout=900.):
        self.wait_ready()
        self.wait_active('controller_server')
        if not self.controller.wait_for_server(timeout_sec=10.):
            raise RuntimeError('X5 MPPI action server is unavailable')
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = 'FollowPath'
        request.goal_checker_id = 'goal_checker'
        progress = TerminalProgress()
        route_progress = PathProgress()
        endpoint = path.poses[-1].pose.position
        via_progress = ViaPathProgress(
            [[p.pose.position.x, p.pose.position.y] for p in path.poses],
            self.route_points, self.route_index)

        def feedback(message):
            if self.pose is None:
                return
            p = self.pose.pose.pose.position
            distance = math.hypot(p.x-endpoint.x, p.y-endpoint.y)
            progress.update(float(message.feedback.distance_to_goal), distance,
                            self.get_clock().now().nanoseconds/1e9)
            route_progress.update(float(message.feedback.distance_to_goal),
                                  self.get_clock().now().nanoseconds/1e9)
            old = self.route_index
            self.route_index = via_progress.advance(old, float(message.feedback.distance_to_goal))
            if self.route_index != old:
                self.route_passage_events.append({'source': 'native_nav2_remaining_path_feedback',
                    'sim_time': self.get_clock().now().nanoseconds/1e9, 'from_index': old,
                    'to_index': self.route_index, 'remaining_path_m': float(message.feedback.distance_to_goal)})

        goal = self.wait(self.controller.send_goal_async(request, feedback_callback=feedback), 10.)
        if not goal.accepted:
            raise RuntimeError('MPPI rejected the path')
        self.active_goal = goal
        future = goal.get_result_async()
        deadline = time.monotonic()+timeout
        while not future.done():
            rclpy.spin_once(self, timeout_sec=.1)
            if (progress.expired(self.get_clock().now().nanoseconds/1e9) or
                    route_progress.expired(self.get_clock().now().nanoseconds/1e9)):
                self.wait(goal.cancel_goal_async(), 5.)
                result = self.wait(future, 5.)
                self.active_goal = None
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    return
                raise ControllerPathError('Path progress stalled; controller cancelled for a fresh Smac plan')
            if time.monotonic() > deadline:
                raise TimeoutError('FollowPath result timed out')
        result = future.result()
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise ControllerPathError(f'MPPI path execution failed, action status={result.status}')

    def stop(self):
        if self.active_goal is not None:
            try:
                self.wait(self.active_goal.cancel_goal_async(), 3.)
            except Exception as error:
                self.get_logger().error(f'Cancellation: {error}')
        for _ in range(5):
            self.stop_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=.03)

    def parameter_profile(self, server, values):
        self.wait_active(server.lstrip('/'))
        reader = self.create_client(GetParameters, server + '/get_parameters')
        client = self.create_client(SetParametersAtomically, server + '/set_parameters_atomically')
        try:
            if not reader.wait_for_service(timeout_sec=5.) or not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError(f'{server} parameters unavailable')
            names = list(values)
            previous = self.wait(reader.call_async(GetParameters.Request(names=names)), 15.)
            requested = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
            if (len(previous.values) != len(requested) or
                    any(old.type != new.value.type for old, new in zip(previous.values, requested))):
                raise RuntimeError(f'{server} profile parameter types do not match the declared types: '
                                   f'actual={[v.type for v in previous.values]}, '
                                   f'requested={[p.value.type for p in requested]}')
            original = [ParameterMessage(name=name, value=value)
                        for name, value in zip(names, previous.values)]
            # Smac rebuilds its motion/heuristic tables in the native callback.
            # A client timeout does not cancel that work or roll back parameters.
            try:
                result = self.wait(client.call_async(SetParametersAtomically.Request(parameters=requested)),
                    PARAMETER_UPDATE_TIMEOUT)
            except TimeoutError as error:
                raise TimeoutError(f'{server} parameter reply timed out; application state is unknown, '
                                   'do not resubmit without checking the server') from error
            if not result.result.successful:
                raise RuntimeError(result.result.reason)
            return client, original
        except Exception:
            self.destroy_client(client)
            raise
        finally:
            self.destroy_client(reader)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', type=Path, default=Path.home() / 'agribot_hil_ws/scenario/mission.json')
    parser.add_argument('--phase', choices=['outbound', 'completion', 'return', 'approach', 'straight_check'], default='outbound')
    parser.add_argument('--execute', action='store_true', help='Without this flag only planning is performed')
    parser.add_argument('--start-via', type=int, default=0,
                        help='Explicit resume index for an interrupted outbound/return path')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text())
    rclpy.init()
    node = Navigator()
    profiles = []
    report = {'phase': args.phase, 'planner': 'Smac Hybrid-A* on X5',
              'executed': False, 'status': 'waiting_for_localization'}
    try:
        node.wait_ready()
        if args.phase == 'approach':
            profiles.append(node.parameter_profile('/planner_server', {'GridBased.tolerance': .01}))
            profiles.append(node.parameter_profile('/controller_server', {
                'goal_checker.xy_goal_tolerance': .015, 'goal_checker.yaw_goal_tolerance': .03}))
        if args.phase == 'straight_check':
            p = node.pose.pose.pose.position
            q = node.pose.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            points = [[p.x+math.cos(yaw), p.y+math.sin(yaw), yaw]]
        elif args.phase == 'approach':
            points = [mission['park']]
        else:
            points = mission[args.phase + '_via'][args.start_via:]
            if args.start_via < 0 or not points:
                raise ValueError('Resume index is outside the mission waypoints')
        report.update(requested_goals=points, status='planning')
        path = node.plan(points)
        report.update(status='planned', path=[[
            p.pose.position.x, p.pose.position.y, p.pose.orientation.z,
            p.pose.orientation.w] for p in path.poses])
        if args.phase == 'approach':
            last = path.poses[-1].pose.position
            report['endpoint_error_m'] = math.hypot(last.x-points[-1][0], last.y-points[-1][1])
            if report['endpoint_error_m'] > .02:
                raise RuntimeError(f"Docking endpoint error {report['endpoint_error_m']:.4f} m exceeds 0.02 m")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(f'Smac planned {len(path.poses)} poses', flush=True)
        if args.execute:
            report['status'] = 'executing'
            node.follow(path)
            report['executed'] = True
            report['status'] = 'succeeded'
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            print('MPPI completed the path through native CAN', flush=True)
    except Exception as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        try:
            node.stop()
            for client, original in reversed(profiles):
                result = node.wait(client.call_async(SetParametersAtomically.Request(parameters=original)),
                                   PARAMETER_UPDATE_TIMEOUT)
                if not result.result.successful:
                    node.get_logger().error(f'Cannot restore docking parameters: {result.result.reason}')
        finally:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
