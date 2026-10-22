#!/usr/bin/env python3
"""Sequential, isolated Gazebo runtime checks, not catalog-only validation."""

import argparse
from collections import deque
import copy
import itertools
import json
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import signal
import platform
import re
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose, FollowPath
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Odometry, Path as PathMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, NavSatFix, PointCloud2
from std_msgs.msg import Bool, String
import yaml


PERCEPTIONS = ("stvl", "voxel")
LOCALIZATIONS = ("fastlivo_rtk", "fast_lio", "navsat", "kiss_icp")
PLANNERS = ("smac_hybrid", "navfn", "theta_star", "smac_2d", "direct")
CONTROLLERS = ("mppi", "rpp", "dwb")
VISIONS = ("off", "object_detection", "instance_segmentation", "pose_estimation")
SAFE_DOMAINS = tuple(domain for domain in (*range(1, 102), *range(215, 233))
                     if domain != 37)


def case_domain(base, index, reuse=False):
    return base if reuse else SAFE_DOMAINS[(SAFE_DOMAINS.index(base) + index) % len(SAFE_DOMAINS)]


def waypoint_acceptance_radii(count, wheelbase, full_route):
    if not math.isfinite(wheelbase) or wheelbase <= 0:
        raise ValueError("Final-position acceptance requires a positive finite wheelbase")
    radii = [0.75] * count
    if radii and full_route:
        radii[-1] = wheelbase * 0.5
    return radii


def terminal_motion_stopped(command, twist):
    return (len(command) == len(twist) == 2
            and all(math.isfinite(value) for value in (*command, *twist))
            and all(abs(value) < 1e-5 for value in command)
            and all(abs(value) < 0.02 for value in twist))


def check_settled_position(observation, waypoint, radius, sample_allowance):
    point = observation["truth"]
    target = waypoint.pose.position
    error = math.hypot(point[0] - target.x, point[1] - target.y)
    if not math.isfinite(error) or error > radius + sample_allowance:
        raise RuntimeError("Vehicle stopped outside the final half-wheelbase position tolerance")
    return error


def abnormal_exits(log):
    return [{"process": name, "code": int(code)}
            for name, code in re.findall(
                r"\[ERROR\] \[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)", log)]


def gazebo_pid(log):
    match = re.search(r"\[gzserver-\d+\]: process started with pid \[(\d+)\]", log)
    if not match:
        raise RuntimeError("Gazebo server PID was not recorded")
    return int(match.group(1))


def loaded_library_paths(maps, basename):
    paths = set()
    for line in maps.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and Path(fields[5]).name.startswith(basename):
            paths.add(fields[5])
    return sorted(paths)


def coverage_cases():
    # Every branch is exercised with real nodes; this is not the full matrix.
    cases = [("stvl", "off", loc, "smac_hybrid", "mppi") for loc in LOCALIZATIONS]
    cases += [("stvl", "off", "fastlivo_rtk", planner, "mppi")
              for planner in PLANNERS[1:]]
    cases += [("stvl", "off", "fastlivo_rtk", "smac_hybrid", ctl)
              for ctl in CONTROLLERS[1:]]
    cases += [("voxel", "off", "fastlivo_rtk", "smac_hybrid", "mppi")]
    cases += [("stvl", vision, "fastlivo_rtk", "smac_hybrid", "mppi")
              for vision in VISIONS[1:]]
    return cases


def direct_builder_path():
    return Path(get_package_prefix("agribot_autonomy")) / \
        "lib/agribot_autonomy/snake_waypoint_runner.py"


def production_direct_path(start, targets):
    spec = importlib.util.spec_from_file_location("acceptance_waypoint_runner", direct_builder_path())
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def point(item):
        q = item.orientation
        return {"x": item.position.x, "y": item.position.y,
                "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))}

    # Use the installed production helper and its default sampling, not a
    # smoother or denser test-only replacement for the direct-route branch.
    points = module.build_path_points([point(target.pose) for target in targets], point(start))
    path = PathMessage()
    path.header.frame_id = "map"
    path.poses = [pose(p["x"], p["y"], p["yaw"]) for p in points]
    return path


def pose(x, y, yaw):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def pose_yaw(value):
    q = value.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def heading_error(value, target):
    difference = pose_yaw(value) - target
    return abs(math.atan2(math.sin(difference), math.cos(difference)))


def path_prefix(path, distance):
    result = PathMessage()
    result.header = path.header
    length = 0.0
    for item in path.poses:
        if result.poses:
            previous = result.poses[-1].pose.position
            current = item.pose.position
            length += math.hypot(current.x - previous.x, current.y - previous.y)
        result.poses.append(copy.deepcopy(item))
        if length >= distance:
            break
    if len(result.poses) >= 2:
        a, b = result.poses[-2].pose.position, result.poses[-1].pose.position
        # A truncated 2D plan often has an unset intermediate orientation.
        # The test's new endpoint must have the tangent of the unchanged path.
        yaw = math.atan2(b.y - a.y, b.x - a.x)
        result.poses[-1].pose.orientation = pose(0, 0, yaw).pose.orientation
    return result


def ordered_waypoint_distances(path, waypoints):
    if not path.poses:
        raise ValueError("Planner returned an empty path")
    if not waypoints:
        return []
    # Minimize the largest ordered miss. A greedy nearest point can jump to a
    # later lap and incorrectly reject an otherwise valid out-and-back route.
    costs = [0.0] * len(path.poses)
    predecessors = []
    for target in waypoints:
        best = 0
        parents = []
        next_costs = []
        for index, item in enumerate(path.poses):
            if costs[index] < costs[best]:
                best = index
            distance = math.hypot(item.pose.position.x - target.pose.position.x,
                                  item.pose.position.y - target.pose.position.y)
            next_costs.append(max(costs[best], distance))
            parents.append(best)
        predecessors.append(parents)
        costs = next_costs
    index = min(range(len(costs)), key=costs.__getitem__)
    errors = []
    for target, parents in zip(reversed(waypoints), reversed(predecessors)):
        item = path.poses[index].pose.position
        errors.append(math.hypot(item.x - target.pose.position.x,
                                 item.y - target.pose.position.y))
        index = parents[index]
    return errors[::-1]


def driven_waypoint_distances(path, waypoints, terminal_observation, full_route):
    if not full_route or not waypoints:
        return ordered_waypoint_distances(path, waypoints)
    # Intermediate neighborhoods share one radius. The final, differently
    # sized neighborhood must use the actual stationary endpoint, not an
    # arbitrary tied sample from the minimax trajectory assignment.
    errors = ordered_waypoint_distances(path, waypoints[:-1])
    point = terminal_observation["truth"]
    target = waypoints[-1].pose.position
    final_error = math.hypot(point[0] - target.x, point[1] - target.y)
    if not math.isfinite(final_error):
        raise ValueError("Final stationary position must be finite")
    return errors + [final_error]


class Probe(Node):
    def __init__(self, localization):
        super().__init__(f"algorithm_runtime_acceptance_{time.monotonic_ns()}", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self.counts = {}
        self.nonempty_clouds = {}
        self.cloud_sizes = {}
        self.topic_stamps = {}
        self.estimate = None
        self.truth = None
        self.initial_truth = None
        self.max_startup_motion = 0.0
        self.max_startup_command = 0.0
        self.max_chassis_linear_speed = 0.0
        self.max_chassis_angular_speed = 0.0
        self.motion_dispatched = False
        self.clock_samples = []
        self.position_errors = []
        self.position_error_time_deltas = []
        self.truth_history = deque(maxlen=4096)
        self.estimate_stamp = None
        self.last_error_stamp = None
        self.image_dynamic_ranges = []
        self.mandatory_waypoints = []
        self.initial_complete_plan = None
        self.latest_plan = None
        self.plan_updates = 0
        self.vision_result = None
        self.fusion_fixed_active = False
        self.fusion_diagnostics = {}
        self.trace = []
        self.capture_terminal = False
        self.terminal_observation = None
        self.latest_command = [0.0, 0.0]
        self.joint_names = set()
        self.navigation_twist = None
        self.navigation_odom_topic = {
            "fastlivo_rtk": "/fastlivo_rtk/odometry",
            "fast_lio": "/fastlio/odometry",
            "navsat": "/odometry/filtered_navsat",
            "kiss_icp": "/kiss/odometry",
        }.get(localization, "/odom")
        self.last_trace_at = 0.0
        self.subscriptions_kept = []
        for topic, kind in (("/lidar/points", PointCloud2), ("/imu/data", Imu),
                            ("/configured/localization_points", PointCloud2),
                            ("/configured/obstacle_points", PointCloud2),
                            ("/camera/rgb/image_raw", Image), ("/odom", Odometry),
                            ("/navsat/fix", NavSatFix),
                            ("/joint_states", JointState),
                            ("/fastlivo_rtk/fixed_active", Bool),
                            ("/diagnostics", DiagnosticArray),
                            ("/amcl_pose", PoseWithCovarianceStamped),
                            ("/clock", Clock), ("/nav2/cmd_vel", Twist),
                            ("/vision/results", String), ("/vision/annotated_image", Image)):
            self.subscriptions_kept.append(self.create_subscription(
                kind, topic, lambda msg, t=topic: self.on_message(t, msg),
                qos_profile_sensor_data))
        if self.navigation_odom_topic != "/odom":
            self.subscriptions_kept.append(self.create_subscription(
                Odometry, self.navigation_odom_topic,
                lambda msg: self.on_message(self.navigation_odom_topic, msg),
                qos_profile_sensor_data))
        self.subscriptions_kept.append(self.create_subscription(
            PathMessage, "/plan", lambda msg: self.on_message("/plan", msg), qos_profile_sensor_data))
        self.planner = ActionClient(self, ComputePathThroughPoses,
                                    "/compute_path_through_poses")
        self.segment_planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "/follow_path")
        self.states = {name: self.create_client(GetState, f"/{name}/get_state")
                       for name in ("map_server", "planner_server", "controller_server")}

    def on_message(self, topic, message):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if hasattr(message, "header"):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.topic_stamps.setdefault(topic, []).append(stamp)
        if isinstance(message, PointCloud2) and message.width * message.height > 0:
            self.nonempty_clouds[topic] = self.nonempty_clouds.get(topic, 0) + 1
            self.cloud_sizes.setdefault(topic, []).append(message.width * message.height)
        if topic == self.navigation_odom_topic:
            self.navigation_twist = {
                "topic": topic, "stamp": stamp, "frame": message.child_frame_id,
                "linear": [getattr(message.twist.twist.linear, axis) for axis in ("x", "y", "z")],
                "angular": [getattr(message.twist.twist.angular, axis) for axis in ("x", "y", "z")],
            }
        if topic == "/clock":
            self.clock_samples.append((time.monotonic(), message.clock.sec +
                                       message.clock.nanosec * 1e-9))
        elif topic == "/joint_states":
            self.joint_names.update(message.name)
        elif topic == "/amcl_pose":
            self.estimate = message.pose.pose
            self.estimate_stamp = stamp
            self.record_pose_error()
        elif topic == "/fastlivo_rtk/fixed_active":
            self.fusion_fixed_active = bool(message.data)
        elif topic == "/diagnostics":
            for status in message.status:
                if status.name == "agribot/fastlivo_rtk_fusion":
                    self.fusion_diagnostics = {item.key: item.value for item in status.values}
        elif topic == "/odom":
            self.truth = message.pose.pose
            self.max_chassis_linear_speed = max(
                self.max_chassis_linear_speed, abs(message.twist.twist.linear.x))
            self.max_chassis_angular_speed = max(
                self.max_chassis_angular_speed, abs(message.twist.twist.angular.z))
            self.truth_history.append((stamp, self.truth))
            if self.initial_truth is None:
                self.initial_truth = self.truth
            if not self.motion_dispatched:
                self.max_startup_motion = max(self.max_startup_motion,
                                             self.displacement(self.initial_truth))
            if self.estimate is not None:
                self.record_pose_error()
                now = time.monotonic()
                if self.capture_terminal or now - self.last_trace_at >= 0.5:
                    self.trace.append({"wall_monotonic": now,
                                       "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                                       "truth": [self.truth.position.x, self.truth.position.y, self.truth.position.z],
                                       "chassis_twist": [message.twist.twist.linear.x, message.twist.twist.angular.z],
                                       "command": self.latest_command,
                                       "navigation_twist": self.navigation_twist,
                                       "estimate": [self.estimate.position.x, self.estimate.position.y, self.estimate.position.z],
                                       "truth_quaternion": [getattr(self.truth.orientation, key) for key in ("x", "y", "z", "w")],
                                       "estimate_quaternion": [getattr(self.estimate.orientation, key) for key in ("x", "y", "z", "w")]})
                    self.last_trace_at = now
        elif topic == "/vision/results":
            self.vision_result = json.loads(message.data)
        elif topic == "/nav2/cmd_vel":
            self.latest_command = [message.linear.x, message.angular.z]
            if not self.motion_dispatched:
                self.max_startup_command = max(self.max_startup_command,
                    abs(message.linear.x), abs(message.angular.z))
        elif topic == "/camera/rgb/image_raw" and len(self.image_dynamic_ranges) < 5:
            samples = message.data[::max(1, len(message.data) // 4096)]
            if samples:
                self.image_dynamic_ranges.append(int(max(samples)) - int(min(samples)))
        elif topic == "/plan":
            self.latest_plan = message
            self.plan_updates += 1
            if self.initial_complete_plan is None and self.mandatory_waypoints and message.poses:
                errors = ordered_waypoint_distances(message, self.mandatory_waypoints)
                if max(errors) <= 0.35:
                    self.initial_complete_plan = message
                    self.motion_dispatched = True

    def record_pose_error(self):
        if not self.truth_history or self.estimate_stamp is None or self.estimate_stamp == self.last_error_stamp:
            return
        if not self.truth_history[0][0] <= self.estimate_stamp <= self.truth_history[-1][0]:
            return
        stamp, truth = min(self.truth_history, key=lambda item: abs(item[0] - self.estimate_stamp))
        delta = abs(stamp - self.estimate_stamp)
        if delta > 0.05:
            return
        self.position_errors.append(math.hypot(truth.position.x - self.estimate.position.x,
                                               truth.position.y - self.estimate.position.y))
        self.position_error_time_deltas.append(delta)
        self.last_error_stamp = self.estimate_stamp

    def displacement(self, start):
        return math.hypot(self.truth.position.x - start.position.x,
                          self.truth.position.y - start.position.y)

    def save_costmap(self, output, case_id, costmap_name="global_costmap"):
        client = self.create_client(GetCostmap, f"/{costmap_name}/get_costmap")
        try:
            if not client.wait_for_service(timeout_sec=3):
                return {"error": "Global costmap service unavailable"}
            message = self.await_future(client.call_async(GetCostmap.Request()), 10).map
            metadata = message.metadata
            data = bytes(message.data)
            binary = output / f"{case_id}.costmap.bin"
            binary.write_bytes(data)
            info = {"frame": message.header.frame_id, "resolution": metadata.resolution,
                    "width": metadata.size_x, "height": metadata.size_y,
                    "origin": [metadata.origin.position.x, metadata.origin.position.y],
                    "data_file": str(binary)}
            (output / f"{case_id}.costmap.json").write_text(json.dumps(info, indent=2) + "\n")
            return info
        finally:
            self.destroy_client(client)

    def until(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def await_future(self, future, seconds):
        if not self.until(future.done, seconds):
            raise TimeoutError("ROS action/service response timed out")
        return future.result()

    def ready(self, seconds, process=None):
        deadline = time.monotonic() + seconds

        def startup_ready():
            if process is not None and process.poll() is not None:
                raise RuntimeError(f"Simulation launch exited during startup: {process.returncode}")
            return (self.estimate is not None and self.truth is not None
                    and self.counts.get("/amcl_pose", 0) >= 5
                    and self.planner.server_is_ready()
                    and self.controller.server_is_ready())

        if not self.until(startup_ready, seconds):
            raise TimeoutError("Localization or Nav2 actions did not become ready")
        for name, client in self.states.items():
            if not client.wait_for_service(timeout_sec=5):
                raise TimeoutError(f"Lifecycle service missing: {name}")
            while True:
                state = self.await_future(client.call_async(GetState.Request()), 5)
                if state.current_state.id == 3:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{name} is not active: {state.current_state.label}")
                self.until(lambda: False, 0.5)
        self.until(lambda: False, min(3.0, max(0.0, deadline - time.monotonic())))

    def plan(self, waypoints, direct):
        if direct:
            return production_direct_path(self.estimate, waypoints)
        goal = ComputePathThroughPoses.Goal()
        goal.goals = waypoints
        for target in goal.goals:
            target.header.stamp = self.get_clock().now().to_msg()
        goal.planner_id = "GridBased"
        goal.use_start = False
        handle = self.await_future(self.planner.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError("Planner rejected the full-route goal")
        result = self.await_future(handle.get_result_async(), 90)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            # The production waypoint runner has the same segmented fallback.
            path = PathMessage()
            path.header.frame_id = "map"
            for index, target in enumerate(waypoints):
                segment_goal = ComputePathToPose.Goal()
                segment_goal.goal = target
                segment_goal.planner_id = "GridBased"
                segment_goal.use_start = index > 0
                if index > 0:
                    segment_goal.start = waypoints[index - 1]
                segment = self.await_future(self.segment_planner.send_goal_async(segment_goal), 10)
                if not segment.accepted:
                    raise RuntimeError(f"Segment {index + 1} was rejected")
                segment_result = self.await_future(segment.get_result_async(), 30)
                if segment_result.status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f"Segment {index + 1} failed: status={segment_result.status}")
                path.poses.extend(segment_result.result.path.poses)
            return path
        return result.result.path

    def reject_outside_map_goal(self):
        goal = ComputePathToPose.Goal()
        goal.goal = pose(1000, 1000, 0)
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.planner_id = "GridBased"
        goal.use_start = False
        handle = self.await_future(self.segment_planner.send_goal_async(goal), 10)
        if not handle.accepted:
            return GoalStatus.STATUS_ABORTED
        result = self.await_future(handle.get_result_async(), 20)
        if result.status == GoalStatus.STATUS_SUCCEEDED or result.result.path.poses:
            raise RuntimeError("Planner accepted an outside-map goal")
        return result.status

    def follow(self, path, seconds):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        start = self.truth
        self.motion_dispatched = True
        handle = self.await_future(self.controller.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError("Controller rejected the path")
        future = handle.get_result_async()
        last_progress = time.monotonic()

        def finished():
            nonlocal last_progress
            now = time.monotonic()
            if now - last_progress >= 30 and self.truth is not None:
                position = self.truth.position
                print(f"Motion progress: truth=({position.x:.2f}, {position.y:.2f}), "
                      f"displacement={self.displacement(start):.2f}m", flush=True)
                last_progress = now
            return future.done()

        if not self.until(finished, seconds):
            self.await_future(handle.cancel_goal_async(), 5)
            raise TimeoutError(f"Controller did not finish within {seconds}s")
        result = future.result()
        return result.status, self.displacement(start)

    def observe_production_route(self, log_file, process, seconds):
        start = self.initial_truth
        last_read = 0.0
        last_progress = 0.0
        succeeded = False
        last_motion = time.monotonic()
        motion_anchor = None

        def finished():
            nonlocal last_read, last_progress, succeeded, last_motion, motion_anchor
            now = time.monotonic()
            if process.poll() is not None:
                raise RuntimeError("Production launch exited during route execution")
            if self.truth is not None and getattr(self, "stall_timeout", 0) > 0:
                point = self.truth.position
                current = (point.x, point.y)
                if motion_anchor is None or math.dist(current, motion_anchor) >= 0.3:
                    motion_anchor, last_motion = current, now
                elif now - last_motion > self.stall_timeout:
                    raise RuntimeError(f"No physical route progress for {self.stall_timeout:.0f}s; validation stopped")
            if now - last_progress >= 30 and self.truth is not None:
                point = self.truth.position
                print(f"Production route: truth=({point.x:.2f}, {point.y:.2f}), "
                      f"plan_updates={self.plan_updates}", flush=True)
                last_progress = now
            if now - last_read >= 1.0:
                succeeded = "Continuous path completed successfully" in log_file.read_text()
                last_read = now
            return succeeded

        if not self.until(finished, seconds):
            raise TimeoutError("Production waypoint runner did not complete the full route")
        self.observe_native_stop()
        return GoalStatus.STATUS_SUCCEEDED, self.displacement(start)


    def observe_native_stop(self):
        # The regular 2 Hz trace can end before braking finishes on a small car.
        # Observe fresh truth at its native rate; never publish a stop command.
        previous_count = len(self.trace)
        stable_since = None
        self.capture_terminal = True

        def settled():
            nonlocal stable_since
            now = time.monotonic()
            fresh = (len(self.trace) > previous_count
                     and 0.0 <= now - self.trace[-1]["wall_monotonic"] <= 0.25)
            stopped = fresh and terminal_motion_stopped(
                self.latest_command, self.trace[-1]["chassis_twist"])
            if not stopped:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = now
            return now - stable_since >= 1.0

        try:
            if not self.until(settled, 10):
                raise RuntimeError("Native success was not followed by fresh stationary odometry and zero command")
            self.terminal_observation = {
                "protocol": "native_success_then_fresh_stationary_odometry_v1",
                "fresh_terminal_samples": len(self.trace) - previous_count,
                "stationary_hold_s": time.monotonic() - stable_since,
                "truth": list(self.trace[-1]["truth"]),
                "command": list(self.latest_command),
                "chassis_twist": list(self.trace[-1]["chassis_twist"]),
            }
        finally:
            self.capture_terminal = False


def stop_session(process):
    # ROS launch children may own process groups but share this dedicated SID.
    if process.poll() is None:
        # Let launch perform ordered shutdown before signaling remaining children.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
    for sig, seconds in ((signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        rows = subprocess.check_output(["ps", "-eo", "pid=,sid=,stat="], text=True)
        pids = [int(row.split()[0]) for row in rows.splitlines()
                if int(row.split()[1]) == process.pid and not row.split()[2].startswith("Z")]
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        try:
            process.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    process.wait(timeout=5)


def run_case(case, args, output, waypoints):
    perception, vision, localization, planner, controller = case
    case_id = "_".join(case)
    result = {"id": case_id, "scope": args.scope, "status": "RUNNING",
              "vehicle_id": args.vehicle_id,
              "scenario": args.scenario,
              "ros_domain_id": int(os.environ["ROS_DOMAIN_ID"]),
              "full_route": "NOT_RUN", "checks": {}, "started": time.strftime("%FT%T%z")}
    result_file = output / f"{case_id}.json"
    log_file = output / f"{case_id}.log"
    result_file.write_text(json.dumps(result, indent=2) + "\n")
    command = ["ros2", "launch", "agribot_vehicle_description",
               "configured_ackermann_sim.launch.py", f"gui:={str(args.gui).lower()}", f"rviz:={str(args.rviz).lower()}",
               f"vehicle_id:={args.vehicle_id}",
               f"scenario:={args.scenario}",
               f"rmw_implementation:={args.rmw}",
               f"run_waypoints:={'true' if args.scope == 'full-route' else 'false'}", f"perception_mode:={perception}",
               f"localization_mode:={localization}", f"controller_mode:={controller}",
               f"planner_mode:={'smac_hybrid' if planner == 'direct' else planner}",
               f"route_mode:={'direct' if planner == 'direct' else 'planned'}",
               f"vision_mode:={vision}", f"vision_python:={args.vision_python}",
               f"manipulator:={args.manipulator}", f"arm_demo:={args.arm_demo}"]
    if getattr(args, "scene_manifest", None):
        command.append(f"scene_manifest:={args.scene_manifest.resolve()}")
        result["scene_manifest"] = str(args.scene_manifest.resolve())
        result["scenario"] = "external_scene"
    started = time.monotonic()
    try:
        node = Probe(localization)
        node.stall_timeout = args.stall_timeout
    except Exception as exc:
        result["status"] = "FAIL"
        result["error"] = f"Probe initialization failed: {type(exc).__name__}: {exc}"
        result["wall_seconds"] = time.monotonic() - started
        result_file.write_text(json.dumps(result, indent=2) + "\n")
        log_file.write_text(result["error"] + "\n")
        print(f"FAIL {case_id}: {result['error']}", flush=True)
        return result
    with log_file.open("w") as log:
        if args.scope == "full-route":
            node.mandatory_waypoints = waypoints
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            node.ready(args.startup_timeout, process)
            result["checks"]["lifecycle_active"] = True
            pid = gazebo_pid(log_file.read_text())
            libraries = loaded_library_paths(Path(f"/proc/{pid}/maps").read_text(),
                                             "libgazebo_transport.so")
            result["checks"]["gazebo_transport_libraries"] = libraries
            if controller == "rpp":
                control_process = re.search(
                    r"\[controller_server-\d+\]: process started with pid \[(\d+)\]", log_file.read_text())
                if control_process is None:
                    raise RuntimeError("RPP controller process was not started")
                rpp_libraries = loaded_library_paths(
                    Path(f"/proc/{control_process[1]}/maps").read_text(),
                    "libagribot_position_only_rpp_controller.so")
                result["checks"]["position_only_rpp_libraries"] = rpp_libraries
                if not rpp_libraries:
                    raise RuntimeError("Position-only RPP adapter was not loaded")
            if args.gui:
                client = re.search(r"\[gzclient-\d+\]: process started with pid \[(\d+)\]",
                                   log_file.read_text())
                if client is None:
                    raise RuntimeError("Gazebo GUI process was not started")
                client_maps = Path(f"/proc/{client[1]}/maps").read_text()
                catalogs = loaded_library_paths(client_maps, "libagribot_gazebo_local_model_catalog.so")
                result["checks"]["gazebo_local_catalog_libraries"] = catalogs
                if not catalogs:
                    raise RuntimeError("Gazebo offline model catalog protection was not loaded")
            sensor_filter = loaded_library_paths(Path(f"/proc/{pid}/maps").read_text(),
                                                 "libagribot_simulation_sensor_collision_filter.so")
            result["checks"]["sensor_collision_filter_libraries"] = sensor_filter
            contract = Path(get_package_share_directory("agribot_vehicle_description")) / \
                "generated" / args.vehicle_id / "vehicle_config.json"
            if json.loads(contract.read_text()).get("metadata", {}).get("calibrationState") == \
                    "unity_nominal_simulation_only" and not sensor_filter:
                raise RuntimeError("Nominal CAD sensor collision filter was not loaded")
            compat = Path(get_package_prefix("agribot_vehicle_description")) / \
                "lib/agribot_gazebo_transport_compat/libgazebo_transport.so.11"
            if compat.is_file() and subprocess.check_output(
                    ["pkg-config", "--modversion", "gazebo"], text=True).strip() == "11.10.2":
                if str(compat.resolve()) not in libraries:
                    raise RuntimeError("Gazebo shutdown compatibility library was not loaded")
            result["checks"]["startup_motion_m"] = node.max_startup_motion
            result["checks"]["startup_nonzero_command"] = node.max_startup_command
            if node.max_startup_motion > 0.15:
                raise RuntimeError("Vehicle moved before planning/control dispatch")
            if node.max_startup_command > 1e-6:
                raise RuntimeError("Nonzero command was sent before planning/control dispatch")
            if args.scope == "smoke":
                result["checks"]["outside_map_goal_status"] = node.reject_outside_map_goal()
            if args.local_maneuver:
                yaw = pose_yaw(node.estimate)
                start = node.estimate.position
                waypoints = [pose(start.x + args.motion_distance * math.cos(yaw) - args.maneuver_lateral_offset * math.sin(yaw),
                                  start.y + args.motion_distance * math.sin(yaw) + args.maneuver_lateral_offset * math.cos(yaw), yaw)]
            result["checks"]["planned_waypoints"] = len(waypoints)
            if args.scope == "full-route":
                if not node.until(lambda: node.initial_complete_plan is not None, args.startup_timeout):
                    raise RuntimeError("Production runner did not publish the complete verified route")
                path = node.initial_complete_plan
                result["execution_pipeline"] = "unity_default_waypoint_runner"
                planning_times = re.findall(r"planning time ([0-9.]+) s", log_file.read_text())
                if planning_times:
                    result["checks"]["planner_reported_seconds"] = float(planning_times[0])
            else:
                plan_started = time.monotonic()
                path = node.plan(waypoints, planner == "direct")
                result["checks"]["planning_wall_seconds"] = time.monotonic() - plan_started
            result["checks"]["path_poses"] = len(path.poses)
            (output / f"{case_id}.path.json").write_text(json.dumps([
                {"x": p.pose.position.x, "y": p.pose.position.y,
                 "qz": p.pose.orientation.z, "qw": p.pose.orientation.w}
                for p in path.poses]) + "\n")
            errors = ordered_waypoint_distances(path, waypoints)
            result["checks"]["ordered_waypoint_errors_m"] = errors
            if max(errors) > 0.35:
                raise RuntimeError("Full route skipped a mandatory waypoint")
            selected = path if args.scope == "full-route" or args.local_maneuver else path_prefix(path, args.motion_distance)
            if args.terminal_yaw_offset:
                terminal = selected.poses[-1].pose
                yaw = pose_yaw(terminal) + args.terminal_yaw_offset
                terminal.orientation = pose(0, 0, yaw).pose.orientation
                result["checks"]["requested_terminal_yaw_rad"] = yaw
            if args.scope == "full-route":
                result["full_route"] = "RUNNING"
                status, displacement = node.observe_production_route(log_file, process, args.motion_timeout)
                result["checks"]["production_plan_updates"] = node.plan_updates
                result["checks"]["terminal_stop_observation"] = node.terminal_observation
            else:
                status, displacement = node.follow(selected, args.motion_timeout)
            result["checks"]["controller_action_status"] = status
            result["checks"]["truth_displacement_m"] = displacement
            if status != GoalStatus.STATUS_SUCCEEDED or displacement < 1.0:
                raise RuntimeError("Controller failed or the Gazebo vehicle did not move")
            result["checks"]["max_chassis_linear_speed_mps"] = node.max_chassis_linear_speed
            result["checks"]["max_chassis_angular_speed_radps"] = node.max_chassis_angular_speed
            if args.terminal_yaw_offset:
                yaw = result["checks"]["requested_terminal_yaw_rad"]
                error = heading_error(node.truth, yaw)
                result["checks"]["truth_terminal_yaw_error_rad"] = error
                result["checks"]["estimate_terminal_yaw_error_rad"] = heading_error(node.estimate, yaw)
                if error > 0.35:
                    raise RuntimeError("Differential controller did not reach the requested terminal heading")
            expected_waypoints = waypoints if args.scope == "full-route" else [
                target for target in waypoints
                if ordered_waypoint_distances(selected, [target])[0] <= 0.35]
            if expected_waypoints:
                driven = PathMessage()
                driven.poses = [pose(item["truth"][0], item["truth"][1], 0) for item in node.trace]
                errors = driven_waypoint_distances(
                    driven, expected_waypoints, node.terminal_observation, args.scope == "full-route")
                result["checks"]["driven_waypoints"] = len(expected_waypoints)
                result["checks"]["driven_waypoint_errors_m"] = errors
                contract = json.loads((Path(get_package_share_directory("agribot_vehicle_description")) /
                    "generated" / args.vehicle_id / "vehicle_config.json").read_text())
                final_radius = float(contract["geometry"]["wheelbaseM"]) * 0.5
                full_route = args.scope == "full-route"
                radii = waypoint_acceptance_radii(len(errors), final_radius * 2, full_route)
                sample_allowance = 0.03 if full_route else 0.0
                result["checks"]["waypoint_neighborhood_radius_m"] = 0.75
                result["checks"]["driven_waypoint_acceptance_radii_m"] = radii
                result["checks"]["final_waypoint_radius_m"] = radii[-1]
                result["checks"]["final_sample_allowance_m"] = sample_allowance
                if any(error > radius + (sample_allowance if i == len(errors) - 1 else 0.0)
                       for i, (error, radius) in enumerate(zip(errors, radii))):
                    raise RuntimeError("Vehicle missed an intermediate 0.75m neighborhood or final half-wheelbase tolerance")
                if full_route:
                    result["checks"]["settled_final_waypoint_error_m"] = check_settled_position(
                        node.terminal_observation, expected_waypoints[-1], radii[-1], sample_allowance)
            for topic in ("/lidar/points", "/configured/obstacle_points", "/imu/data", "/camera/rgb/image_raw", "/odom"):
                if node.counts.get(topic, 0) < 3:
                    raise RuntimeError(f"Sensor topic missing: {topic}")
            for topic in ("/lidar/points", "/configured/obstacle_points"):
                if node.nonempty_clouds.get(topic, 0) < 3:
                    raise RuntimeError(f"Point cloud messages are empty: {topic}")
            result["checks"]["camera_dynamic_ranges"] = node.image_dynamic_ranges
            if not node.image_dynamic_ranges or max(node.image_dynamic_ranges) == 0:
                raise RuntimeError("Simulation camera produced only blank frames")
            if localization in ("fast_lio", "fastlivo_rtk"):
                if node.nonempty_clouds.get("/configured/localization_points", 0) < 3:
                    raise RuntimeError("Simulation localization time adapter produced no clouds")
            if localization in ("navsat", "fastlivo_rtk"):
                if node.counts.get("/navsat/fix", 0) < 3:
                    raise RuntimeError("Simulated RTK fixes are missing")
            if localization == "fastlivo_rtk":
                result["checks"]["rtk_fixed_active"] = node.fusion_fixed_active
                result["checks"]["fusion_diagnostics"] = node.fusion_diagnostics
                if int(node.fusion_diagnostics.get("fixed_rtk_factors", "0")) < 2:
                    raise RuntimeError("Fusion did not accept enough RTK factors")
                if not node.fusion_fixed_active:
                    raise RuntimeError("FAST-LIVO2/RTK fusion did not activate fixed-position constraints")
            if vision != "off":
                if not node.until(lambda: node.counts.get("/vision/results", 0) >= 3
                                  and node.counts.get("/vision/annotated_image", 0) >= 3, 30):
                    raise RuntimeError("Real visual inference produced no outputs")
                if node.vision_result["mode"] != vision:
                    raise RuntimeError("Visual output mode is not the selected mode")
            if args.manipulator == "true":
                arm_joints = sorted(name for name in node.joint_names if name.startswith("manipulator_"))
                result["checks"]["manipulator_feedback_joints"] = arm_joints
                if len(arm_joints) < 8:
                    raise RuntimeError("Mounted arm did not publish all six joints and two gripper joints")
            result["status"] = "PASS"
            if args.scope == "full-route":
                result["full_route"] = "PASS"
        except Exception as exc:
            result["status"] = "FAIL"
            result["error"] = f"{type(exc).__name__}: {exc}"
            try:
                result["failure_costmap"] = node.save_costmap(output, case_id)
                result["failure_local_costmap"] = node.save_costmap(output, case_id + ".local", "local_costmap")
            except Exception as diagnostic_exc:
                result["failure_costmap"] = {"error": str(diagnostic_exc)}
        except BaseException:
            result["status"] = "INTERRUPTED"
            raise
        finally:
            if result["full_route"] == "RUNNING":
                result["full_route"] = result["status"]
            try:
                stop_session(process)
            except Exception as exc:
                result["status"] = "FAIL"
                result["cleanup_error"] = str(exc)
            result["execution_status"] = result["status"]
            result["abnormal_process_exits"] = abnormal_exits(log_file.read_text())
            if result["abnormal_process_exits"] and result["status"] == "PASS":
                result["status"] = "FAIL"
                result["error"] = "Runtime checks passed, but one or more processes exited abnormally"
            result["wall_seconds"] = time.monotonic() - started
            result["topic_counts"] = node.counts
            result["nonempty_cloud_counts"] = node.nonempty_clouds
            result["cloud_points"] = {topic: {"min": min(sizes), "max": max(sizes),
                                              "mean": sum(sizes) / len(sizes)}
                                      for topic, sizes in node.cloud_sizes.items()}
            result["topic_sim_hz"] = {topic: (len(stamps) - 1) / (stamps[-1] - stamps[0])
                                      for topic, stamps in node.topic_stamps.items()
                                      if len(stamps) > 1 and stamps[-1] > stamps[0]}
            if len(node.clock_samples) >= 2:
                a, b = node.clock_samples[0], node.clock_samples[-1]
                result["real_time_factor"] = (b[1] - a[1]) / max(b[0] - a[0], 1e-9)
            if node.position_errors:
                result["pose_error_pairs"] = len(node.position_errors)
                result["pose_error_pair_max_delta_s"] = max(node.position_error_time_deltas)
                result["horizontal_pose_rmse_m"] = math.sqrt(
                    sum(e * e for e in node.position_errors) / len(node.position_errors))
                result["horizontal_pose_max_error_m"] = max(node.position_errors)
            (output / f"{case_id}.trace.json").write_text(json.dumps(node.trace) + "\n")
            node.planner.destroy()
            node.segment_planner.destroy()
            node.controller.destroy()
            node.destroy_node()
            result_file.write_text(json.dumps(result, indent=2) + "\n")
    print(f"{result['status']} {case_id}: {result.get('error', 'checks completed')}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("coverage", "navigation-matrix", "direct-matrix", "matrix"), default="coverage")
    parser.add_argument("--case", help="Exact canonical combination ID")
    parser.add_argument("--vehicle-id", default="ackermann_current")
    parser.add_argument("--scene-manifest", type=Path,
                        help="Use an external prepared scene and its route instead of a farm scene")
    parser.add_argument("--manipulator", choices=("configured", "true", "false"), default="configured")
    parser.add_argument("--arm-demo", choices=("configured", "true", "false"), default="configured")
    parser.add_argument("--gui", action="store_true", help="Display Gazebo during runtime validation")
    parser.add_argument("--rviz", action="store_true", help="Display RViz during runtime validation")
    parser.add_argument("--scenario", choices=("orchard", "arable_field", "greenhouse", "farmyard", "tomato_greenhouse", "strawberry_field", "vineyard", "vegetable_field", "wide_headland"),
                        default="orchard")
    parser.add_argument("--controller-filter", choices=CONTROLLERS,
                        help="Retest one controller across the selected suite")
    parser.add_argument("--scope", choices=("smoke", "full-route"), default="smoke")
    parser.add_argument("--domain", type=int, default=73)
    parser.add_argument("--rmw", choices=("rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"),
                        default=os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"))
    parser.add_argument("--reuse-domain", action="store_true",
                        help="Use one domain for a separate restart/reuse stress test")
    parser.add_argument("--startup-timeout", type=float, default=120)
    parser.add_argument("--motion-timeout", type=float, default=90)
    parser.add_argument("--stall-timeout", type=float, default=0,
                        help="Optional validation-only no-motion timeout; zero disables the observer")
    parser.add_argument("--motion-distance", type=float, default=2.0)
    parser.add_argument("--terminal-yaw-offset", type=float, default=0.0,
                        help="Smoke-only terminal heading change in radians for differential controller checks")
    parser.add_argument("--local-maneuver", action="store_true",
                        help="Plan a short forward target at the actual spawn pose instead of the scene mission")
    parser.add_argument("--maneuver-lateral-offset", type=float, default=0.0,
                        help="Local-maneuver target lateral offset in meters")
    parser.add_argument("--vision-python", default=str(
        Path.home() / ".local/share/agribot/vision_venv/bin/python3"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_-]*", args.vehicle_id):
        parser.error("Invalid vehicle ID")
    if not math.isfinite(args.motion_distance) or args.motion_distance < 1.5:
        parser.error("--motion-distance must be finite and >= 1.5m")
    if not math.isfinite(args.stall_timeout) or args.stall_timeout < 0:
        parser.error("--stall-timeout must be finite and nonnegative")
    if args.domain not in SAFE_DOMAINS:
        parser.error("Use Linux-safe domains 1..101 or 215..232, excluding Unity 37")
    if args.scope == "full-route" and args.motion_timeout < 2000:
        parser.error("Full route requires --motion-timeout >= 2000")
    if not math.isfinite(args.terminal_yaw_offset) or (
            args.terminal_yaw_offset and args.scope != "smoke"):
        parser.error("--terminal-yaw-offset must be finite and is only supported for smoke checks")
    if not math.isfinite(args.maneuver_lateral_offset) or (
            args.maneuver_lateral_offset and not args.local_maneuver):
        parser.error("Lateral offset must be finite and requires --local-maneuver")
    if args.local_maneuver and args.scope != "smoke":
        parser.error("Local maneuver checks cannot be reported as full-route acceptance")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = args.rmw
    share = Path(get_package_share_directory("agribot_vehicle_description"))
    vehicle = json.loads((share / "generated" / args.vehicle_id / "vehicle_config.json").read_text())
    if args.manipulator == "true" and args.vehicle_id not in ("ackermann_current", "unity_car3"):
        parser.error("Manipulator is supported only on Car0 and Car3")
    if args.terminal_yaw_offset and vehicle["vehicleType"] == "ackermann":
        parser.error("Terminal heading maneuvers require a differential or tracked vehicle")
    if args.rmw == "rmw_cyclonedds_cpp":
        os.environ.setdefault("CYCLONEDDS_URI", "file://" + str(
            share / "config/cyclonedds_simulation.xml"))
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("XAUTHORITY", str(Path.home() / ".Xauthority"))
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{11400 + args.domain}"
    output = args.output or (Path.home() / ".local/state/agribot-algorithm-validation" /
                             time.strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    if (output / "summary.json").exists():
        parser.error("Output already contains a run; choose a new path to preserve evidence")
    cases = coverage_cases() if args.suite == "coverage" else list(itertools.product(
        PERCEPTIONS, VISIONS if args.suite == "matrix" else ("off",),
        LOCALIZATIONS, ("direct",) if args.suite == "direct-matrix" else PLANNERS, CONTROLLERS))
    if args.case:
        cases = [case for case in itertools.product(
            PERCEPTIONS, VISIONS, LOCALIZATIONS, PLANNERS, CONTROLLERS)
                 if "_".join(case) == args.case]
        if not cases:
            parser.error("Unknown canonical combination ID")
    if args.controller_filter:
        cases = [case for case in cases if case[-1] == args.controller_filter]
    waypoint_file = Path(get_package_share_directory("agribot_ackermann_mppi")) / \
        "config/orchard_waypoints_ackermann_smac.yaml"
    scenario_files = []
    if args.scene_manifest:
        spec = importlib.util.spec_from_file_location(
            "runtime_external_scene", share / "launch/configured_ackermann_sim.launch.py")
        external_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(external_module)
        external = external_module._external_scene_bundle(args.scene_manifest)
        waypoint_file = Path(external["waypoints"])
        scenario_files = [args.scene_manifest.resolve(), Path(external["world"]), Path(external["map"])]
        image = Path(yaml.safe_load(Path(external["map"]).read_text())["image"])
        scenario_files.append(image if image.is_absolute() else Path(external["map"]).parent / image)
    elif args.scenario != "orchard":
        from agribot_farm_sim.scenarios import build, write_vehicle_route
        scene = build(args.scenario, Path.home() / ".cache/agribot/scenarios")
        waypoint_file = write_vehicle_route(args.scenario, vehicle, scene)
        scenario_files = [scene / name for name in ("world.sdf", "map.yaml", "geometry.json")]
        image = Path(yaml.safe_load((scene / "map.yaml").read_text())["image"])
        scenario_files.append(image if image.is_absolute() else scene / image)
        scenario_files.append(Path(get_package_share_directory("agribot_farm_sim")) / "config/scenarios.json")
    waypoints = [pose(p["x"], p["y"], p["yaw"])
                 for p in yaml.safe_load(waypoint_file.read_text())["waypoints"]]
    summary = {"scope": args.scope, "vehicle_id": args.vehicle_id,
               "scenario": args.scenario,
               "manipulator": args.manipulator, "arm_demo": args.arm_demo,
               "planned_cases": len(cases), "results": [],
               "motion_distance_m": args.motion_distance,
               "terminal_yaw_offset_rad": args.terminal_yaw_offset,
               "maneuver_lateral_offset_m": args.maneuver_lateral_offset,
               "local_maneuver": args.local_maneuver,
               "case_ids": ["_".join(case) for case in cases],
               "claim": "Smoke PASS is not complete-route acceptance"}
    if args.scene_manifest:
        summary["scene_manifest"] = str(args.scene_manifest.resolve())
        summary["scenario"] = "external_scene"
    summary["host"] = {"machine": platform.machine(), "node": platform.node(),
                       "os": platform.platform(), "ros_distro": os.environ.get("ROS_DISTRO"),
                       "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}
    snapshot_files = scenario_files + [Path(__file__), share / "launch/configured_ackermann_sim.launch.py",
                      Path(get_package_prefix("agribot_autonomy")) /
                      "lib/agribot_autonomy/snake_waypoint_runner.py",
                      share / f"generated/{args.vehicle_id}/vehicle_config.json", waypoint_file,
                      share / f"generated/{args.vehicle_id}/models/{args.vehicle_id}.sdf",
                      share / f"generated/{args.vehicle_id}/urdf/{args.vehicle_id}.urdf.xacro",
                      share / "config/cyclonedds_simulation.xml", direct_builder_path(),
                      Path(get_package_prefix("agribot_ackermann_mppi")) /
                      "lib/agribot_ackermann_mppi/sim_navsat_bridge.py",
                      Path(get_package_prefix("agribot_ackermann_mppi")) /
                      "lib/libagribot_ackermann_mppi_drive.so",
                      Path(get_package_prefix("agribot_hardware_bringup")) /
                      "lib/agribot_hardware_bringup/navsat_pose_bridge.py",
                      Path(get_package_prefix("fast_livo")) / "lib/fast_livo/fastlivo_mapping"]
    snapshot_files += [share / "config/ackermann_mppi_simulation.yaml",
                      share / "config/nominal_ackermann_navigation.yaml",
                      share / "config/tracked_contact_simulation.yaml",
                      share / "config/terminal_stop_simulation.yaml",
                      share / "config/behavior_trees/navigate_initial_plan_with_recovery.xml",
                      share / "config/behavior_trees/navigate_through_poses_initial_plan_with_recovery.xml",
                      share / "config/rpp_controller.yaml",
                      share / "config/dwb_controller.yaml",
                      share / "config/differential_controllers.yaml",
                      Path(get_package_prefix("agribot_vehicle_description")) /
                      "lib/agribot_vehicle_description/simulation_lidar_time_adapter.py",
                      Path(get_package_prefix("agribot_vehicle_description")) /
                      "lib/agribot_vehicle_description/simulation_initial_pose.py",
                      Path(get_package_prefix("agribot_vehicle_description")) /
                      "lib/libagribot_ackermann_trajectory_generator.so",
                      Path(get_package_prefix("agribot_vehicle_description")) /
                      "lib/libagribot_route_progress_goal_checker.so",
                      Path(get_package_share_directory("agribot_ackermann_mppi")) /
                      "launch/ackermann_waypoint_depth_collect_sim.launch.py"]
    snapshot_files += list((share / "generated" / args.vehicle_id / "config").rglob("*.yaml"))
    snapshot_files.append(Path(get_package_prefix("agribot_vehicle_description")) /
                          "lib/agribot_vehicle_description/simulation_vehicle_cloud_filter.py")
    if args.rviz:
        snapshot_files += [
            Path(get_package_share_directory("scout_viz")) / "scripts/run_rviz2_clean_env.sh",
            Path(get_package_prefix("scout_viz")) / "lib/libscout_image_display.so",
        ]
    if any(case[-1] == "rpp" for case in cases):
        snapshot_files += [share / "position_only_rpp_plugins.xml",
            Path(get_package_prefix("agribot_vehicle_description")) /
            "lib/libagribot_position_only_rpp_controller.so"]
    if args.gui:
        snapshot_files.append(Path(get_package_prefix("agribot_ackermann_mppi")) /
                              "lib/libagribot_gazebo_local_model_catalog.so")
    compat = Path(get_package_prefix("agribot_vehicle_description")) / \
        "lib/agribot_gazebo_transport_compat/libgazebo_transport.so.11"
    if compat.is_file():
        snapshot_files.append(compat)
    summary["file_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in snapshot_files}
    summary["git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    summary["git_status"] = subprocess.check_output(["git", "status", "--short"], text=True)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Runtime evidence: {output}; {len(cases)} cases; scope={args.scope}", flush=True)
    try:
        for index, case in enumerate(cases):
            domain = case_domain(args.domain, index, args.reuse_domain)
            os.environ["ROS_DOMAIN_ID"] = str(domain)
            os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{11400 + domain}"
            # Fresh DDS participants prevent a stopped case's discovery cache
            # or action entities from contaminating the next algorithm check.
            rclpy.init()
            try:
                summary["results"].append(run_case(case, args, output, waypoints))
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
            (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    except KeyboardInterrupt:
        result_file = output / ("_".join(case) + ".json")
        if result_file.exists():
            summary["results"].append(json.loads(result_file.read_text()))
        summary["interrupted"] = True
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if any(item["status"] != "PASS" for item in summary["results"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
