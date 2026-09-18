#!/usr/bin/env python3

from collections import deque
import math
from pathlib import Path
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import (
    ComputePathThroughPoses,
    FollowPath,
    NavigateThroughPoses,
    NavigateToPose,
)
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def poses_are_stable(samples, position_tolerance: float, yaw_tolerance: float) -> bool:
    if not samples:
        return False
    reference_x, reference_y, reference_yaw = samples[-1]
    return all(
        math.hypot(x - reference_x, y - reference_y) <= position_tolerance
        and abs(normalize_angle(yaw - reference_yaw)) <= yaw_tolerance
        for x, y, yaw in samples
    )


def load_waypoints(path: str):
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    waypoints = data.get("waypoints", [])
    if not waypoints:
        raise RuntimeError(f"No waypoints found in {path}")
    return waypoints


def transform_waypoint(waypoint, origin_x, origin_y, origin_yaw):
    dx = float(waypoint["x"]) - origin_x
    dy = float(waypoint["y"]) - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    transformed_x = cos_yaw * dx + sin_yaw * dy
    transformed_y = -sin_yaw * dx + cos_yaw * dy
    transformed_yaw = normalize_angle(float(waypoint.get("yaw", 0.0)) - origin_yaw)
    return {"x": transformed_x, "y": transformed_y, "yaw": transformed_yaw}


def build_segments(key_waypoints, start_pos, step=2.0):
    """Build navigation segments: each key waypoint has a list of interpolation
    points leading up to it from the previous key waypoint.

    Returns a list of segments:
    [
      {"key_wp": key_point_0, "interp": [interp_a, interp_b, ...]},
      {"key_wp": key_point_1, "interp": [interp_c, interp_d, ...]},
      ...
    ]

    The interp points in each segment are between the previous key point
    (or start_pos for the first segment) and this key point.
    interp points are ordered from near-prev to near-key, excluding endpoints.
    """
    if not key_waypoints:
        return []

    all_points = [start_pos] + key_waypoints

    segments = []
    for i in range(1, len(all_points)):
        x0 = float(all_points[i - 1]["x"])
        y0 = float(all_points[i - 1]["y"])
        x1 = float(all_points[i]["x"])
        y1 = float(all_points[i]["y"])
        yaw_key = float(all_points[i].get("yaw", 0.0))
        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx * dx + dy * dy)
        segment_yaw = math.atan2(dy, dx)

        interp = []
        if dist > step:
            n = int(dist / step)
            for k in range(1, n):
                t = k / n
                ix = x0 + t * dx
                iy = y0 + t * dy
                interp.append({"x": ix, "y": iy, "yaw": normalize_angle(segment_yaw)})

        segments.append({
            "key_wp": {"x": x1, "y": y1, "yaw": normalize_angle(yaw_key)},
            "interp": interp,
        })

    return segments


def build_path_points(key_waypoints, start_pos, step=0.5):
    """Build a single continuous path through all key waypoints.

    The input waypoints are the user-authored hard constraints. Interpolation
    here only densifies the controller path; these points are not dispatched as
    individual action goals.
    """
    if not key_waypoints:
        return [start_pos]

    all_points = [start_pos] + key_waypoints
    path_points = []

    for i in range(len(all_points) - 1):
        current_point = all_points[i]
        next_point = all_points[i + 1]
        x0 = float(current_point["x"])
        y0 = float(current_point["y"])
        x1 = float(next_point["x"])
        y1 = float(next_point["y"])
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        segment_yaw = math.atan2(dy, dx) if dist > 1e-6 else float(next_point.get("yaw", 0.0))
        segment_steps = max(1, int(math.ceil(dist / max(step, 1e-3))))

        if i == 0:
            start_wp = dict(current_point)
            start_wp["yaw"] = normalize_angle(segment_yaw)
            path_points.append(start_wp)

        for k in range(1, segment_steps):
            t = k / segment_steps
            path_points.append(
                {
                    "x": x0 + t * dx,
                    "y": y0 + t * dy,
                    "yaw": normalize_angle(segment_yaw),
                }
            )

        end_wp = dict(next_point)
        if i < len(all_points) - 2:
            end_wp["yaw"] = normalize_angle(segment_yaw)
        else:
            end_wp["yaw"] = normalize_angle(float(next_point.get("yaw", segment_yaw)))
        path_points.append(end_wp)

    return path_points


class SnakeWaypointRunner(Node):
    def __init__(self) -> None:
        super().__init__("snake_waypoint_runner")
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.startup_delay = float(self.declare_parameter("startup_delay", 5.0).value)
        self.navigation_mode = self.declare_parameter("navigation_mode", "follow_path").value
        self.action_name = self.declare_parameter("action_name", "navigate_to_pose").value
        self.through_poses_action_name = self.declare_parameter(
            "through_poses_action_name", "navigate_through_poses"
        ).value
        self.planner_action_name = self.declare_parameter(
            "planner_action_name", "compute_path_through_poses"
        ).value
        self.planner_id = self.declare_parameter("planner_id", "GridBased").value
        self.path_action_name = self.declare_parameter("path_action_name", "follow_path").value
        self.controller_id = self.declare_parameter("controller_id", "FollowPath").value
        self.waypoint_file = self.declare_parameter("waypoint_file", "").value
        self.stop_on_failure = bool(self.declare_parameter("stop_on_failure", True).value)
        self.retries_per_waypoint = int(self.declare_parameter("retries_per_waypoint", 0).value)
        self.waypoint_transform_enabled = bool(
            self.declare_parameter("waypoint_transform_enabled", False).value
        )
        self.source_origin_x = float(self.declare_parameter("waypoint_source_origin_x", 0.0).value)
        self.source_origin_y = float(self.declare_parameter("waypoint_source_origin_y", 0.0).value)
        self.source_origin_yaw = float(self.declare_parameter("waypoint_source_origin_yaw", 0.0).value)
        self.goal_topic = self.declare_parameter("goal_topic", "current_goal").value
        self.goal_array_topic = self.declare_parameter("goal_array_topic", "current_goal_array").value
        self.sequence_topic = self.declare_parameter("sequence_topic", "waypoint_sequence").value
        self.transition_delay = float(self.declare_parameter("transition_delay", 1.5).value)
        # Start position from launch params (map coordinates, same as spawn position)
        self.initial_pose_x = float(self.declare_parameter("initial_pose_x", 0.0).value)
        self.initial_pose_y = float(self.declare_parameter("initial_pose_y", 0.0).value)
        self.initial_pose_yaw = float(self.declare_parameter("initial_pose_yaw", 0.0).value)
        self.advance_distance = float(self.declare_parameter("advance_distance", 2.0).value)
        self.path_step = float(self.declare_parameter("path_step", 0.5).value)
        self.proximity_advance_enabled = bool(
            self.declare_parameter("proximity_advance_enabled", False).value
        )
        self.path_topic = self.declare_parameter("path_topic", "waypoint_path").value
        self.plan_topic = self.declare_parameter("plan_topic", "/plan").value
        self.require_pose_before_start = bool(
            self.declare_parameter("require_pose_before_start", False).value
        )
        self.preplan_display_delay = float(
            self.declare_parameter("preplan_display_delay", 0.5).value
        )
        self.preplan_waypoint_tolerance = float(
            self.declare_parameter("preplan_waypoint_tolerance", 0.35).value
        )
        self.preplan_retry_delay = float(
            self.declare_parameter("preplan_retry_delay", 0.5).value
        )
        self.readiness_gate_enabled = bool(
            self.declare_parameter("readiness_gate_enabled", False).value
        )
        self.readiness_pose_samples_required = max(
            1, int(self.declare_parameter("readiness_pose_samples", 1).value)
        )
        self.readiness_position_tolerance = max(
            0.0,
            float(self.declare_parameter("readiness_position_tolerance", 0.05).value),
        )
        self.readiness_yaw_tolerance = max(
            0.0,
            float(self.declare_parameter("readiness_yaw_tolerance", 0.03).value),
        )
        self.readiness_settle_time = max(
            0.0, float(self.declare_parameter("readiness_settle_time", 0.0).value)
        )
        lifecycle_services = {
            "planner_server": self.declare_parameter(
                "planner_lifecycle_service", "/planner_server/get_state"
            ).value,
            "controller_server": self.declare_parameter(
                "controller_lifecycle_service", "/controller_server/get_state"
            ).value,
            "global_costmap": self.declare_parameter(
                "costmap_lifecycle_service",
                "/global_costmap/global_costmap/get_state",
            ).value,
        }
        self.lifecycle_clients = {}
        self.lifecycle_futures = {}
        self.lifecycle_active = {}
        if self.readiness_gate_enabled:
            for name, service_name in lifecycle_services.items():
                self.lifecycle_clients[name] = self.create_client(GetState, service_name)
                self.lifecycle_futures[name] = None
                self.lifecycle_active[name] = False
        self.lifecycle_ready_since = None
        self.readiness_pose_samples = deque(
            maxlen=self.readiness_pose_samples_required
        )
        self._last_readiness_reason = ""
        self._last_wait_log_time = 0.0

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, latched_qos)
        self.goal_array_pub = self.create_publisher(PoseArray, self.goal_array_topic, latched_qos)
        self.sequence_pub = self.create_publisher(PoseArray, self.sequence_topic, latched_qos)
        self.path_pub = self.create_publisher(NavPath, self.path_topic, latched_qos)
        self.plan_pub = self.create_publisher(NavPath, self.plan_topic, latched_qos)
        self.pose_client = ActionClient(self, NavigateToPose, self.action_name)
        self.through_poses_client = ActionClient(
            self, NavigateThroughPoses, self.through_poses_action_name
        )
        self.planner_client = ActionClient(
            self, ComputePathThroughPoses, self.planner_action_name
        )
        self.path_client = ActionClient(self, FollowPath, self.path_action_name)
        self.timer = self.create_timer(self.startup_delay, self.run_once)

        # Subscribe to /amcl_pose for robot position in map frame
        # ground_truth_localization publishes PoseWithCovarianceStamped here
        # with the correct map-frame coordinates (odom→map transform applied)
        self.robot_x = self.initial_pose_x
        self.robot_y = self.initial_pose_y
        self.robot_yaw = self.initial_pose_yaw
        self.pose_received = False
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
        )

        self.key_waypoints = []
        self.segments = []
        self.path_points = []
        self.path_progress_index = 0
        self.seg_idx = 0
        self.interp_idx = 0
        self.active_goal_handle = None
        self.advance_timer = None
        self.path_retry_timer = None
        self.through_poses_retry_timer = None
        self.preplan_dispatch_timer = None
        self.preplan_retry_timer = None
        self.preplanned_path = None
        self.preplan_waypoints = []
        self.preplan_waypoint_path_indices = []
        self.preplan_progress_index = 0
        self.preplan_completed_waypoints = 0
        self.ran = False
        self.attempts_remaining = 0

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # /amcl_pose is in map frame — correct coordinates
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose_received = True
        if all(
            math.isfinite(value)
            for value in (self.robot_x, self.robot_y, self.robot_yaw)
        ):
            self.readiness_pose_samples.append(
                (self.robot_x, self.robot_y, self.robot_yaw)
            )
        self._update_preplanned_progress()

    def _poll_lifecycle_states(self) -> None:
        for name, client in self.lifecycle_clients.items():
            if (
                self.lifecycle_active[name]
                or self.lifecycle_futures[name] is not None
            ):
                continue
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_futures[name] = future
            future.add_done_callback(
                lambda completed, lifecycle_name=name: self._on_lifecycle_state(
                    lifecycle_name, completed
                )
            )

    def _on_lifecycle_state(self, name: str, future) -> None:
        self.lifecycle_futures[name] = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to query {name} lifecycle state: {exc}"
            )
            return
        self.lifecycle_active[name] = (
            response.current_state.id == State.PRIMARY_STATE_ACTIVE
        )

    def _readiness_wait_reason(self):
        if self.require_pose_before_start and not self.pose_received:
            return "waiting for the first /amcl_pose"
        if not self.readiness_gate_enabled:
            return None

        self._poll_lifecycle_states()
        inactive = [
            name for name, active in self.lifecycle_active.items() if not active
        ]
        if inactive:
            self.lifecycle_ready_since = None
            return f"waiting for active lifecycle nodes: {', '.join(inactive)}"

        if self.lifecycle_ready_since is None:
            self.lifecycle_ready_since = time.monotonic()
            self.readiness_pose_samples.clear()
            return (
                "lifecycle nodes are active; waiting for post-activation "
                "costmap updates"
            )

        settle_remaining = self.readiness_settle_time - (
            time.monotonic() - self.lifecycle_ready_since
        )
        if settle_remaining > 0.0:
            return (
                "waiting for post-activation costmap updates "
                f"({settle_remaining:.1f} s remaining)"
            )

        if len(self.readiness_pose_samples) < self.readiness_pose_samples_required:
            return (
                "waiting for stable localization samples "
                f"({len(self.readiness_pose_samples)}/"
                f"{self.readiness_pose_samples_required})"
            )
        if not poses_are_stable(
            self.readiness_pose_samples,
            self.readiness_position_tolerance,
            self.readiness_yaw_tolerance,
        ):
            return "waiting for localization pose to become stable"
        return None

    def _update_preplanned_progress(self) -> None:
        """Track progress monotonically so retries keep every unvisited waypoint."""
        if self.preplanned_path is None or not self.preplanned_path.poses:
            return

        search_start = min(
            self.preplan_progress_index, len(self.preplanned_path.poses) - 1
        )
        search_end = min(search_start + 200, len(self.preplanned_path.poses))
        nearest_index = min(
            range(search_start, search_end),
            key=lambda index: math.hypot(
                self.preplanned_path.poses[index].pose.position.x - self.robot_x,
                self.preplanned_path.poses[index].pose.position.y - self.robot_y,
            ),
        )
        self.preplan_progress_index = max(self.preplan_progress_index, nearest_index)

        while (
            self.preplan_completed_waypoints
            < len(self.preplan_waypoint_path_indices)
            and self.preplan_progress_index
            >= self.preplan_waypoint_path_indices[self.preplan_completed_waypoints]
        ):
            self.preplan_completed_waypoints += 1

    def _robot_distance_to(self, waypoint) -> float:
        return math.hypot(
            float(waypoint["x"]) - self.robot_x,
            float(waypoint["y"]) - self.robot_y,
        )

    def _current_goal(self):
        """Get the current navigation target point."""
        if self.seg_idx >= len(self.segments):
            return None
        seg = self.segments[self.seg_idx]
        if self.interp_idx < len(seg["interp"]):
            return seg["interp"][self.interp_idx]
        return seg["key_wp"]

    def _advance_goal_index(self) -> None:
        if self.seg_idx >= len(self.segments):
            return
        if self._is_at_key_wp():
            self.seg_idx += 1
            self.interp_idx = 0
        else:
            self.interp_idx += 1

    def _skip_nearby_goals(self) -> None:
        while True:
            goal_wp = self._current_goal()
            if goal_wp is None:
                return
            if self._robot_distance_to(goal_wp) > self.advance_distance:
                return
            self.get_logger().info(
                f"Skipping nearby goal ({goal_wp['x']:.2f}, {goal_wp['y']:.2f}); "
                f"distance {self._robot_distance_to(goal_wp):.2f} m <= "
                f"advance_distance {self.advance_distance:.2f} m"
            )
            self._advance_goal_index()

    def _is_at_key_wp(self) -> bool:
        """Check if we are currently targeting the key waypoint of this segment."""
        if self.seg_idx >= len(self.segments):
            return False
        seg = self.segments[self.seg_idx]
        return self.interp_idx >= len(seg["interp"])

    def _make_pose(self, waypoint) -> Pose:
        pose = Pose()
        pose.position.x = float(waypoint["x"])
        pose.position.y = float(waypoint["y"])
        yaw = float(waypoint.get("yaw", 0.0))
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _make_pose_array(self, waypoints) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = [self._make_pose(waypoint) for waypoint in waypoints]
        return pose_array

    def _flatten_sequence(self):
        ordered_waypoints = []
        for segment in self.segments:
            ordered_waypoints.extend(segment["interp"])
            ordered_waypoints.append(segment["key_wp"])
        return ordered_waypoints

    def _publish_sequence(self) -> None:
        self.sequence_pub.publish(self._make_pose_array(self._flatten_sequence()))

    def _publish_current_goal_visual(self, goal_wp) -> None:
        self.goal_array_pub.publish(self._make_pose_array([goal_wp]))

    def _publish_key_waypoints(self, key_waypoints) -> None:
        self.sequence_pub.publish(self._make_pose_array(key_waypoints))

    def _publish_path_visuals(self, path: NavPath) -> None:
        self.path_pub.publish(path)
        self.plan_pub.publish(path)

    def _make_path(self, waypoints) -> NavPath:
        path = NavPath()
        stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        path.poses = []
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = self._make_pose(waypoint)
            path.poses.append(pose_stamped)
        return path

    def make_goal(self, waypoint):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self._make_pose(waypoint)
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        return goal, goal_pose

    def make_path_goal(self, waypoints):
        goal = FollowPath.Goal()
        goal.path = self._make_path(waypoints)
        goal.controller_id = self.controller_id
        return goal

    def make_through_poses_goal(self, waypoints):
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = self._make_pose(waypoint)
            goal.poses.append(pose_stamped)
        return goal

    def make_preplan_goal(self, waypoints):
        goal = ComputePathThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = self._make_pose(waypoint)
            goal.goals.append(pose_stamped)
        goal.planner_id = self.planner_id
        goal.use_start = False
        return goal

    def _preplanned_path_waypoint_indices(self, path: NavPath, waypoints):
        """Verify every authored waypoint occurs in order on the Smac result."""
        cursor = 0
        matched_indices = []
        for waypoint_index, waypoint in enumerate(waypoints, start=1):
            matched_index = None
            for path_index in range(cursor, len(path.poses)):
                position = path.poses[path_index].pose.position
                distance = math.hypot(
                    position.x - float(waypoint["x"]),
                    position.y - float(waypoint["y"]),
                )
                if distance <= self.preplan_waypoint_tolerance:
                    matched_index = path_index
                    break
            if matched_index is None:
                self.get_logger().error(
                    f"Global path does not contain key waypoint "
                    f"{waypoint_index}/{len(waypoints)} within "
                    f"{self.preplan_waypoint_tolerance:.2f} m; vehicle remains stopped"
                )
                return None
            matched_indices.append(matched_index)
            cursor = matched_index
        return matched_indices

    def _request_preplanned_path(self) -> None:
        if self.preplan_retry_timer is not None:
            self.destroy_timer(self.preplan_retry_timer)
            self.preplan_retry_timer = None
        if not self.preplan_waypoints:
            self.get_logger().error("Preplanned route is empty; vehicle remains stopped")
            return

        final_goal = self.preplan_waypoints[-1]
        final_goal_pose = PoseStamped()
        final_goal_pose.header.frame_id = self.frame_id
        final_goal_pose.header.stamp = self.get_clock().now().to_msg()
        final_goal_pose.pose = self._make_pose(final_goal)
        self.goal_pub.publish(final_goal_pose)
        self._publish_current_goal_visual(final_goal)
        self._publish_key_waypoints(self.key_waypoints)

        self.get_logger().info(
            f"Preplanning one continuous global path through all "
            f"{len(self.preplan_waypoints)} remaining key waypoints; vehicle remains stopped"
        )
        future = self.planner_client.send_goal_async(
            self.make_preplan_goal(self.preplan_waypoints)
        )
        future.add_done_callback(self._on_preplan_goal_response)

    def _schedule_preplan_retry(self) -> None:
        if self.preplan_retry_timer is not None:
            return
        delay = max(self.preplan_retry_delay, 0.1)
        self.get_logger().info(
            f"Retrying the complete Smac preplan in {delay:.1f} s "
            "while the vehicle remains stopped"
        )
        self.preplan_retry_timer = self.create_timer(
            delay, self._request_preplanned_path
        )

    def _on_preplan_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to request Smac preplan: {exc}; vehicle remains stopped"
            )
            self._schedule_preplan_retry()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Smac preplan request was rejected; vehicle remains stopped"
            )
            self._schedule_preplan_retry()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_preplan_result)

    def _on_preplan_result(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to receive Smac preplan: {exc}; vehicle remains stopped"
            )
            self._schedule_preplan_retry()
            return

        if getattr(wrapped_result, "status", None) != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Smac preplan failed with status "
                f"{getattr(wrapped_result, 'status', None)}; vehicle remains stopped"
            )
            self._schedule_preplan_retry()
            return

        path = wrapped_result.result.path
        if len(path.poses) < 2:
            self.get_logger().error(
                "Global planner returned an empty path; vehicle remains stopped"
            )
            self._schedule_preplan_retry()
            return
        waypoint_path_indices = self._preplanned_path_waypoint_indices(
            path, self.preplan_waypoints
        )
        if waypoint_path_indices is None:
            self._schedule_preplan_retry()
            return

        self.preplanned_path = path
        self.preplan_waypoint_path_indices = waypoint_path_indices
        self.preplan_progress_index = 0
        self.preplan_completed_waypoints = 0
        self._publish_path_visuals(path)
        duration = wrapped_result.result.planning_time
        planning_seconds = duration.sec + duration.nanosec / 1e9
        self.get_logger().info(
            f"Global preplan ready: {len(path.poses)} poses, "
            f"all {len(self.preplan_waypoints)} remaining key waypoints verified in order, "
            f"planning time {planning_seconds:.3f} s"
        )
        self.get_logger().info(
            f"Published the complete path to RViz; controller dispatch waits "
            f"{self.preplan_display_delay:.2f} s"
        )
        self.preplan_dispatch_timer = self.create_timer(
            max(self.preplan_display_delay, 0.01), self._dispatch_preplanned_path
        )

    def _dispatch_preplanned_path(self) -> None:
        if self.preplan_dispatch_timer is not None:
            self.destroy_timer(self.preplan_dispatch_timer)
            self.preplan_dispatch_timer = None
        if self.preplanned_path is None:
            self.get_logger().error(
                "No verified global path to execute; vehicle remains stopped"
            )
            return

        goal = FollowPath.Goal()
        goal.path = self.preplanned_path
        goal.controller_id = self.controller_id
        self.get_logger().info(
            "Complete global path is visible and verified; dispatching it to the selected controller"
        )
        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(self._on_path_goal_response)

    def _send_through_poses(self) -> None:
        if not self.key_waypoints:
            self.get_logger().error("Through-poses route is empty; nothing to send")
            return

        final_goal = self.key_waypoints[-1]
        final_goal_pose = PoseStamped()
        final_goal_pose.header.frame_id = self.frame_id
        final_goal_pose.header.stamp = self.get_clock().now().to_msg()
        final_goal_pose.pose = self._make_pose(final_goal)
        self.goal_pub.publish(final_goal_pose)
        self._publish_current_goal_visual(final_goal)
        self._publish_key_waypoints(self.key_waypoints)

        self.get_logger().info(
            f"Sending {len(self.key_waypoints)} key waypoints through "
            f"{self.through_poses_action_name}; final goal "
            f"({final_goal['x']:.2f}, {final_goal['y']:.2f})"
        )
        send_goal_future = self.through_poses_client.send_goal_async(
            self.make_through_poses_goal(self.key_waypoints)
        )
        send_goal_future.add_done_callback(self._on_through_poses_goal_response)

    def _on_through_poses_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send through-poses goal: {exc}")
            self._handle_through_poses_failure(status=None)
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Through-poses goal was rejected")
            self._handle_through_poses_failure(status=None)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_through_poses_result)

    def _on_through_poses_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to receive through-poses result: {exc}")
            self._handle_through_poses_failure(status=None)
            return

        status = getattr(result, "status", None)
        self.active_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.through_poses_retry_timer is not None:
                self.destroy_timer(self.through_poses_retry_timer)
                self.through_poses_retry_timer = None
            self.get_logger().info("Through-poses route completed successfully")
            return
        if status == GoalStatus.STATUS_CANCELED:
            return
        self._handle_through_poses_failure(status=status)

    def _handle_through_poses_failure(self, status) -> None:
        self.active_goal_handle = None
        self.attempts_remaining -= 1
        self.get_logger().warn(
            f"Through-poses route failed with status {status}, "
            f"remaining retries: {self.attempts_remaining}"
        )
        if self.attempts_remaining <= 0 and self.stop_on_failure:
            self.get_logger().error("Stopping through-poses navigation")
            return
        if self.attempts_remaining <= 0:
            self.attempts_remaining = self.retries_per_waypoint + 1
        if self.through_poses_retry_timer is None:
            self.through_poses_retry_timer = self.create_timer(
                max(self.transition_delay, 0.1), self._retry_through_poses
            )

    def _retry_through_poses(self) -> None:
        if self.through_poses_retry_timer is not None:
            self.destroy_timer(self.through_poses_retry_timer)
            self.through_poses_retry_timer = None
        self._send_through_poses()

    def _send_current_goal(self) -> None:
        """Send the current navigation target as a goal."""
        self._skip_nearby_goals()
        goal_wp = self._current_goal()
        if goal_wp is None:
            self.get_logger().info("All waypoints completed!")
            self._stop_advance_timer()
            return

        goal, goal_pose = self.make_goal(goal_wp)
        self.goal_pub.publish(goal_pose)
        self._publish_current_goal_visual(goal_wp)

        if self._is_at_key_wp():
            self.get_logger().info(
                f"Targeting key waypoint {self.seg_idx + 1}/{len(self.segments)}: "
                f"({goal_wp['x']:.2f}, {goal_wp['y']:.2f})"
            )
        else:
            self.get_logger().info(
                f"Targeting interp {self.interp_idx + 1}/{len(self.segments[self.seg_idx]['interp'])} "
                f"in segment {self.seg_idx + 1}/{len(self.segments)}: "
                f"({goal_wp['x']:.2f}, {goal_wp['y']:.2f})"
            )

        # Cancel previous goal if active
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        send_goal_future = self.pose_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _remaining_path_points(self):
        """Return a path that starts at the current pose and preserves route order."""
        if not self.path_points or not self.pose_received:
            return self.path_points

        search_start = min(self.path_progress_index, len(self.path_points) - 1)
        nearest_index = min(
            range(search_start, len(self.path_points)),
            key=lambda index: math.hypot(
                float(self.path_points[index]["x"]) - self.robot_x,
                float(self.path_points[index]["y"]) - self.robot_y,
            ),
        )
        self.path_progress_index = nearest_index

        remaining = [dict(point) for point in self.path_points[nearest_index:]]
        current_pose = {
            "x": self.robot_x,
            "y": self.robot_y,
            "yaw": self.robot_yaw,
        }
        if self._robot_distance_to(remaining[0]) <= 0.05:
            remaining[0] = current_pose
        else:
            remaining.insert(0, current_pose)

        self.get_logger().info(
            f"Resuming continuous path at sample {nearest_index}/{len(self.path_points) - 1} "
            f"from robot pose ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )
        return remaining

    def _send_continuous_path(self, key_waypoints, resume=False) -> None:
        if not self.path_points:
            self.get_logger().error("Continuous path is empty; nothing to send")
            return

        final_goal = key_waypoints[-1]
        final_goal_pose = PoseStamped()
        final_goal_pose.header.frame_id = self.frame_id
        final_goal_pose.header.stamp = self.get_clock().now().to_msg()
        final_goal_pose.pose = self._make_pose(final_goal)
        self.goal_pub.publish(final_goal_pose)
        self._publish_current_goal_visual(final_goal)

        dispatch_points = self._remaining_path_points() if resume else self.path_points
        path_goal = self.make_path_goal(dispatch_points)
        self._publish_path_visuals(path_goal.path)
        self.get_logger().info(
            f"Targeting continuous path: {len(key_waypoints)} key waypoints, "
            f"{len(dispatch_points)} remaining path samples, final goal "
            f"({final_goal['x']:.2f}, {final_goal['y']:.2f})"
        )

        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        send_goal_future = self.path_client.send_goal_async(path_goal)
        send_goal_future.add_done_callback(self._on_path_goal_response)

    def _schedule_continuous_path_retry(self) -> None:
        if self.path_retry_timer is not None:
            return

        retry_delay = max(self.transition_delay, 0.1)
        self.get_logger().info(
            f"Retrying continuous path from the current pose in {retry_delay:.1f} s"
        )
        self.path_retry_timer = self.create_timer(
            retry_delay, self._retry_continuous_path
        )

    def _retry_continuous_path(self) -> None:
        if self.path_retry_timer is not None:
            self.destroy_timer(self.path_retry_timer)
            self.path_retry_timer = None
        self._send_continuous_path(self.key_waypoints, resume=True)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            self._handle_failure(status=None)
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected")
            self._handle_failure(status=None)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_path_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send path goal: {exc}")
            self._handle_path_failure(status=None)
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Continuous path goal was rejected")
            self._handle_path_failure(status=None)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_path_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to receive result: {exc}")
            self._handle_failure(status=None)
            return

        status = getattr(result, "status", None)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.active_goal_handle = None
            self._on_goal_succeeded()
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.active_goal_handle = None
            return

        self._handle_failure(status=status)

    def _on_path_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to receive path result: {exc}")
            self._handle_path_failure(status=None)
            return

        status = getattr(result, "status", None)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.active_goal_handle = None
            if self.path_retry_timer is not None:
                self.destroy_timer(self.path_retry_timer)
                self.path_retry_timer = None
            self.get_logger().info("Continuous path completed successfully")
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.active_goal_handle = None
            return

        if self.navigation_mode == "plan_then_follow_path":
            self._handle_preplanned_path_failure(status=status)
            return

        self._handle_path_failure(status=status)

    def _handle_preplanned_path_failure(self, status) -> None:
        """Replan from the live pose without dropping any unvisited route point."""
        self.active_goal_handle = None
        completed = min(
            self.preplan_completed_waypoints, len(self.preplan_waypoints)
        )
        remaining = self.preplan_waypoints[completed:]
        if not remaining:
            self.get_logger().info(
                "Controller ended after every mandatory waypoint was traversed"
            )
            return

        self.get_logger().warn(
            f"Preplanned path failed with status {status}; "
            f"replanning from the current pose through all {len(remaining)} "
            "unvisited mandatory waypoints"
        )
        self.preplan_waypoints = remaining
        self.preplanned_path = None
        self.preplan_waypoint_path_indices = []
        self.preplan_progress_index = 0
        self.preplan_completed_waypoints = 0
        self._schedule_preplan_retry()

    def _on_goal_succeeded(self) -> None:
        """Called when nav2 reports the current goal was reached."""
        if self._is_at_key_wp():
            self._advance_goal_index()
            self.attempts_remaining = self.retries_per_waypoint + 1
            self.get_logger().info(
                f"Reached key waypoint, advancing to segment {self.seg_idx + 1}"
            )
            self._send_current_goal()
        else:
            self._advance_goal_index()
            self._send_current_goal()

    def _handle_failure(self, status) -> None:
        self.active_goal_handle = None
        self.attempts_remaining -= 1
        self.get_logger().warn(
            f"Goal failed with status {status}, "
            f"remaining retries: {self.attempts_remaining}"
        )
        if self.attempts_remaining > 0:
            self._send_current_goal()
            return

        if self.stop_on_failure:
            self.get_logger().error("Stopping waypoint sequence")
            self._stop_advance_timer()
            return

        self._advance_goal_index()
        self.attempts_remaining = self.retries_per_waypoint + 1
        self._send_current_goal()

    def _handle_path_failure(self, status) -> None:
        self.active_goal_handle = None
        self.attempts_remaining -= 1
        self.get_logger().warn(
            f"Continuous path failed with status {status}, "
            f"remaining retries: {self.attempts_remaining}"
        )
        if self.attempts_remaining > 0:
            self._schedule_continuous_path_retry()
            return

        if self.stop_on_failure:
            self.get_logger().error("Stopping continuous path navigation")
            return

        self.get_logger().info(
            "Continuous path exhausted retries; continuing from the current pose "
            "while keeping the remaining key-waypoint order fixed"
        )
        self.attempts_remaining = self.retries_per_waypoint + 1
        self._schedule_continuous_path_retry()

    def _advance_check(self) -> None:
        """Periodically check: if robot is close to current target,
        cancel the goal and advance to next target."""
        if not self.proximity_advance_enabled:
            return
        goal_wp = self._current_goal()
        if goal_wp is None:
            return
        if self.active_goal_handle is None:
            return

        dist = self._robot_distance_to(goal_wp)
        if dist < self.advance_distance:
            if self._is_at_key_wp():
                self._advance_goal_index()
                self.attempts_remaining = self.retries_per_waypoint + 1
                self.get_logger().info(
                    f"Proximity advance: reached key waypoint, "
                    f"moving to segment {self.seg_idx + 1}"
                )
            else:
                self._advance_goal_index()
                self.get_logger().info(
                    f"Proximity advance: interp {self.interp_idx} "
                    f"in segment {self.seg_idx + 1}"
                )

            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
            self._send_current_goal()

    def _stop_advance_timer(self) -> None:
        if self.advance_timer is not None:
            self.destroy_timer(self.advance_timer)
            self.advance_timer = None

    def run_once(self) -> None:
        if self.ran:
            return

        readiness_wait_reason = self._readiness_wait_reason()
        if readiness_wait_reason is not None:
            now_sec = time.monotonic()
            if (
                readiness_wait_reason != self._last_readiness_reason
                or now_sec - self._last_wait_log_time >= 5.0
            ):
                self.get_logger().info(
                    f"Navigation readiness gate: {readiness_wait_reason}"
                )
                self._last_readiness_reason = readiness_wait_reason
                self._last_wait_log_time = now_sec
            return

        if self.readiness_gate_enabled:
            self.get_logger().info(
                "Navigation readiness gate passed: localization is stable; "
                "planner, controller and global costmap are active"
            )
        self.ran = True
        self.destroy_timer(self.timer)

        raw_waypoints = load_waypoints(self.waypoint_file)
        if self.waypoint_transform_enabled:
            raw_waypoints = [
                transform_waypoint(
                    wp, self.source_origin_x, self.source_origin_y, self.source_origin_yaw
                )
                for wp in raw_waypoints
            ]

        # Prefer the latest localization pose so first-segment interpolation
        # starts from the robot's real map position instead of a default origin.
        if self.pose_received:
            start_pos = {
                "x": self.robot_x,
                "y": self.robot_y,
                "yaw": self.robot_yaw,
            }
            start_pos_source = "/amcl_pose"
        else:
            start_pos = {
                "x": self.initial_pose_x,
                "y": self.initial_pose_y,
                "yaw": self.initial_pose_yaw,
            }
            start_pos_source = "launch initial_pose"

        self.key_waypoints = raw_waypoints
        self.attempts_remaining = self.retries_per_waypoint + 1

        if self.navigation_mode == "follow_path":
            self.path_points = build_path_points(raw_waypoints, start_pos, step=self.path_step)
            self._publish_key_waypoints(raw_waypoints)
            path_msg = self._make_path(self.path_points)
            self._publish_path_visuals(path_msg)
            self.get_logger().info(
                f"Start position ({start_pos_source}): ({start_pos['x']:.2f}, {start_pos['y']:.2f}), "
                f"{len(raw_waypoints)} key waypoints, {len(self.path_points)} path samples "
                f"(step={self.path_step:.2f}m)"
            )
            self.get_logger().info(f"Waiting for action server {self.path_action_name}")
            self.path_client.wait_for_server()
            self._send_continuous_path(raw_waypoints)
            return

        if self.navigation_mode == "navigate_through_poses":
            self._publish_key_waypoints(raw_waypoints)
            self.get_logger().info(
                f"Start position ({start_pos_source}): "
                f"({start_pos['x']:.2f}, {start_pos['y']:.2f}), "
                f"{len(raw_waypoints)} key waypoints"
            )
            self.get_logger().info(
                f"Waiting for action server {self.through_poses_action_name}"
            )
            self.through_poses_client.wait_for_server()
            self._send_through_poses()
            return

        if self.navigation_mode == "plan_then_follow_path":
            self.preplan_waypoints = list(raw_waypoints)
            self._publish_key_waypoints(raw_waypoints)
            self.get_logger().info(
                f"Start position ({start_pos_source}): "
                f"({start_pos['x']:.2f}, {start_pos['y']:.2f}), "
                f"{len(raw_waypoints)} mandatory key waypoints"
            )
            self.get_logger().info(
                f"Waiting for planner action server {self.planner_action_name}"
            )
            self.planner_client.wait_for_server()
            self.get_logger().info(
                f"Waiting for controller action server {self.path_action_name}"
            )
            self.path_client.wait_for_server()
            self._request_preplanned_path()
            return

        self.segments = build_segments(raw_waypoints, start_pos, step=2.0)
        self._publish_sequence()
        self.seg_idx = 0
        self.interp_idx = 0

        total_interp = sum(len(s["interp"]) for s in self.segments)
        self.get_logger().info(
            f"Start position ({start_pos_source}): ({start_pos['x']:.2f}, {start_pos['y']:.2f}), "
            f"{len(self.segments)} key waypoints, "
            f"{total_interp} interp points (step=2.0m)"
        )
        first_goal = self._current_goal()
        if first_goal:
            self.get_logger().info(
                f"First goal: ({first_goal['x']:.2f}, {first_goal['y']:.2f})"
            )

        self.get_logger().info(f"Waiting for action server {self.action_name}")
        self.pose_client.wait_for_server()
        self._send_current_goal()

        if self.proximity_advance_enabled:
            self.get_logger().info(
                f"Proximity advance enabled (distance={self.advance_distance:.2f} m)"
            )
            self.advance_timer = self.create_timer(0.3, self._advance_check)
        else:
            self.get_logger().info(
                "Proximity advance disabled; waiting for NavigateToPose success before "
                "sending the next goal"
            )


def main() -> None:
    rclpy.init()
    node = SnakeWaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
