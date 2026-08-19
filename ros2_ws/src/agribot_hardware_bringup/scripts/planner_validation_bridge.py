#!/usr/bin/env python3

import copy
import hashlib
import json
import math
from pathlib import Path as FilePath

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def normalized_frame(frame_id):
    return frame_id.lstrip("/")


def finite_number(value, description):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{description} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise RuntimeError(f"{description} must be a finite number")
    return number


def strict_json_document(path):
    def reject_constant(constant):
        raise ValueError(f"non-finite JSON number: {constant}")

    def reject_duplicate_keys(pairs):
        document = {}
        for key, value in pairs:
            if key in document:
                raise ValueError(f"duplicate JSON key: {key}")
            document[key] = value
        return document

    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(f"semantic route is not strict JSON: {error}") from error


def validated_position(document, description):
    if not isinstance(document, dict):
        raise RuntimeError(f"{description} must be an object")
    return {
        "x": finite_number(document.get("x"), f"{description} x"),
        "y": finite_number(document.get("y"), f"{description} y"),
        "z": finite_number(document.get("z", 0.0), f"{description} z"),
    }


def validated_route_pose(route_pose, route_index):
    if not isinstance(route_pose, dict):
        raise RuntimeError(f"semantic route pose {route_index} must be an object")
    declared_index = route_pose.get("index", route_index)
    if (
        isinstance(declared_index, bool)
        or not isinstance(declared_index, int)
        or declared_index != route_index
    ):
        raise RuntimeError(
            f"semantic route pose {route_index} has an invalid route index"
        )
    place_id = route_pose.get("place_id")
    if not isinstance(place_id, str) or not place_id:
        raise RuntimeError(f"semantic route pose {route_index} has no place id")
    return {
        "selector": place_id,
        "place_id": place_id,
        "route_index": route_index,
        "position": validated_position(
            route_pose.get("position"), f"semantic route pose {route_index}"
        ),
        "yaw": finite_number(
            route_pose.get("yaw"), f"semantic route pose {route_index} yaw"
        ),
    }


def semantic_stop_pose(stop, route_poses, stop_index):
    if not isinstance(stop, dict):
        raise RuntimeError(f"semantic route stop {stop_index} must be an object")
    selector = stop.get("selector")
    if not isinstance(selector, str) or not selector:
        raise RuntimeError(f"semantic route stop {stop_index} has no selector")
    route_index = stop.get("navigation_route_index")
    if (
        isinstance(route_index, bool)
        or not isinstance(route_index, int)
        or not 0 <= route_index < len(route_poses)
    ):
        raise RuntimeError(
            f"semantic route stop {selector} has an invalid navigation route index"
        )
    route_pose = route_poses[route_index]
    anchor = stop.get("navigation_anchor_position", stop.get("position"))
    anchor_position = validated_position(anchor, f"semantic stop {selector} anchor")
    route_position = route_pose["position"]
    planar_error = math.hypot(
        route_position["x"] - anchor_position["x"],
        route_position["y"] - anchor_position["y"],
    )
    if planar_error > 0.05:
        raise RuntimeError(
            f"semantic route stop {selector} is {planar_error:.3f} m from its route pose"
        )
    expected_place = stop.get("navigation_anchor_place")
    if expected_place and route_pose["place_id"] != expected_place:
        raise RuntimeError(
            f"semantic route stop {selector} does not match its anchor place"
        )
    return {
        "selector": selector,
        "position": route_position,
        "yaw": route_pose["yaw"],
        "route_index": route_index,
    }


def load_semantic_route_plan(route_file, map_frame):
    route_path = FilePath(route_file).expanduser().resolve()
    if not route_path.is_file():
        raise RuntimeError(f"semantic route file does not exist: {route_path}")
    document = strict_json_document(route_path)
    if not isinstance(document, dict) or document.get("schema_version") != 3:
        raise RuntimeError("semantic route must use schema version 3")
    route_frame = normalized_frame(str(document.get("frame_id", "")))
    if route_frame != map_frame:
        raise RuntimeError(
            f"semantic route frame '{route_frame}' does not match '{map_frame}'"
        )
    policy = document.get("execution_policy")
    if (
        not isinstance(policy, dict)
        or policy.get("preview_only") is not True
        or policy.get("execution_authorized") is not False
        or policy.get("requires_nav2_path_planning") is not True
    ):
        raise RuntimeError("semantic route does not satisfy the preview-only policy")

    route = document.get("route")
    raw_route_poses = route.get("poses") if isinstance(route, dict) else None
    stops = document.get("resolved_stops")
    if not isinstance(raw_route_poses, list) or not raw_route_poses:
        raise RuntimeError("semantic route contains no route poses")
    if not isinstance(stops, list) or len(stops) < 2:
        raise RuntimeError("semantic route requires a start and at least one destination")

    request = document.get("request")
    if not isinstance(request, dict):
        raise RuntimeError("semantic route has no request contract")
    via = request.get("via", [])
    if not isinstance(via, list) or any(not isinstance(item, str) for item in via):
        raise RuntimeError("semantic route request has invalid intermediate stops")
    requested_selectors = [request.get("start")] + via + [request.get("goal")]
    resolved_selectors = [
        stop.get("selector") if isinstance(stop, dict) else None for stop in stops
    ]
    if requested_selectors != resolved_selectors:
        raise RuntimeError("semantic route resolved stops do not match request order")

    route_poses = [
        validated_route_pose(route_pose, route_index)
        for route_index, route_pose in enumerate(raw_route_poses)
    ]
    stop_poses = [
        semantic_stop_pose(stop, route_poses, index)
        for index, stop in enumerate(stops)
    ]
    stop_route_indices = [stop["route_index"] for stop in stop_poses]
    if stop_route_indices != sorted(set(stop_route_indices)):
        raise RuntimeError("semantic route stops are not strictly ordered")
    if stop_route_indices[0] != 0 or stop_route_indices[-1] != len(route_poses) - 1:
        raise RuntimeError(
            "semantic route must begin at its requested start and end at its goal"
        )
    avoidance = document.get("avoidance_constraints", {})
    if not isinstance(avoidance, dict):
        raise RuntimeError("semantic route avoidance constraints must be an object")
    influence_radius = finite_number(
        avoidance.get("influence_radius_m", 0.0), "semantic avoidance influence radius"
    )
    decay_length = finite_number(
        avoidance.get("decay_length_m", 0.0), "semantic avoidance decay length"
    )
    if influence_radius <= 0.0 or decay_length <= 0.0:
        raise RuntimeError("semantic avoidance influence radius and decay length must be positive")
    avoidance_nodes = avoidance.get("nodes", [])
    if not isinstance(avoidance_nodes, list):
        raise RuntimeError("semantic avoidance nodes must be a list")
    avoidance_zones = []
    for index, node in enumerate(avoidance_nodes):
        if not isinstance(node, dict):
            raise RuntimeError(f"semantic avoidance node {index} must be an object")
        selector = node.get("selector")
        if not isinstance(selector, str) or not selector:
            raise RuntimeError(f"semantic avoidance node {index} has no selector")
        avoidance_zones.append(
            {
                "selector": selector,
                "position": validated_position(
                    node.get("position"), f"semantic avoidance node {selector}"
                ),
                "influence_radius_m": influence_radius,
                "decay_length_m": decay_length,
            }
        )
    if bool(avoidance_zones) != bool(
        policy.get("requires_nav2_proximity_layer", False)
    ):
        raise RuntimeError(
            "semantic avoidance zones do not match the proximity-cost policy"
        )
    return {
        "route_id": str(document.get("route_id", route_path.stem)),
        "route_path": route_path,
        "route_sha256": hashlib.sha256(route_path.read_bytes()).hexdigest(),
        "astar_poses": route_poses,
        "requested_stops": stop_poses,
        "avoidance_zones": avoidance_zones,
    }


class PlannerValidationBridge(Node):
    def __init__(self):
        super().__init__("planner_validation_bridge")

        self.direct_planning_enabled = self.declare_parameter(
            "direct_planning_enabled", True
        ).value
        publish_default_transform = self.declare_parameter(
            "publish_default_transform", False
        ).value
        self.map_frame = normalized_frame(
            self.declare_parameter("map_frame", "map").value
        )
        self.base_frame = normalized_frame(
            self.declare_parameter("base_frame", "base_link").value
        )
        initial_pose_topic = self.declare_parameter(
            "initial_pose_topic", "/initialpose"
        ).value
        goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        path_topic = self.declare_parameter(
            "path_topic", "/planning_test/path"
        ).value
        initial_pose_display_topic = self.declare_parameter(
            "initial_pose_display_topic", "/planning_test/initial_pose"
        ).value
        goal_display_topic = self.declare_parameter(
            "goal_display_topic", "/planning_test/goal_pose"
        ).value
        waypoints_display_topic = self.declare_parameter(
            "waypoints_display_topic", "/planning_test/waypoints"
        ).value
        planner_action = self.declare_parameter(
            "planner_action", "/compute_path_through_poses"
        ).value
        planner_state_service = self.declare_parameter(
            "planner_state_service", "/planner_server/get_state"
        ).value
        semantic_proximity_costmap_topic = self.declare_parameter(
            "semantic_proximity_costmap_topic", "/semantic_navigation/proximity_costmap"
        ).value
        global_costmap_topic = self.declare_parameter(
            "global_costmap_topic", "/global_costmap/costmap"
        ).value
        self.planner_id = self.declare_parameter(
            "planner_id", "GridBased"
        ).value
        self.route_waypoint_mode = str(
            self.declare_parameter(
                "route_waypoint_mode", "semantic_stops"
            ).value
        ).strip()
        if self.route_waypoint_mode not in (
            "semantic_stops",
            "all_astar",
            "requested_stops",
        ):
            raise RuntimeError(
                "route_waypoint_mode must be 'semantic_stops', 'all_astar' "
                "or 'requested_stops'"
            )
        path_output_file = str(
            self.declare_parameter("path_output_file", "").value
        ).strip()
        self.path_output_path = None
        if path_output_file:
            self.path_output_path = FilePath(path_output_file).expanduser().resolve()
            if self.path_output_path.suffix.lower() != ".json":
                raise RuntimeError("path_output_file must use the .json suffix")
        self.map_yaml = str(self.declare_parameter("map_yaml", "").value).strip()
        self.planner_params_file = str(
            self.declare_parameter("planner_params_file", "").value
        ).strip()
        route_plan = str(
            self.declare_parameter("route_plan", "").value
        ).strip()
        if self.route_waypoint_mode == "requested_stops" and not route_plan:
            raise RuntimeError(
                "requested_stops waypoint mode requires a semantic route plan"
            )
        if self.route_waypoint_mode == "requested_stops" and self.path_output_path is None:
            raise RuntimeError(
                "requested_stops waypoint mode is reserved for topology certification "
                "and requires path_output_file"
            )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_publisher = self.create_publisher(Path, path_topic, transient_qos)
        self.initial_pose_publisher = self.create_publisher(
            PoseStamped, initial_pose_display_topic, transient_qos
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, goal_display_topic, transient_qos
        )
        self.waypoints_publisher = self.create_publisher(
            PoseArray, waypoints_display_topic, transient_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            semantic_proximity_costmap_topic,
            self.semantic_proximity_costmap_callback,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            global_costmap_topic,
            self.global_costmap_callback,
            transient_qos,
        )
        if self.direct_planning_enabled:
            self.create_subscription(
                PoseStamped, goal_topic, self.goal_callback, 10
            )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.planner_client = None
        self.planner_state_client = None
        if self.direct_planning_enabled:
            self.planner_client = ActionClient(
                self, ComputePathThroughPoses, planner_action
            )
            self.planner_state_client = self.create_client(
                GetState, planner_state_service
            )
        self.planner_is_active = False
        self.planner_state_future = None
        self.semantic_route_id = ""
        self.semantic_route_path = None
        self.semantic_route_sha256 = ""
        self.requested_destination_count = 0
        self.avoidance_zones = []
        self.initial_pose = PoseStamped() if publish_default_transform else None
        self.waypoints = []
        self.goal_generation = 0
        self.dispatched_generation = 0
        self.request_in_flight = False
        self.semantic_proximity_costmap_ready = False
        self.semantic_proximity_applied = False

        if route_plan:
            if not self.direct_planning_enabled:
                raise RuntimeError(
                    "semantic route auto-planning requires direct_planning_enabled"
                )
            semantic_route = load_semantic_route_plan(route_plan, self.map_frame)
            self.semantic_route_id = semantic_route["route_id"]
            self.semantic_route_path = semantic_route["route_path"]
            self.semantic_route_sha256 = semantic_route["route_sha256"]
            if self.route_waypoint_mode in (
                "semantic_stops",
                "requested_stops",
            ):
                route_poses = semantic_route["requested_stops"]
            else:
                route_poses = semantic_route["astar_poses"]
            self.initial_pose = self.pose_from_semantic_stop(
                route_poses[0]
            )
            self.waypoints = [
                self.pose_from_semantic_stop(stop)
                for stop in route_poses[1:]
            ]
            self.requested_destination_count = (
                len(semantic_route["requested_stops"]) - 1
            )
            self.avoidance_zones = semantic_route["avoidance_zones"]
            self.goal_generation = 1

        if self.initial_pose is not None:
            self.initial_pose.header.frame_id = self.map_frame
            if not route_plan:
                self.initial_pose.pose.orientation.w = 1.0
            self.publish_pose(self.initial_pose_publisher, self.initial_pose)
        self.publish_waypoints()
        if self.waypoints:
            self.publish_pose(self.goal_publisher, self.waypoints[-1])

        self.create_timer(0.10, self.timer_callback)

        if self.semantic_route_id:
            self.get_logger().info(
                f"Loaded preview-only semantic route {self.semantic_route_id}: "
                f"{len(self.waypoints) + 1} {self.route_point_description()}, "
                f"{self.requested_destination_count} requested destination(s), "
                f"{len(self.avoidance_zones)} avoidance zone(s); "
                "Smac planning will start automatically"
            )
        elif self.direct_planning_enabled:
            self.get_logger().info(
                "Pure planning validation is ready; set 2D Pose Estimate, then "
                "add one or more 2D Goal Poses"
            )
        else:
            self.get_logger().info(
                "Native Nav Through Poses validation is ready; set the start "
                "with 2D Pose Estimate before submitting accumulated poses"
            )

    def route_point_description(self):
        if self.route_waypoint_mode == "requested_stops":
            return "ordered semantic certification anchor(s)"
        if self.route_waypoint_mode == "semantic_stops":
            return "ordered language-model destination(s)"
        return "ordered A* point(s)"

    def pose_from_semantic_stop(self, stop):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = stop["position"]["x"]
        pose.pose.position.y = stop["position"]["y"]
        pose.pose.position.z = stop["position"]["z"]
        pose.pose.orientation.z = math.sin(0.5 * stop["yaw"])
        pose.pose.orientation.w = math.cos(0.5 * stop["yaw"])
        return pose

    def semantic_proximity_costmap_callback(self, message):
        if not message.data or message.info.width == 0 or message.info.height == 0:
            return
        if not self.semantic_proximity_costmap_ready:
            self.semantic_proximity_costmap_ready = True
            self.get_logger().info(
                "Semantic proximity costmap is ready; waiting for the global costmap"
            )

    def global_costmap_callback(self, message):
        if not self.semantic_proximity_costmap_ready or self.semantic_proximity_applied:
            return
        if not message.data or message.info.width == 0 or message.info.height == 0:
            return
        origin = message.info.origin
        yaw = math.atan2(
            2.0
            * (
                origin.orientation.w * origin.orientation.z
                + origin.orientation.x * origin.orientation.y
            ),
            1.0
            - 2.0
            * (
                origin.orientation.y * origin.orientation.y
                + origin.orientation.z * origin.orientation.z
            ),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for zone in self.avoidance_zones:
            dx = zone["position"]["x"] - origin.position.x
            dy = zone["position"]["y"] - origin.position.y
            column = int(
                math.floor((cosine * dx + sine * dy) / message.info.resolution)
            )
            row = int(
                math.floor((-sine * dx + cosine * dy) / message.info.resolution)
            )
            if (
                not 0 <= column < message.info.width
                or not 0 <= row < message.info.height
                or (message.data[row * message.info.width + column] & 0xFF) < 150
            ):
                return
        self.semantic_proximity_applied = True
        self.get_logger().info(
            "Semantic route costs are present in the global costmap; Smac planning may start"
        )

    def initial_pose_callback(self, message):
        frame = normalized_frame(message.header.frame_id or self.map_frame)
        if frame != self.map_frame:
            self.get_logger().error(
                f"Ignoring initial pose in frame '{frame}'; expected '{self.map_frame}'"
            )
            return

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = self.map_frame
        self.initial_pose.pose = copy.deepcopy(message.pose.pose)
        self.semantic_route_id = ""
        self.requested_destination_count = 0
        self.avoidance_zones.clear()
        self.waypoints.clear()
        self.goal_generation += 1
        self.publish_pose(self.initial_pose_publisher, self.initial_pose)
        self.publish_waypoints()
        self.clear_path()
        self.get_logger().info(
            "Accepted planning start pose and cleared all waypoints at "
            f"({self.initial_pose.pose.position.x:.3f}, "
            f"{self.initial_pose.pose.position.y:.3f})"
        )

    def goal_callback(self, message):
        if self.initial_pose is None:
            self.get_logger().error(
                "Ignoring goal: set the start with RViz 2D Pose Estimate first"
            )
            return

        frame = normalized_frame(message.header.frame_id or self.map_frame)
        if frame != self.map_frame:
            self.get_logger().error(
                f"Ignoring goal in frame '{frame}'; expected '{self.map_frame}'"
            )
            return

        waypoint = copy.deepcopy(message)
        waypoint.header.frame_id = self.map_frame
        self.waypoints.append(waypoint)
        self.goal_generation += 1
        self.publish_pose(self.goal_publisher, waypoint)
        self.publish_waypoints()
        self.get_logger().info(
            f"Added waypoint {len(self.waypoints)} at "
            f"({waypoint.pose.position.x:.3f}, {waypoint.pose.position.y:.3f})"
        )
        if not self.planner_client.server_is_ready():
            self.get_logger().warning(
                "Planner action is not active yet; the goal will be sent when ready"
            )

    def timer_callback(self):
        if self.initial_pose is not None:
            self.publish_start_transform()
        if self.direct_planning_enabled and not self.planner_is_active:
            self.poll_planner_state()
        if (
            self.direct_planning_enabled
            and self.planner_is_active
            and self.waypoints
            and (not self.semantic_route_id or self.semantic_proximity_applied)
            and not self.request_in_flight
            and self.goal_generation > self.dispatched_generation
            and self.planner_client.server_is_ready()
        ):
            self.dispatch_goal()

    def poll_planner_state(self):
        if (
            self.planner_state_future is not None
            or not self.planner_state_client.service_is_ready()
        ):
            return
        self.planner_state_future = self.planner_state_client.call_async(
            GetState.Request()
        )
        self.planner_state_future.add_done_callback(
            self.planner_state_callback
        )

    def planner_state_callback(self, future):
        self.planner_state_future = None
        try:
            state_id = future.result().current_state.id
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().warning(f"Cannot read planner lifecycle state: {error}")
            return
        self.planner_is_active = state_id == State.PRIMARY_STATE_ACTIVE

    def publish_start_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.initial_pose.pose.position.x
        transform.transform.translation.y = self.initial_pose.pose.position.y
        transform.transform.translation.z = self.initial_pose.pose.position.z
        transform.transform.rotation = copy.deepcopy(self.initial_pose.pose.orientation)
        self.tf_broadcaster.sendTransform(transform)

    def publish_pose(self, publisher, pose):
        pose.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(pose)

    def publish_waypoints(self):
        message = PoseArray()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = [copy.deepcopy(waypoint.pose) for waypoint in self.waypoints]
        self.waypoints_publisher.publish(message)

    def clear_path(self):
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(message)

    def dispatch_goal(self):
        request = ComputePathThroughPoses.Goal()
        request.start = copy.deepcopy(self.initial_pose)
        request.goals = [copy.deepcopy(waypoint) for waypoint in self.waypoints]
        stamp = self.get_clock().now().to_msg()
        request.start.header.stamp = stamp
        for waypoint in request.goals:
            waypoint.header.stamp = stamp
        request.planner_id = self.planner_id
        request.use_start = True

        request_generation = self.goal_generation
        self.dispatched_generation = request_generation
        self.request_in_flight = True
        future = self.planner_client.send_goal_async(request)
        future.add_done_callback(
            lambda response: self.goal_response_callback(
                response, request_generation
            )
        )
        goal_kind = (
            self.route_point_description() if self.semantic_route_id else "waypoint(s)"
        )
        self.get_logger().info(
            f"Requested Smac path through {len(request.goals)} {goal_kind}"
        )

    def goal_response_callback(self, future, request_generation):
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.request_in_flight = False
            self.get_logger().error(f"Planner request failed: {error}")
            return

        if not goal_handle.accepted:
            self.request_in_flight = False
            self.get_logger().error("Planner rejected the path request")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.result_callback(result, request_generation)
        )

    def result_callback(self, future, request_generation):
        self.request_in_flight = False
        try:
            wrapped_result = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(f"Planner result failed: {error}")
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Smac planning failed with action status {wrapped_result.status}"
            )
            return

        if request_generation != self.goal_generation:
            self.get_logger().info(
                "Discarding an outdated path because the waypoint list changed"
            )
            return

        path = wrapped_result.result.path
        if not path.poses:
            self.get_logger().error("Smac returned an empty path")
            return

        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path)
        if self.avoidance_zones:
            self.get_logger().info(
                "Smac path uses the bounded semantic proximity costs"
            )
        if self.path_output_path is not None:
            self.write_path_output(path)
        goal_kind = (
            self.route_point_description() if self.semantic_route_id else "waypoint(s)"
        )
        self.get_logger().info(
            f"Smac path through {len(self.waypoints)} {goal_kind} ready: "
            f"{len(path.poses)} poses, "
            f"planning time {wrapped_result.result.planning_time.sec}."
            f"{wrapped_result.result.planning_time.nanosec:09d} s"
        )

    def write_path_output(self, path):
        poses = []
        path_length = 0.0
        previous = None
        for index, stamped_pose in enumerate(path.poses):
            position = stamped_pose.pose.position
            orientation = stamped_pose.pose.orientation
            values = (
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"Smac path pose {index} contains a non-finite value")
            yaw = math.atan2(
                2.0
                * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0
                - 2.0
                * (
                    orientation.y * orientation.y
                    + orientation.z * orientation.z
                ),
            )
            if previous is not None:
                path_length += math.hypot(
                    position.x - previous[0], position.y - previous[1]
                )
            previous = (position.x, position.y)
            poses.append(
                {
                    "index": index,
                    "position": {
                        "x": float(position.x),
                        "y": float(position.y),
                        "z": float(position.z),
                    },
                    "yaw": float(yaw),
                }
            )

        source = {
            "semantic_route": str(self.semantic_route_path),
            "semantic_route_sha256": self.semantic_route_sha256,
        }
        for key, configured_path in (
            ("map_yaml", self.map_yaml),
            ("planner_params", self.planner_params_file),
        ):
            if not configured_path:
                continue
            source_path = FilePath(configured_path).expanduser().resolve()
            if not source_path.is_file():
                raise RuntimeError(f"{key} source file does not exist: {source_path}")
            source[key] = str(source_path)
            source[f"{key}_sha256"] = hashlib.sha256(
                source_path.read_bytes()
            ).hexdigest()

        document = {
            "schema_version": 1,
            "frame_id": self.map_frame,
            "planner_id": str(self.planner_id),
            "route_id": self.semantic_route_id,
            "route_waypoint_mode": self.route_waypoint_mode,
            "source": source,
            "statistics": {
                "path_poses": len(poses),
                "path_length_m": path_length,
                "certification_anchors": len(self.waypoints) + 1,
            },
            "poses": poses,
        }
        self.path_output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path_output_path.with_suffix(
            self.path_output_path.suffix + ".tmp"
        )
        temporary.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path_output_path)
        self.get_logger().info(
            f"Saved planner-certified reference path to {self.path_output_path}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PlannerValidationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
