#!/usr/bin/env python3

import copy

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def normalized_frame(frame_id):
    return frame_id.lstrip("/")


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
        self.planner_id = self.declare_parameter(
            "planner_id", "GridBased"
        ).value

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
        if self.direct_planning_enabled:
            self.create_subscription(
                PoseStamped, goal_topic, self.goal_callback, 10
            )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.planner_client = None
        if self.direct_planning_enabled:
            self.planner_client = ActionClient(
                self, ComputePathThroughPoses, planner_action
            )
        self.initial_pose = PoseStamped() if publish_default_transform else None
        if self.initial_pose is not None:
            self.initial_pose.header.frame_id = self.map_frame
            self.initial_pose.pose.orientation.w = 1.0
            self.publish_pose(self.initial_pose_publisher, self.initial_pose)
        self.waypoints = []
        self.goal_generation = 0
        self.dispatched_generation = 0
        self.request_in_flight = False
        self.create_timer(0.10, self.timer_callback)

        if self.direct_planning_enabled:
            self.get_logger().info(
                "Pure planning validation is ready; set 2D Pose Estimate, then "
                "add one or more 2D Goal Poses"
            )
        else:
            self.get_logger().info(
                "Native Nav Through Poses validation is ready; set the start "
                "with 2D Pose Estimate before submitting accumulated poses"
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
        if (
            self.direct_planning_enabled
            and self.waypoints
            and not self.request_in_flight
            and self.goal_generation > self.dispatched_generation
            and self.planner_client.server_is_ready()
        ):
            self.dispatch_goal()

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
        self.get_logger().info(
            f"Requested Smac path through {len(request.goals)} waypoint(s)"
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
        self.get_logger().info(
            f"Smac path through {len(self.waypoints)} waypoint(s) ready: "
            f"{len(path.poses)} poses, "
            f"planning time {wrapped_result.result.planning_time.sec}."
            f"{wrapped_result.result.planning_time.nanosec:09d} s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerValidationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
