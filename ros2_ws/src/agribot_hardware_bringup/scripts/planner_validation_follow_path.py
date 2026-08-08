#!/usr/bin/env python3

import rclpy
from nav2_msgs.action import FollowPath, Wait
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PlannerValidationFollowPath(Node):
    def __init__(self):
        super().__init__("planner_validation_follow_path")

        action_name = self.declare_parameter("action_name", "/follow_path").value
        path_topic = self.declare_parameter(
            "path_topic", "/planning_test/path"
        ).value

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, path_topic, transient_qos)
        self.action_server = ActionServer(
            self,
            FollowPath,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.wait_action_server = ActionServer(
            self,
            Wait,
            "/wait",
            execute_callback=self.execute_wait_callback,
            cancel_callback=self.cancel_callback,
        )
        self.clear_local_costmap_service = self.create_service(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            self.clear_local_costmap_callback,
        )
        self.get_logger().info(
            "Dry-run FollowPath server is ready; no velocity command can be published"
        )

    def goal_callback(self, request):
        if not request.path.poses:
            self.get_logger().error("Rejecting an empty FollowPath request")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        path = goal_handle.request.path
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path)
        self.get_logger().info(
            "Native Nav Through Poses produced one continuous FollowPath goal "
            f"containing {len(path.poses)} poses; dry-run completed"
        )
        goal_handle.succeed()
        return FollowPath.Result()

    async def execute_wait_callback(self, goal_handle):
        self.get_logger().warning(
            "The validation behavior tree requested recovery Wait; "
            "the dry-run server completes it immediately"
        )
        goal_handle.succeed()
        return Wait.Result()

    def clear_local_costmap_callback(self, _request, response):
        self.get_logger().warning(
            "The validation behavior tree requested a local costmap clear; "
            "the dry-run service has no local costmap to clear"
        )
        return response

    def destroy_node(self):
        self.action_server.destroy()
        self.wait_action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerValidationFollowPath()
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
