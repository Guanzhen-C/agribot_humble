#!/usr/bin/env python3

import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String


class DifferentialSimValidator(Node):
    def __init__(self):
        super().__init__("differential_sim_validator")
        self.goal_x = float(self.declare_parameter("goal_x", 35.0).value)
        self.goal_y = float(self.declare_parameter("goal_y", 35.5).value)
        self.goal_yaw = float(self.declare_parameter("goal_yaw", 0.0).value)
        self.startup_timeout_sec = float(
            self.declare_parameter("startup_timeout_sec", 120.0).value
        )
        self.navigation_timeout_sec = float(
            self.declare_parameter("navigation_timeout_sec", 240.0).value
        )
        self.report_file = str(
            self.declare_parameter(
                "report_file", "/tmp/differential_fastlivo_rtk_sim_report.json"
            ).value
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        report_qos = QoSProfile(depth=1)
        report_qos.reliability = ReliabilityPolicy.RELIABLE
        report_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.report_publisher = self.create_publisher(
            String, "/simulation/validation_report", report_qos
        )
        self.create_subscription(
            Bool, "/fastlivo_rtk/ready", self.handle_ready, state_qos
        )
        self.create_subscription(
            Bool, "/fastlivo_rtk/fixed_active", self.handle_fixed, state_qos
        )
        self.create_subscription(
            Odometry,
            "/simulation/ground_truth",
            self.handle_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/fastlivo_rtk/odometry", self.handle_fused_odom, 20
        )
        self.create_subscription(NavPath, "/plan", self.handle_plan, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self.handle_command, 10)
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("controller_server", "planner_server", "bt_navigator")
        }
        self.lifecycle_futures = {name: None for name in self.lifecycle_clients}
        self.lifecycle_states = {name: None for name in self.lifecycle_clients}

        self.ready = False
        self.fixed_active = False
        self.latest_ground_truth = None
        self.latest_fused = None
        self.start_ground_truth = None
        self.goal_sent = False
        self.goal_handle = None
        self.completed = False
        self.started_wall_time = time.monotonic()
        self.goal_wall_time = None
        self.plan_pose_count = 0
        self.plan_length_m = 0.0
        self.nonzero_command_count = 0
        self.max_linear_speed = 0.0
        self.max_angular_speed = 0.0
        self.localization_squared_errors = []
        self.max_ground_truth_displacement = 0.0
        self.timer = self.create_timer(0.25, self.tick)
        self.get_logger().info(
            f"Waiting for FAST-LIVO2+RTK, then navigating to "
            f"({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_yaw:.2f})"
        )

    def handle_ready(self, message):
        self.ready = bool(message.data)

    def handle_fixed(self, message):
        self.fixed_active = bool(message.data)

    def handle_ground_truth(self, message):
        self.latest_ground_truth = message
        if self.goal_sent and self.start_ground_truth is not None:
            dx = message.pose.pose.position.x - self.start_ground_truth[0]
            dy = message.pose.pose.position.y - self.start_ground_truth[1]
            self.max_ground_truth_displacement = max(
                self.max_ground_truth_displacement, math.hypot(dx, dy)
            )

    def handle_fused_odom(self, message):
        self.latest_fused = message
        if not self.goal_sent or self.latest_ground_truth is None:
            return
        fused_stamp = self.stamp_sec(message)
        truth_stamp = self.stamp_sec(self.latest_ground_truth)
        if abs(fused_stamp - truth_stamp) > 0.2:
            return
        dx = message.pose.pose.position.x - self.latest_ground_truth.pose.pose.position.x
        dy = message.pose.pose.position.y - self.latest_ground_truth.pose.pose.position.y
        self.localization_squared_errors.append(dx * dx + dy * dy)

    def handle_plan(self, message):
        self.plan_pose_count = max(self.plan_pose_count, len(message.poses))
        length = 0.0
        for previous, current in zip(message.poses, message.poses[1:]):
            length += math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
        self.plan_length_m = max(self.plan_length_m, length)

    def handle_command(self, message):
        linear = abs(message.linear.x)
        angular = abs(message.angular.z)
        self.max_linear_speed = max(self.max_linear_speed, linear)
        self.max_angular_speed = max(self.max_angular_speed, angular)
        if linear > 1.0e-3 or angular > 1.0e-3:
            self.nonzero_command_count += 1

    @staticmethod
    def stamp_sec(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9

    def tick(self):
        if self.completed:
            return
        elapsed = time.monotonic() - self.started_wall_time
        if not self.goal_sent:
            if elapsed > self.startup_timeout_sec:
                self.finish(False, "startup timeout before localization/navigation became ready")
                return
            if not (
                self.ready
                and self.fixed_active
                and self.latest_ground_truth is not None
                and self.latest_fused is not None
                and self.navigation_is_active()
                and self.action_client.server_is_ready()
            ):
                return
            self.send_goal()
            return
        if (
            self.goal_wall_time is not None
            and time.monotonic() - self.goal_wall_time > self.navigation_timeout_sec
        ):
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.finish(False, "navigation timeout")

    def navigation_is_active(self):
        for name, client in self.lifecycle_clients.items():
            future = self.lifecycle_futures[name]
            if future is not None and future.done():
                try:
                    self.lifecycle_states[name] = future.result().current_state.id
                except Exception as error:
                    self.get_logger().warning(
                        f"Failed to query {name} lifecycle state: {error}"
                    )
                    self.lifecycle_states[name] = None
                self.lifecycle_futures[name] = None
            if self.lifecycle_futures[name] is None and client.service_is_ready():
                self.lifecycle_futures[name] = client.call_async(GetState.Request())
        return all(
            state == State.PRIMARY_STATE_ACTIVE
            for state in self.lifecycle_states.values()
        )

    def send_goal(self):
        pose = NavigateToPose.Goal()
        pose.pose.header.frame_id = "map"
        pose.pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.goal_x
        pose.pose.pose.position.y = self.goal_y
        pose.pose.pose.orientation.z = math.sin(self.goal_yaw * 0.5)
        pose.pose.pose.orientation.w = math.cos(self.goal_yaw * 0.5)
        self.start_ground_truth = (
            self.latest_ground_truth.pose.pose.position.x,
            self.latest_ground_truth.pose.pose.position.y,
        )
        self.goal_sent = True
        self.goal_wall_time = time.monotonic()
        future = self.action_client.send_goal_async(pose)
        future.add_done_callback(self.handle_goal_response)
        self.get_logger().info("Validation goal sent through Nav2 NavigateToPose")

    def handle_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.finish(False, f"goal request failed: {error}")
            return
        if not self.goal_handle.accepted:
            self.finish(False, "Nav2 rejected validation goal")
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_result)

    def handle_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.finish(False, f"navigation result failed: {error}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.finish(False, f"Nav2 finished with action status {status}")
            return
        self.finish(True, "Nav2 reported success")

    def finish(self, nav_success, reason):
        if self.completed:
            return
        self.completed = True
        goal_error = None
        if self.latest_ground_truth is not None:
            goal_error = math.hypot(
                self.latest_ground_truth.pose.pose.position.x - self.goal_x,
                self.latest_ground_truth.pose.pose.position.y - self.goal_y,
            )
        rmse = None
        if self.localization_squared_errors:
            rmse = math.sqrt(
                sum(self.localization_squared_errors)
                / len(self.localization_squared_errors)
            )
        checks = {
            "nav_action_succeeded": nav_success,
            "fastlivo_rtk_ready": self.ready,
            "fixed_rtk_active": self.fixed_active,
            "state_lattice_plan_generated": self.plan_pose_count >= 10
            and self.plan_length_m >= 1.0,
            "mppi_published_motion": self.nonzero_command_count >= 10
            and self.max_linear_speed > 0.05,
            "differential_turn_command_seen": self.max_angular_speed > 0.05,
            "vehicle_moved": self.max_ground_truth_displacement > 1.0,
            "goal_error_within_0_5m": goal_error is not None and goal_error <= 0.5,
            "localization_rmse_within_0_5m": rmse is not None and rmse <= 0.5,
        }
        passed = all(checks.values())
        report = {
            "passed": passed,
            "reason": reason,
            "checks": checks,
            "metrics": {
                "plan_pose_count": self.plan_pose_count,
                "plan_length_m": self.plan_length_m,
                "nonzero_command_count": self.nonzero_command_count,
                "max_linear_speed_mps": self.max_linear_speed,
                "max_angular_speed_radps": self.max_angular_speed,
                "max_ground_truth_displacement_m": self.max_ground_truth_displacement,
                "goal_error_m": goal_error,
                "localization_horizontal_rmse_m": rmse,
                "localization_sample_count": len(self.localization_squared_errors),
                "wall_time_sec": time.monotonic() - self.started_wall_time,
            },
        }
        message = String()
        message.data = json.dumps(report, ensure_ascii=True, sort_keys=True)
        self.report_publisher.publish(message)
        try:
            Path(self.report_file).write_text(
                json.dumps(report, indent=2, ensure_ascii=True) + "\n",
                encoding="ascii",
            )
        except OSError as error:
            self.get_logger().error(f"Failed to write validation report: {error}")
        log = self.get_logger().info if passed else self.get_logger().error
        log(("PASS" if passed else "FAIL") + ": " + message.data)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialSimValidator()
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
