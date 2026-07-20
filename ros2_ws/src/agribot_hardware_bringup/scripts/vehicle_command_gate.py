#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp_command(
    linear_x: float,
    angular_z: float,
    max_linear: float,
    max_angular: float,
) -> Tuple[float, float]:
    values = (linear_x, angular_z, max_linear, max_angular)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command and limits must be finite")
    if max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError("velocity limits must be positive")
    return (
        max(-max_linear, min(max_linear, linear_x)),
        max(-max_angular, min(max_angular, angular_z)),
    )


def zero_twist() -> Twist:
    return Twist()


class VehicleCommandGate(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "vehicle_command_gate", parameter_overrides=parameter_overrides
        )
        self.input_topic = self.declare_parameter(
            "input_topic", "/nav2/cmd_vel_safe"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/hardware/cmd_vel"
        ).value
        preflight_topic = self.declare_parameter(
            "preflight_topic", "/hardware/preflight_ready"
        ).value
        e_stop_topic = self.declare_parameter(
            "e_stop_topic", "/safety/e_stop"
        ).value
        hardware_e_stop_topic = self.declare_parameter(
            "hardware_e_stop_topic", "/hardware/chassis_e_stop"
        ).value
        drive_enable_topic = self.declare_parameter(
            "drive_enable_topic", "/safety/drive_enable"
        ).value

        self.input_timeout = float(
            self.declare_parameter("input_timeout_sec", 0.30).value
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 20.0).value
        )
        self.max_linear = float(
            self.declare_parameter("max_linear_velocity", 0.80).value
        )
        self.max_angular = float(
            self.declare_parameter("max_angular_velocity", 0.65).value
        )
        self.require_preflight = bool(
            self.declare_parameter("require_preflight", True).value
        )
        self.require_hardware_e_stop = bool(
            self.declare_parameter("require_hardware_e_stop", False).value
        )
        self.drive_enabled = bool(
            self.declare_parameter("initially_enabled", False).value
        )

        if self.input_timeout <= 0.0 or self.publish_rate <= 0.0:
            raise ValueError("input_timeout_sec and publish_rate must be positive")
        clamp_command(0.0, 0.0, self.max_linear, self.max_angular)

        self.preflight_ready = not self.require_preflight
        self.e_stop = False
        self.hardware_e_stop_received = not self.require_hardware_e_stop
        self.hardware_e_stop = self.require_hardware_e_stop
        self.latest_command = zero_twist()
        self.latest_command_time_ns = None
        self.last_active = None

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.active_publisher = self.create_publisher(
            Bool, "/hardware/command_output_active", state_qos
        )
        self.create_subscription(Twist, self.input_topic, self.handle_command, 20)
        self.create_subscription(Bool, preflight_topic, self.handle_preflight, state_qos)
        self.create_subscription(Bool, e_stop_topic, self.handle_e_stop, 10)
        self.create_subscription(
            Bool, hardware_e_stop_topic, self.handle_hardware_e_stop, 10
        )
        self.create_subscription(Bool, drive_enable_topic, self.handle_drive_enable, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_command)

        self.get_logger().info(
            "Vehicle command gate ready: input=%s output=%s enabled=%s"
            % (self.input_topic, self.output_topic, self.drive_enabled)
        )

    def handle_command(self, msg: Twist) -> None:
        try:
            linear_x, angular_z = clamp_command(
                msg.linear.x,
                msg.angular.z,
                self.max_linear,
                self.max_angular,
            )
        except ValueError as exception:
            self.get_logger().error(f"Rejected invalid velocity command: {exception}")
            self.latest_command_time_ns = None
            return

        command = zero_twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.latest_command = command
        self.latest_command_time_ns = self.get_clock().now().nanoseconds

    def handle_preflight(self, msg: Bool) -> None:
        self.preflight_ready = bool(msg.data)

    def handle_e_stop(self, msg: Bool) -> None:
        self.e_stop = bool(msg.data)

    def handle_hardware_e_stop(self, msg: Bool) -> None:
        self.hardware_e_stop_received = True
        self.hardware_e_stop = bool(msg.data)

    def handle_drive_enable(self, msg: Bool) -> None:
        self.drive_enabled = bool(msg.data)

    def command_is_fresh(self) -> bool:
        if self.latest_command_time_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.latest_command_time_ns) / 1e9
        return 0.0 <= age <= self.input_timeout

    def output_is_active(self) -> bool:
        return (
            self.drive_enabled
            and self.preflight_ready
            and not self.e_stop
            and self.hardware_e_stop_received
            and not self.hardware_e_stop
            and self.command_is_fresh()
        )

    def publish_command(self) -> None:
        active = self.output_is_active()
        self.publisher.publish(self.latest_command if active else zero_twist())

        active_msg = Bool()
        active_msg.data = active
        self.active_publisher.publish(active_msg)

        if active != self.last_active:
            if active:
                self.get_logger().info("CAN command output enabled")
            else:
                reasons = []
                if not self.drive_enabled:
                    reasons.append("drive disabled")
                if not self.preflight_ready:
                    reasons.append("preflight not ready")
                if self.e_stop:
                    reasons.append("emergency stop")
                if not self.hardware_e_stop_received:
                    reasons.append("hardware emergency-stop state missing")
                elif self.hardware_e_stop:
                    reasons.append("hardware emergency stop")
                if not self.command_is_fresh():
                    reasons.append("command missing or stale")
                self.get_logger().warn("CAN command output stopped: " + ", ".join(reasons))
            self.last_active = active

    def destroy_node(self):
        if rclpy.ok():
            stop = zero_twist()
            try:
                for _ in range(3):
                    self.publisher.publish(stop)
            except Exception as exception:
                self.get_logger().warn(f"Could not publish shutdown stop: {exception}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = VehicleCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
