#!/usr/bin/env python3

from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SimulatedJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("simulated_joint_state_publisher")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.16459).value)
        self.track_width = float(self.declare_parameter("track_width", 0.58306).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 50.0).value)
        self.joint_names: List[str] = list(
            self.declare_parameter(
                "joint_names",
                ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"],
            ).value
        )

        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.last_stamp_ns: Optional[int] = None

        self.publisher = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, 20)
        self.create_timer(1.0 / max(self.publish_hz, 1e-3), self.publish_joint_state)

    def handle_odom(self, msg: Odometry) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        linear = float(msg.twist.twist.linear.x)
        angular = float(msg.twist.twist.angular.z)

        left_linear = linear - angular * self.track_width * 0.5
        right_linear = linear + angular * self.track_width * 0.5
        left_wheel_velocity = left_linear / max(self.wheel_radius, 1e-6)
        right_wheel_velocity = right_linear / max(self.wheel_radius, 1e-6)

        if self.last_stamp_ns is not None and stamp_ns > self.last_stamp_ns:
            dt = (stamp_ns - self.last_stamp_ns) / 1_000_000_000.0
            self.joint_positions[0] += left_wheel_velocity * dt
            self.joint_positions[1] += right_wheel_velocity * dt
            self.joint_positions[2] += left_wheel_velocity * dt
            self.joint_positions[3] += right_wheel_velocity * dt

        self.joint_velocities[0] = left_wheel_velocity
        self.joint_velocities[1] = right_wheel_velocity
        self.joint_velocities[2] = left_wheel_velocity
        self.joint_velocities[3] = right_wheel_velocity
        self.last_stamp_ns = stamp_ns

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.joint_positions)
        msg.velocity = list(self.joint_velocities)
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimulatedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
