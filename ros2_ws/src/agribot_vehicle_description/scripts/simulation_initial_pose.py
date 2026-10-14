#!/usr/bin/env python3
"""Supply the known spawn pose once, never ground-truth motion feedback."""

import copy
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class InitialPoseBuffer:
    def __init__(self):
        self.truth = deque(maxlen=100)
        self.done = False

    def seed(self, odometry):
        if self.done or not self.truth or stamp_seconds(odometry) < 1.0:
            return None
        truth = min(self.truth, key=lambda msg: abs(stamp_seconds(msg) - stamp_seconds(odometry)))
        if abs(stamp_seconds(truth) - stamp_seconds(odometry)) > 0.05:
            return None
        result = PoseWithCovarianceStamped()
        result.header = copy.deepcopy(odometry.header)
        result.header.frame_id = "map"
        result.pose.pose = copy.deepcopy(truth.pose.pose)
        self.done = True
        self.truth.clear()
        return result


class SimulationInitialPose(Node):
    def __init__(self):
        super().__init__("simulation_initial_pose")
        self.buffer = InitialPoseBuffer()
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/configured/initial_localization_pose", 1)
        self.truth_subscription = self.create_subscription(
            Odometry, "/odom", self.on_truth, qos_profile_sensor_data)
        self.local_subscription = self.create_subscription(
            Odometry, "/fastlivo/odometry", self.on_local, qos_profile_sensor_data)

    def on_truth(self, message):
        if not self.buffer.done:
            self.buffer.truth.append(message)

    def on_local(self, message):
        if self.publisher.get_subscription_count() == 0:
            return
        seed = self.buffer.seed(message)
        if seed is not None:
            self.publisher.publish(seed)
            self.destroy_subscription(self.truth_subscription)
            self.destroy_subscription(self.local_subscription)
            self.get_logger().info("Published synchronized spawn pose once; truth subscriptions removed")


def main():
    rclpy.init()
    node = SimulationInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
