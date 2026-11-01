#!/usr/bin/env python3
"""Isolated ROS transport test using synthetic odometry, never vehicle hardware."""

import argparse
import json
import math
import os
from pathlib import Path
import time
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger

from agribot_localization_switch.node import LocalizationSwitch


class DriverlessEstimator:
    def __init__(self, *args):
        self.running = False
        self.starts = self.stops = 0

    def start(self):
        self.running = True
        self.starts += 1

    def stop(self):
        self.running = False
        self.stops += 1

    def alive(self):
        return self.running


class Inputs(Node):
    def __init__(self, duration):
        super().__init__("synthetic_indoor_handover_inputs")
        self.duration = duration
        self.started = time.monotonic()
        self.takeover_at = None
        self.mode = ""
        self.fault = ""
        self.output, self.flags, self.statuses = [], [], []
        self.sent_fused = 0
        self.fused_at = -math.inf
        self.publishers_by_topic = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for cls, topic in (
            (Odometry, "/localization/navsat/odometry"),
            (Odometry, "/localization/fastlivo/odometry"),
            (UInt8, "/rtk/fix_quality"), (NavSatFix, "/rtk/fix"),
            (PoseWithCovarianceStamped, "/rtk/heading_with_covariance"),
            (Bool, "/localization/fastlivo/ready"),
        ):
            qos = latched if topic == "/localization/fastlivo/ready" else 10
            self.publishers_by_topic[topic] = self.create_publisher(cls, topic, qos)
        self.create_subscription(Odometry, "/fastlivo_rtk/odometry", self.odom, 20)
        self.create_subscription(Bool, "/fastlivo_rtk/ready", self.flag, latched)
        self.create_subscription(String, "/localization/active_method", self.method, latched)
        self.create_subscription(String, "/localization/status", lambda m: self.statuses.append(m.data), latched)
        self.client = self.create_client(Trigger, "/localization/switch_to_fastlivo")
        self.request = None
        self.create_timer(.05, self.emit)

    def odom(self, message):
        self.output.append((time.monotonic(), message))

    def flag(self, message):
        self.flags.append((time.monotonic(), message.data))

    def method(self, message):
        self.mode = message.data
        if self.mode == "fastlivo_rtk" and self.takeover_at is None:
            self.takeover_at = time.monotonic()

    def publish(self, topic, message):
        self.publishers_by_topic[topic].publish(message)

    def emit(self):
        now = time.monotonic()
        elapsed = now-self.started
        stamp = self.get_clock().now().to_msg()
        indoor = self.takeover_at is not None
        self.publish("/rtk/fix_quality", UInt8(data=0 if indoor else 4))
        if not indoor:
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.status.status = 2
            fix.latitude, fix.longitude, fix.altitude = 38., 115., 20.
            fix.position_covariance = [.001, 0., 0., 0., .001, 0., 0., 0., .002]
            self.publish("/rtk/fix", fix)
            heading = PoseWithCovarianceStamped()
            heading.header.stamp = stamp
            heading.pose.pose.orientation.w = 1.
            heading.pose.covariance[35] = .001
            self.publish("/rtk/heading_with_covariance", heading)
            self.publish("/localization/navsat/odometry", self.pose(stamp, elapsed))
        self.publish("/localization/fastlivo/ready", Bool(data=True))
        if self.fault != "dropout" and now-self.fused_at >= 1./3.:
            self.fused_at = now
            message = self.pose(stamp, elapsed, fused=True)
            if self.fault == "jump":
                message.pose.pose.position.x += 5.
            self.publish("/localization/fastlivo/odometry", message)
            self.sent_fused += 1
        if elapsed > 1. and self.request is None and self.client.service_is_ready():
            self.request = self.client.call_async(Trigger.Request())

    def pose(self, stamp, elapsed, fused=False):
        message = Odometry()
        message.header.frame_id, message.child_frame_id = "map", "base_link"
        message.header.stamp = stamp
        message.pose.pose.orientation.w = 1.
        message.pose.pose.position.x = .2*elapsed + (.1 if fused else 0.)
        message.twist.twist.linear.x = .2
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = .01
        if fused and self.takeover_at is not None:
            fraction = min(1., (time.monotonic()-self.takeover_at)/self.duration)
            message.pose.covariance[0] = .01+fraction*6.03
            message.pose.covariance[7] = .01+fraction*2.32
        return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=70., help="Seconds without RTK after takeover")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        parser.error("Requires ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID")
    if args.duration < 10 or not math.isfinite(args.duration):
        parser.error("duration must be finite and >= 10 seconds")
    rclpy.init(args=["--ros-args", "-p", "map_base:=/tmp/synthetic_handover_map", "-p", "wait_for_request:=false"])
    executor = SingleThreadedExecutor()
    manager = inputs = None
    report = {"input_kind": "synthetic ROS messages; no sensors, Nav2 or chassis", "passed": False}
    try:
        with patch("agribot_localization_switch.node.EstimatorProcess", DriverlessEstimator):
            manager = LocalizationSwitch()
        inputs = Inputs(args.duration)
        executor.add_node(manager); executor.add_node(inputs)

        def run_until(condition, timeout):
            deadline = time.monotonic()+timeout
            while not condition():
                if time.monotonic() > deadline:
                    raise AssertionError("Timed out waiting for ROS validation condition")
                executor.spin_once(timeout_sec=.05)

        run_until(lambda: inputs.takeover_at is not None, 15.)
        run_until(lambda: time.monotonic()-inputs.takeover_at >= args.duration, args.duration+2.)
        indoor_flags = [v for t, v in inputs.flags if t > inputs.takeover_at+.3]
        assert indoor_flags and all(indoor_flags), "Healthy indoor localization was revoked"
        received = [(t, m) for t, m in inputs.output if t > inputs.takeover_at+.3]
        assert len(received) > args.duration*2
        gaps = [b[0]-a[0] for a, b in zip(received, received[1:])]
        assert max(gaps) < 1., "Public localization stream was interrupted"
        assert all(m.twist.twist.linear.x == .2 for _, m in received)
        assert received[-1][1].pose.covariance[0] > 5.9
        assert manager.navsat.stops == 1 and manager.fastlivo.alive()
        report.update(no_rtk_duration_sec=args.duration, public_samples=len(received),
                      max_receipt_gap_sec=max(gaps), final_xy_variance=[
                          received[-1][1].pose.covariance[i] for i in (0, 7)])
        for fault, wait in (("jump", 2.), ("dropout", 6.)):
            inputs.fault = fault
            since = time.monotonic()
            run_until(lambda: any(t > since and not v for t, v in inputs.flags), wait)
            before = len(inputs.output)
            until = time.monotonic()+.4
            run_until(lambda: time.monotonic() >= until, 1.)
            assert len(inputs.output) == before, "Invalid localization was published"
            inputs.fault = ""
            since = time.monotonic()
            run_until(lambda: any(t > since and v for t, v in inputs.flags), 2.)
        report.update(passed=True, jump_rejected=True, dropout_rejected=True,
                      recovered_without_gnss=True, navsat_starts=manager.navsat.starts)
    finally:
        if manager:
            manager.close()
            executor.remove_node(manager); manager.destroy_node()
        if inputs:
            executor.remove_node(inputs); inputs.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
