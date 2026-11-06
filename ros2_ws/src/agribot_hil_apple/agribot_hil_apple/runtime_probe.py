"""Observe the real HIL topics without publishing commands or localization."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Bool, String


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__('hil_runtime_probe')
        self.clock_first = None
        self.clock_latest = None
        self.topics = {}
        self.diagnostics = {}
        self.subscriptions_ = []
        for topic, message in (
            ('/clock', Clock), ('/camera/rgb/image_raw', Image),
            ('/hil/camera/decoded', Image),
            ('/imu/data', Imu), ('/configured/localization_points', PointCloud2),
            ('/fastlivo/odometry', Odometry), ('/fastlivo_rtk/odometry', Odometry),
            ('/hil/ground_truth', Odometry), ('/fastlivo_rtk/ready', Bool),
            ('/hil/can/status', String), ('/diagnostics', DiagnosticArray),
        ):
            self.subscriptions_.append(self.create_subscription(
                message, topic, lambda msg, name=topic: self.receive(name, msg),
                qos_profile_sensor_data))

    def receive(self, topic, message):
        wall = time.monotonic()
        entry = self.topics.setdefault(topic, {'count': 0, 'maximum_receipt_gap_sec': 0.})
        if 'last_receipt' in entry:
            entry['maximum_receipt_gap_sec'] = max(
                entry['maximum_receipt_gap_sec'], wall - entry['last_receipt'])
        entry.update(count=entry['count'] + 1, last_receipt=wall)
        if isinstance(message, Clock):
            self.clock_latest = message.clock.sec + message.clock.nanosec * 1e-9
            if self.clock_first is None:
                self.clock_first = self.clock_latest
        if hasattr(message, 'header'):
            entry['stamp'] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if isinstance(message, Odometry):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            entry['position'] = [p.x, p.y, p.z]
            entry['orientation'] = [q.x, q.y, q.z, q.w]
        if isinstance(message, Bool):
            entry['true_count'] = entry.get('true_count', 0) + int(message.data)
            entry['data'] = message.data
        if isinstance(message, String):
            try:
                entry['data'] = json.loads(message.data)
            except ValueError:
                entry['data'] = message.data
        if isinstance(message, DiagnosticArray):
            for status in message.status:
                self.diagnostics[status.name] = {
                    'level': (int.from_bytes(status.level, 'little')
                              if isinstance(status.level, bytes) else status.level),
                    'message': status.message,
                    'values': {value.key: value.value for value in status.values}}

    def report(self, duration):
        elapsed = (self.clock_latest - self.clock_first
                   if self.clock_first is not None else 0.)
        wall = time.monotonic()
        topics = {}
        for topic, value in self.topics.items():
            entry = {key: item for key, item in value.items() if key != 'last_receipt'}
            entry['receipt_age_sec'] = wall - value['last_receipt']
            if 'stamp' in value and self.clock_latest is not None:
                entry['measurement_age_sec'] = self.clock_latest - value['stamp']
            entry['rate_per_sim_second'] = value['count'] / elapsed if elapsed > 0 else None
            topics[topic] = entry
        return {'wall_duration_sec': duration, 'sim_duration_sec': elapsed,
                'clock': self.clock_latest, 'topics': topics,
                'diagnostics': self.diagnostics}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node = RuntimeProbe()
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        report = json.dumps(node.report(time.monotonic()-start), indent=2) + '\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(report)
        print(report, end='')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
