"""Read-only PC evidence: CAN commands, actual plant/chassis motion and fusion."""
import argparse
import json
from pathlib import Path
import socket
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from agribot_hil_apple.isobus_can_mux import CAN_FRAME
from agribot_hil_apple.tracked_work_wire import validate


class Recorder(Node):
    def __init__(self):
        super().__init__('hil_tracked_orchard_recorder')
        self.start = time.monotonic()
        self.report = dict(source='Independent PC observations; no command/localization publisher',
            counts={}, samples={}, native_can=[], can_errors=[])
        self.sample_at = {}
        topics = [('truth', '/hil/ground_truth', Odometry), ('fused', '/fastlivo_rtk/odometry', Odometry),
                  ('wheel', '/hil/wheel/odometry', Odometry), ('motors', '/hil/work/joint_states', JointState),
                  ('ready', '/fastlivo_rtk/ready', Bool), ('bridge', '/hil/tracked_can/status', String)]
        for name, topic, kind in topics:
            self.create_subscription(kind, topic, lambda message, key=name: self.receive(key, message), qos_profile_sensor_data)
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
        self.bus.bind(('can0',))
        self.create_timer(.02, self.read_can)

    def receive(self, name, msg):
        now = time.monotonic()-self.start
        self.report['counts'][name] = self.report['counts'].get(name, 0)+1
        if now-self.sample_at.get(name, -1) < .2:
            return
        self.sample_at[name] = now
        if isinstance(msg, Odometry):
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            value = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z]
        elif isinstance(msg, JointState):
            value = dict(zip(msg.name, msg.velocity))
        elif isinstance(msg, String):
            value = json.loads(msg.data)
        else:
            value = msg.data
        self.report['samples'].setdefault(name, []).append([now, value])

    def read_can(self):
        for _ in range(128):
            try:
                raw = self.bus.recv(CAN_FRAME.size)
            except BlockingIOError:
                break
            try:
                identifier, length, data = CAN_FRAME.unpack(raw)
                if length != 8 or identifier & 0xe0000000:
                    raise ValueError('Unexpected non-native CAN frame')
                validate(data)
                self.report['native_can'].append([time.monotonic()-self.start, identifier, data.hex()])
            except ValueError as error:
                self.report['can_errors'].append(str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=900.)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init()
    node = Recorder()
    try:
        while time.monotonic()-node.start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report['wall_seconds'] = time.monotonic()-node.start
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(node.report, indent=2)+'\n')
        node.bus.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
