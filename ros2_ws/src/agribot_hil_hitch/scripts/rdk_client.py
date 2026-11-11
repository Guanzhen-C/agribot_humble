"""X5 high-level lift client and dedicated virtual-tractor CAN driver."""
import argparse
import json
import math
from pathlib import Path
import queue
import socket
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from wire import DRIVE, DRIVE_STATE, drive_command, validate
from powertrain_wire import parse_profile, healthy_feedback


def parse_command(text):
    values = json.loads(text)
    if not isinstance(values, dict) or set(values) != {'mode', 'position', 'speed'}:
        raise ValueError('Expected mode, position and speed')
    mode, position, speed = (values[k] for k in ('mode', 'position', 'speed'))
    if mode not in ('hold', 'position', 'raise', 'lower') or type(position) is not int or \
            type(speed) is not int or not 0 <= position <= 10000 or not 0 <= speed <= 100 or \
            (mode != 'hold' and speed == 0):
        raise ValueError('Invalid lift command')
    return ('hold', 'position', 'raise', 'lower').index(mode), position, speed


class Client(Node):
    def __init__(self, args):
        super().__init__('hil_hitch_rdk_client')
        self.trace = args.trace.open('x')
        self.endpoint = subprocess.Popen([str(args.endpoint), 'client', args.interface], stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, text=True, bufsize=1)
        self.events = queue.Queue(maxsize=4096)
        self.reader_failed = threading.Event()
        self.reader = threading.Thread(target=self.read, daemon=True)
        self.reader.start()
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
        self.bus.bind((args.interface,))
        self.frame = struct.Struct('=IB3x8s')
        self.drive_authorized = self.declare_parameter('drive_authorized', False).value
        self.work_authorized = self.declare_parameter('work_authorized', False).value
        self.require_localization = self.declare_parameter('require_localization', True).value
        self.powertrain_enabled = self.declare_parameter('powertrain_enabled', False).value
        self.power_target = (0, 0, 1600, 0., 600, 150.)
        self.power_at = -math.inf
        self.last_power_gate = None
        self.power_data = {}
        self.power_publish_at = -math.inf
        self.drive_target = (0., 0.)
        self.drive_at = self.work_at = self.pose_at = -math.inf
        self.target = (0, 0, 0)
        self.ready, self.was_ready = False, False
        self.prime_until = time.monotonic()+.5
        self.sequence = 0
        self.state_pub = self.create_publisher(String, '/hil_hitch/isobus/events', 20)
        self.power_pub = self.create_publisher(String, '/hil_hitch/powertrain/status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom_pub = self.create_publisher(Odometry, '/hil_hitch/wheel_odom', 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self.velocity, 1)
        self.create_subscription(String, '/hil_hitch/command', self.work, 1)
        self.create_subscription(String, '/hil_hitch/powertrain/command', self.power, 1)
        self.create_subscription(Odometry, '/odometry/filtered_navsat', self.pose, 10)
        self.add_on_set_parameters_callback(self.configure)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.x, self.y, self.yaw = -20., 0., 0.
        self.odom_stamp = None

    def read(self):
        try:
            for line in self.endpoint.stdout:
                self.events.put_nowait(json.loads(line))
        except (ValueError, queue.Full):
            self.reader_failed.set()

    def write(self, text):
        if self.endpoint.poll() is not None:
            raise RuntimeError('Hitch endpoint exited')
        self.endpoint.stdin.write(text+'\n'); self.endpoint.stdin.flush()

    def configure(self, parameters):
        for item in parameters:
            if item.name in ('drive_authorized', 'work_authorized') and type(item.value) is not bool:
                return SetParametersResult(successful=False, reason='Authorization must be boolean')
            if (item.name == 'require_localization' and item.value != self.require_localization) or \
                    (item.name == 'powertrain_enabled' and item.value != self.powertrain_enabled):
                return SetParametersResult(successful=False, reason='Restart required')
        for item in parameters:
            if item.name == 'drive_authorized':
                self.drive_authorized = item.value; self.drive_target = (0., 0.); self.drive_at = -math.inf
                self.power_target = (0, 0, 1600, 0., 600, 150.); self.power_at = -math.inf
                self.write('STOP_POWERTRAIN')
                self.prime_until = time.monotonic()+.3
            elif item.name == 'work_authorized':
                self.work_authorized = item.value; self.target = (0, 0, 0); self.work_at = -math.inf
                self.write('STOP'); self.prime_until = time.monotonic()+.3
        return SetParametersResult(successful=True)

    def velocity(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if not all(math.isfinite(x) for x in (v, w)) or not 0 <= v <= .5001:
            self.drive_target = (0., 0.); self.drive_at = -math.inf
            return
        steering = max(-.61, min(.61, math.atan2(2.1851241912748174*w, v))) if v > .005 else 0.
        self.drive_target = (min(v, .5), steering); self.drive_at = time.monotonic()

    def work(self, msg):
        try:
            self.target = parse_command(msg.data); self.work_at = time.monotonic()
        except (ValueError, TypeError):
            self.target = (0, 0, 0); self.work_at = -math.inf; self.write('STOP')

    def power(self, msg):
        try:
            self.power_target = parse_profile(msg.data); self.power_at = time.monotonic()
        except (ValueError, TypeError) as error:
            self.power_target = (0, 0, 1600, 0., 600, 150.); self.power_at = -math.inf
            self.write('STOP_POWERTRAIN')
            self.trace.write(json.dumps(dict(at=time.monotonic(), event=dict(
                kind='powertrain_invalid_command', reason=str(error))))+'\n')

    def power_event(self, event, now):
        kind = event['kind']
        if not kind.startswith('powertrain_') or 'data' not in event:
            return
        raw = bytes.fromhex(event['data']); validate(raw)
        self.power_data[kind] = dict(at=now, data=raw.hex())
        if kind == 'powertrain_ack' and raw[1] != 0:
            self.power_target = (0, 0, 1600, 0., 600, 150.); self.power_at = -math.inf

    def pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.pose_at = time.monotonic()

    def feedback(self, data):
        validate(data)
        speed, steer, state, fault, _ = struct.unpack('<hhBBB', data[:7])
        stamp = self.get_clock().now()
        dt = 0. if self.odom_stamp is None else (stamp-self.odom_stamp).nanoseconds*1e-9
        self.odom_stamp = stamp
        v, steering = speed*.001, steer*.0001
        yaw_rate = v*math.tan(steering)/2.1851241912748174
        if 0 < dt < .3 and state == 1 and not fault:
            self.x += v*math.cos(self.yaw)*dt; self.y += v*math.sin(self.yaw)*dt
            self.yaw += yaw_rate*dt
        msg = Odometry(); msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x; msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw/2); msg.pose.pose.orientation.w = math.cos(self.yaw/2)
        msg.twist.twist.linear.x = v; msg.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.reader_failed.is_set():
            raise RuntimeError('Hitch endpoint reader failed')
        for _ in range(256):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if event['kind'] == 'network':
                self.ready = event['ready']
            if event['kind'] == 'ack' and bytes.fromhex(event['data'])[1] != 0:
                self.target = (0, 0, 0); self.work_at = -math.inf
            self.power_event(event, now)
            self.state_pub.publish(String(data=json.dumps(event)))
            self.trace.write(json.dumps(dict(at=now, event=event))+'\n')
        if self.ready != self.was_ready:
            self.target = (0, 0, 0); self.work_at = self.drive_at = -math.inf
            self.prime_until = now+.3; self.was_ready = self.ready
        # Monitoring must never backpressure the command lease/control loop.
        if self.power_data and now-self.power_publish_at >= .05:
            self.power_pub.publish(String(data=json.dumps(dict(self.power_data, snapshot_at=now))))
            self.power_publish_at = now
        working = self.work_authorized and self.ready and now >= self.prime_until and now-self.work_at < .25
        mode, position, speed = self.target if working else (0, 0, 0)
        self.write(f'SET {mode} {position} {speed} {int(mode != 0)}')
        moving = self.drive_authorized and self.ready and now >= self.prime_until and now-self.drive_at < .2 and \
            (not self.require_localization or now-self.pose_at < .8)
        if self.powertrain_enabled and not healthy_feedback(self.power_data, now):
            moving = False
        power_on = self.powertrain_enabled and self.drive_authorized and self.ready and \
            now >= self.prime_until and now-self.power_at < .25
        pmode, gear, rpm, ratio, shift, rate = self.power_target if power_on else (0, 0, 1600, 0., 600, 150.)
        gate = (self.powertrain_enabled, self.drive_authorized, self.ready,
                now >= self.prime_until, now-self.power_at < .25, pmode)
        if gate != self.last_power_gate:
            self.trace.write(json.dumps(dict(at=now, event=dict(kind='powertrain_gate',
                enabled=gate[0], authorized=gate[1], ready=gate[2], primed=gate[3],
                fresh=gate[4], mode=pmode)))+'\n')
            self.last_power_gate = gate
        self.write(f'SETTINGS {shift} {rate}')
        self.write(f'POWER {pmode} {gear} {rpm} {ratio}')
        v, steer = self.drive_target if moving else (0., 0.)
        data = drive_command(v, steer, moving, self.sequence)
        self.bus.send(self.frame.pack(DRIVE, 8, data)); self.sequence = (self.sequence+1) & 255
        for _ in range(64):
            try:
                identity, size, data = self.frame.unpack(self.bus.recv(self.frame.size))
            except BlockingIOError:
                break
            if identity == DRIVE_STATE and size == 8:
                try:
                    self.feedback(data)
                except ValueError:
                    pass
        self.trace.flush()

    def destroy_node(self):
        if self.endpoint.poll() is None:
            try:
                self.write('STOP'); self.endpoint.stdin.close(); self.endpoint.wait(timeout=2)
            except (OSError, RuntimeError, subprocess.TimeoutExpired):
                self.endpoint.terminate(); self.endpoint.wait(timeout=2)
        try:
            data = drive_command(0., 0., False, self.sequence)
            self.bus.send(self.frame.pack(DRIVE, 8, data))
        finally:
            self.bus.close(); self.trace.close(); self.reader.join(timeout=1)
            super().destroy_node()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', type=Path, required=True)
    parser.add_argument('--interface', default='vcan74')
    parser.add_argument('--trace', type=Path, required=True)
    args, ros = parser.parse_known_args()
    if args.interface != 'vcan74':
        raise ValueError('Only the isolated hitch bench interface is permitted')
    args.trace.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros)
    node = Client(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
