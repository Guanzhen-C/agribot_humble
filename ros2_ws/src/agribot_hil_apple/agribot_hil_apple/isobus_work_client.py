"""ROS command lease adapter for the pinned C++ AgIsoStack++ client."""
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from .isobus_can_mux import open_virtual
from .tracked_work_wire import mounted_work


def parse_target(text, allow_shredder=False):
    target = json.loads(text)
    names = ('fan_gear', 'pump_gear', 'shredder_level')
    if not isinstance(target, dict) or set(target) != set(names):
        raise ValueError('Expected fan_gear, pump_gear and shredder_level')
    values = [target[name] for name in names]
    if any(type(value) is not int for value in values) or not all(
            0 <= value <= limit for value, limit in zip(values, (8, 8, 10000))):
        raise ValueError('Invalid implement range/type')
    if values[2] and not allow_shredder:
        raise ValueError('Shredder is not mounted in this spraying HIL scene')
    return values


class IsobusWorkClient(Node):
    def __init__(self):
        super().__init__('hil_isobus_work_client')
        default = Path.home()/'.local/state/agribot-isobus-hil-20261109/build-host/hil_isobus_endpoint'
        executable = self.declare_parameter('endpoint_executable', str(default)).value
        interface = self.declare_parameter('virtual_interface', 'vcan73').value
        self.implement = self.declare_parameter('implement_kind', 'sprayer').value
        mounted_work([0, 0, 0], self.implement)
        self.authorized = self.declare_parameter('work_authorized', False).value
        self.allow_shredder = self.declare_parameter('allow_shredder', self.implement == 'shredder').value
        if self.allow_shredder != (self.implement == 'shredder'):
            raise ValueError('Implement kind and shredder permission disagree')
        self.values = [0, 0, 0]
        self.command_at = -math.inf
        self.ready = False
        self.last_ready = False
        self.events = queue.Queue(maxsize=4096)
        self.reader_overflow = threading.Event()
        check = open_virtual(interface)
        check.close()
        self.process = subprocess.Popen([executable, 'client', interface], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=None, text=True, bufsize=1)
        self.reader = threading.Thread(target=self.read_events, daemon=True)
        self.reader.start()
        self.publisher = self.create_publisher(String, '/hil/isobus/events', 20)
        self.create_subscription(String, '/hil/work/command', self.command, 1)
        self.add_on_set_parameters_callback(self.configure)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def read_events(self):
        for line in self.process.stdout:
            try:
                self.events.put_nowait(json.loads(line))
            except (ValueError, queue.Full):
                self.reader_overflow.set()
                return

    def configure(self, parameters):
        for parameter in parameters:
            if parameter.name != 'work_authorized' and parameter.name in (
                    'endpoint_executable', 'virtual_interface', 'allow_shredder', 'implement_kind'):
                if parameter.value != self.get_parameter(parameter.name).value:
                    return SetParametersResult(successful=False, reason='Restart required')
        for parameter in parameters:
            if parameter.name == 'work_authorized':
                self.authorized = bool(parameter.value)
                self.values = [0, 0, 0]
                self.command_at = -math.inf
                self.write('STOP\n')
        return SetParametersResult(successful=True)

    def write(self, text):
        if self.process.poll() is not None:
            raise RuntimeError('ISOBUS endpoint exited')
        self.process.stdin.write(text)
        self.process.stdin.flush()

    def command(self, msg):
        try:
            values = parse_target(msg.data, self.allow_shredder)
            mounted_work(values, self.implement)
            if any(values) and (not self.authorized or not self.ready):
                raise ValueError('Implement authorization/address claim not ready')
            self.values = values
            self.command_at = time.monotonic()
        except ValueError as error:
            self.values = [0, 0, 0]
            self.command_at = -math.inf
            self.write('STOP\n')
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def tick(self):
        try:
            if self.reader_overflow.is_set():
                raise RuntimeError('ISOBUS event reader failed/overflowed')
            for _ in range(256):
                try:
                    event = self.events.get_nowait()
                except queue.Empty:
                    break
                if event.get('kind') == 'network':
                    self.ready = bool(event['ready'])
                if event.get('kind') == 'ack' and bytes.fromhex(event['data'])[3] != 0:
                    self.values = [0, 0, 0]
                    self.command_at = -math.inf
                self.publisher.publish(String(data=json.dumps(event)))
            if self.ready != self.last_ready:
                self.values = [0, 0, 0]
                self.command_at = -math.inf
                self.last_ready = self.ready
            fresh = time.monotonic() - self.command_at < .25
            values = self.values if self.authorized and self.ready and fresh else [0, 0, 0]
            self.write(''.join(f'SET {device} {value} {int(value > 0)}\n'
                               for device, value in enumerate(values, 1)))
        except (RuntimeError, OSError) as error:
            self.authorized = False
            self.values = [0, 0, 0]
            self.get_logger().error(str(error), throttle_duration_sec=2.)
            if self.process.poll() is None:
                self.process.terminate()

    def destroy_node(self):
        if self.process.poll() is None:
            try:
                self.write('STOP\n')
                self.process.stdin.close()
                self.process.wait(timeout=2)
            except (OSError, RuntimeError, subprocess.TimeoutExpired):
                self.process.terminate()
                self.process.wait(timeout=2)
        self.reader.join(timeout=1)
        super().destroy_node()


def main():
    rclpy.init()
    node = IsobusWorkClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
