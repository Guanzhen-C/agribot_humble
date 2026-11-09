"""PC-only native CAN plant/chassis emulator. No ROS drive-command bypass."""
import json
from pathlib import Path
import socket
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .isobus_can_mux import CAN_FRAME
from .tracked_work_wire import CounterLease, body_velocity, decode_drive, decode_work, drive_feedback, plant_feedback


def open_physical(interface, serial):
    device = (Path('/sys/class/net')/interface/'device').resolve()
    if (device/'driver').resolve().name != 'gs_usb' or (device.parent/'serial').read_text().strip() != serial:
        raise ValueError('Unexpected PC physical USB-CAN identity')
    link, = json.loads(subprocess.check_output(['ip', '-j', '-d', 'link', 'show', interface]))
    info = link['linkinfo']['info_data']
    if ('UP' not in link['flags'] or link['mtu'] != 16 or
            info['bittiming']['bitrate'] != 250000 or info['state'] != 'ERROR-ACTIVE'):
        raise ValueError('Tracked HIL requires classic CAN, 250 kbps, UP and ERROR-ACTIVE')
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                   b''.join(struct.pack('=II', ident, 0xc00007ff) for ident in (0x514, 0x580)))
    bus.bind((interface,))
    return bus


class TrackedGazeboCanBridge(Node):
    def __init__(self):
        super().__init__('hil_tracked_gazebo_can_bridge')
        self.motion = self.declare_parameter('motion_authorized', False).value
        self.work = self.declare_parameter('work_authorized', False).value
        interface = self.declare_parameter('can_interface', 'can0').value
        serial = self.declare_parameter('usb_serial', '003000224856511520343739').value
        self.bus = open_physical(interface, serial)
        self.drive_lease, self.work_lease = CounterLease(.5), CounterLease(.5)
        self.target = (0., 0., False)
        self.gears = (0, 0, 0)
        self.speed = self.yaw = 0.
        self.rpm = [0., 0.]
        self.truth_at = self.motors_at = -float('inf')
        self.counter = 0
        self.rx = self.tx = self.rejected = 0
        self.last_feedback = self.last_work_feedback = self.last_status = 0.
        self.fault = ''
        self.drive_pub = self.create_publisher(Twist, '/hil/actuator/cmd_vel', 1)
        self.work_pub = self.create_publisher(Float64MultiArray, '/hil/work/target_rpm', 1)
        self.status = self.create_publisher(String, '/hil/tracked_can/status', 1)
        self.create_subscription(Odometry, '/hil/ground_truth', self.truth, 5)
        self.create_subscription(JointState, '/hil/work/joint_states', self.motors, 5)
        self.add_on_set_parameters_callback(self.configure)
        self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def configure(self, parameters):
        for parameter in parameters:
            if parameter.name in ('can_interface', 'usb_serial') and parameter.value != self.get_parameter(parameter.name).value:
                return SetParametersResult(successful=False, reason='Restart required for physical bus changes')
        for parameter in parameters:
            if parameter.name == 'motion_authorized':
                self.motion = bool(parameter.value)
                self.target = (0., 0., False)
                self.drive_lease = CounterLease(.5)
            if parameter.name == 'work_authorized':
                self.work = bool(parameter.value)
                self.gears = (0, 0, 0)
                self.work_lease = CounterLease(.5)
        return SetParametersResult(successful=True)

    def truth(self, msg):
        q, v, w = msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        self.speed, self.yaw = body_velocity([q.x, q.y, q.z, q.w], [v.x, v.y, v.z], [w.x, w.y, w.z])
        self.truth_at = time.monotonic()

    def motors(self, msg):
        import math
        mapping = dict(zip(msg.name, msg.velocity))
        names = ('work_fan_joint', 'work_pump_joint')
        if all(name in mapping and math.isfinite(mapping[name]) for name in names):
            self.rpm = [mapping[name] * 30 / math.pi for name in names]
            self.motors_at = time.monotonic()

    def send(self, identifier, payload):
        self.bus.send(CAN_FRAME.pack(identifier, 8, payload))
        self.tx += 1

    def tick(self):
        now = time.monotonic()
        try:
            for _ in range(64):
                try:
                    raw = self.bus.recv(CAN_FRAME.size)
                except BlockingIOError:
                    break
                try:
                    identifier, length, data = CAN_FRAME.unpack(raw)
                    if identifier & 0xe0000000 or length != 8:
                        raise ValueError('Only eight-byte standard native frames accepted')
                    if identifier == 0x514:
                        target = decode_drive(data)
                        accepted = self.drive_lease.accept(data, now, target[:2] == (0., 0.))
                        if accepted:
                            self.target = target
                    elif identifier == 0x580:
                        gears = decode_work(data)
                        # Shredder is codec-tested, deliberately absent from this spraying scene.
                        if gears[2]:
                            raise ValueError('Shredder is not mounted in the spraying HIL scene')
                        accepted = self.work_lease.accept(data, now, not any(gears))
                        if accepted:
                            self.gears = gears
                    else:
                        raise ValueError('Unexpected native command')
                    if not accepted:
                        raise ValueError('Duplicate, stale or unprimed command')
                    self.rx += 1
                except (ValueError, struct.error):
                    self.rejected += 1
            truth_fresh = now - self.truth_at < .5
            motors_fresh = now - self.motors_at < .5
            command = Twist()
            if self.motion and truth_fresh and self.drive_lease.fresh(now) and not self.fault:
                command.linear.x, command.angular.z = self.target[:2]
            self.drive_pub.publish(command)
            output_gears = self.gears[:2] if self.work and motors_fresh and self.work_lease.fresh(now) and not self.fault else (0, 0)
            # Illustrative motor curves, NOT measured implement calibration.
            self.work_pub.publish(Float64MultiArray(data=[output_gears[0] * 300., output_gears[1] * 200.]))
            if truth_fresh and now - self.last_feedback >= .1:
                for identifier, data in drive_feedback(self.speed, self.yaw, self.target[2], self.counter):
                    self.send(identifier, data)
                self.counter = (self.counter + 1) & 15
                self.last_feedback = now
            if motors_fresh and now - self.last_work_feedback >= .5:
                for identifier, data in plant_feedback(*self.rpm, output_gears, self.counter, bool(self.fault)):
                    self.send(identifier, data)
                self.last_work_feedback = now
        except (OSError, ValueError) as error:
            self.fault = str(error)
            self.drive_pub.publish(Twist())
            self.work_pub.publish(Float64MultiArray(data=[0., 0.]))
            self.get_logger().error(self.fault, throttle_duration_sec=2.)
        if now - self.last_status >= 1:
            self.status.publish(String(data=json.dumps(dict(
                motion_authorized=self.motion, work_authorized=self.work,
                rx=self.rx, tx=self.tx, rejected=self.rejected, fault=self.fault,
                drive_lease_fresh=self.drive_lease.fresh(now),
                work_lease_fresh=self.work_lease.fresh(now),
                actual_fan_rpm=self.rpm[0], actual_pump_rpm=self.rpm[1],
                truth_fresh=now-self.truth_at < .5, motors_fresh=now-self.motors_at < .5))))
            self.last_status = now

    def destroy_node(self):
        if rclpy.ok():
            self.drive_pub.publish(Twist())
            self.work_pub.publish(Float64MultiArray(data=[0., 0.]))
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrackedGazeboCanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
