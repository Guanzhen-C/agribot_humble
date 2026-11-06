"""Native CAN is the only command source for the simulated chassis."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from .can_wire import CanSocket, COMMAND_ID, FEEDBACK_IDS, decode_command, feedback_payload


def actuator_command(payload, wheelbase, authorized):
    speed, steering = decode_command(payload)
    if not math.isfinite(wheelbase) or wheelbase <= 0:
        raise ValueError('Invalid wheelbase')
    if abs(speed) > .81 or abs(steering) > .385:
        raise ValueError('Command exceeds the physical Car0 limits')
    return (speed, speed * math.tan(steering) / wheelbase) if authorized else (0., 0.)


class GazeboCanBridge(Node):
    def __init__(self):
        super().__init__('hil_gazebo_can_bridge')
        self.authorized = self.declare_parameter('motion_authorized', False).value
        serial = self.declare_parameter('usb_serial', '').value.removeprefix('usb:')
        interface = self.declare_parameter('can_interface', 'can0').value
        self.wheelbase = self.declare_parameter('wheelbase_m', .5265874862670898).value
        self.bus = CanSocket(interface, serial, [COMMAND_ID])
        self.bus.socket.setblocking(False)
        self.publisher = self.create_publisher(Twist, '/hil/actuator/cmd_vel', 1)
        self.status = self.create_publisher(String, '/hil/can/status', 1)
        self.create_subscription(Odometry, '/hil/ground_truth', self.odometry, 5)
        self.last_command = self.last_truth = 0.
        self.speed = self.yaw_rate = 0.
        self.target = (0., 0.)
        self.rx = self.tx = self.errors = 0
        self.nonzero_rx = 0
        self.last_feedback = self.last_status = 0.
        self.fault = ''
        self.add_on_set_parameters_callback(self.configure_motion)
        self.timer = self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def configure_motion(self, parameters):
        for parameter in parameters:
            if parameter.name in ('usb_serial', 'can_interface', 'wheelbase_m'):
                if parameter.value != self.get_parameter(parameter.name).value:
                    return SetParametersResult(successful=False, reason='Physical CAN configuration requires a restart')
        for parameter in parameters:
            if parameter.name == 'motion_authorized':
                self.authorized = bool(parameter.value)
                if not self.authorized:
                    self.target = (0., 0.)
        return SetParametersResult(successful=True)

    def odometry(self, msg):
        self.speed, self.yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        self.last_truth = time.monotonic()

    def tick(self):
        now = time.monotonic()
        try:
            for _ in range(64):
                frame = self.bus.receive()
                if frame is None:
                    break
                identifier, data = frame
                if identifier != COMMAND_ID:
                    raise ValueError('Unexpected CAN command ID')
                self.target = actuator_command(data, self.wheelbase, self.authorized)
                self.nonzero_rx += int(abs(decode_command(data)[0]) > .0001)
                self.last_command = now
                self.rx += 1
            enabled = self.authorized and now - self.last_truth < .5 and not self.fault
            command = Twist()
            if enabled and now - self.last_command < .3:
                command.linear.x, command.angular.z = self.target
            self.publisher.publish(command)
            if (now - self.last_feedback >= .05 and now - self.last_truth < .5
                    and now - self.last_command < .5):
                payload = feedback_payload(self.speed, self.yaw_rate,
                                           angular_velocity=(0., 0., self.yaw_rate),
                                           stopped=not enabled)
                for index, identifier in enumerate(FEEDBACK_IDS):
                    self.bus.send(identifier, payload[index * 8:index * 8 + 8])
                    self.tx += 1
                self.last_feedback = now
        except Exception as error:
            self.fault = str(error)
            self.errors += 1
            self.publisher.publish(Twist())
            self.get_logger().error(self.fault, throttle_duration_sec=2.)
        if now - self.last_status > 1.:
            self.status.publish(String(data=json.dumps({
                'authorized': self.authorized, 'rx_commands': self.rx,
                'nonzero_commands': self.nonzero_rx,
                'tx_feedback': self.tx, 'errors': self.errors, 'fault': self.fault,
                'command_fresh': now - self.last_command < .3,
                'truth_fresh': now - self.last_truth < .5})))
            self.last_status = now

    def destroy_node(self):
        self.publisher.publish(Twist())
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GazeboCanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
