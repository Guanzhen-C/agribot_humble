#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from scout_msgs.msg import ScoutControl
from sensor_msgs.msg import Joy


BUTTON_A = 0
BUTTON_B = 1
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5

LEFT_HORIZ = 0
LEFT_VERT = 1

BRAKE_POINT = -0.2
SMALL_VALUE = 1e-4


class JoystickTranslator(Node):
    def __init__(self) -> None:
        super().__init__("joystick_translator")
        joystick_topic = self.declare_parameter("joystick_topic", "/joy_teleop/joy").value
        vehicle_control_topic = self.declare_parameter(
            "vehicle_control_topic", "/scout_control"
        ).value
        self.publish_hz = float(self.declare_parameter("publish_hz", 20.0).value)

        self.publisher = self.create_publisher(ScoutControl, vehicle_control_topic, 10)
        self.subscription = self.create_subscription(Joy, joystick_topic, self.handle_joy, 10)
        self.last_control = None
        self.last_rx_time = None
        self.no_joy_warning_printed = False
        self.create_timer(1.0 / max(self.publish_hz, 1e-3), self.handle_timer)

    def handle_timer(self) -> None:
        if self.last_control is None or self.last_rx_time is None:
            return
        if self.get_clock().now() - self.last_rx_time > Duration(seconds=0.25):
            return
        self.last_control.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.last_control)

    def handle_joy(self, message: Joy) -> None:
        if len(message.buttons) <= BUTTON_RB:
            if not self.no_joy_warning_printed:
                self.get_logger().warn("No proper joystick is attached.")
                self.no_joy_warning_printed = True
            return

        control = ScoutControl()
        control.stamp = message.header.stamp
        control.gearshift = ScoutControl.NEUTRAL

        if message.buttons[BUTTON_Y] == 1:
            control.gearshift = ScoutControl.FORWARD
        elif message.buttons[BUTTON_A] == 1:
            control.gearshift = ScoutControl.REVERSE
        elif message.buttons[BUTTON_B] == 1:
            control.gearshift = ScoutControl.NEUTRAL
        elif self.last_control is not None:
            control.gearshift = self.last_control.gearshift

        enable = False
        control.enable_turbo = False
        if message.buttons[BUTTON_RB] == 1:
            control.enable_turbo = True
        if message.buttons[BUTTON_LB] == 1:
            enable = True
            control.enable_turbo = False
        if not enable and not control.enable_turbo:
            return

        if message.axes[LEFT_VERT] < BRAKE_POINT:
            control.brake = -float(message.axes[LEFT_VERT])
        if abs(message.axes[LEFT_VERT]) < SMALL_VALUE:
            control.brake = 0.0

        if message.axes[LEFT_VERT] >= 0.0:
            control.throttle = float(message.axes[LEFT_VERT])
            control.brake = 0.0
        else:
            control.throttle = 0.0

        control.steering = float(message.axes[LEFT_HORIZ])
        self.last_control = deepcopy(control)
        self.last_rx_time = self.get_clock().now()
        self.publisher.publish(control)


def main() -> None:
    rclpy.init()
    node = JoystickTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
