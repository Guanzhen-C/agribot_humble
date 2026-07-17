#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from scout_msgs.msg import ScoutControl


GEARSHIFT_NAMES = {
    ScoutControl.NO_COMMAND: "NO_COMMAND",
    ScoutControl.NEUTRAL: "NEUTRAL",
    ScoutControl.FORWARD: "FORWARD",
    ScoutControl.REVERSE: "REVERSE",
}


class ScoutControlTranslator(Node):
    def __init__(self) -> None:
        super().__init__("scout_control_translator")
        vehicle_control_topic = self.declare_parameter(
            "vehicle_control_topic", "/scout_control"
        ).value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/joy_teleop/cmd_vel").value
        self.scale_linear = float(self.declare_parameter("scale_linear", 0.4).value)
        self.scale_angular = float(self.declare_parameter("scale_angular", 0.6).value)
        self.scale_linear_turbo = float(self.declare_parameter("scale_linear_turbo", 1.0).value)
        self.scale_angular_turbo = float(self.declare_parameter("scale_angular_turbo", 1.2).value)
        self.last_gearshift = ScoutControl.NEUTRAL

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.subscription = self.create_subscription(
            ScoutControl, vehicle_control_topic, self.handle_control, 10
        )

    def handle_control(self, message: ScoutControl) -> None:
        scale_linear = self.scale_linear_turbo if message.enable_turbo else self.scale_linear
        scale_angular = self.scale_angular_turbo if message.enable_turbo else self.scale_angular

        if message.gearshift != self.last_gearshift:
            self.get_logger().info(
                f"Gearshift: {GEARSHIFT_NAMES.get(self.last_gearshift, 'UNKNOWN')} -> "
                f"{GEARSHIFT_NAMES.get(message.gearshift, 'UNKNOWN')}"
            )
        self.last_gearshift = message.gearshift
        if message.gearshift == ScoutControl.NEUTRAL:
            return

        twist = Twist()
        if message.gearshift == ScoutControl.FORWARD:
            if message.throttle > 0.0:
                twist.linear.x = float(message.throttle * scale_linear)
            elif message.brake > 0.0:
                twist.linear.x = -float(message.brake * scale_linear)
        elif message.gearshift == ScoutControl.REVERSE:
            if message.throttle > 0.0:
                twist.linear.x = -float(message.throttle * scale_linear)
            elif message.brake > 0.0:
                twist.linear.x = float(message.brake * scale_linear)

        twist.angular.z = float(message.steering * scale_angular)
        self.publisher.publish(twist)


def main() -> None:
    rclpy.init()
    node = ScoutControlTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
