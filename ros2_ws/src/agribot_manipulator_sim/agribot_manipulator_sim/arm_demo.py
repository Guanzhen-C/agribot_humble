"""Animate simulated joints only; never command a physical arm or mobile base."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ["manipulator_joint" + str(index) for index in range(1, 7)] + [
    "manipulator_finger_joint1", "manipulator_finger_joint2"]


def demo_positions(seconds):
    # Small, repeatable workspace above the mounting plate, not a grasp planner.
    wave = math.sin(seconds * math.tau / 16.0)
    lift = (1.0 - math.cos(seconds * math.tau / 16.0)) * 0.5
    return [0.55 * wave, -0.25 * lift, 0.65 * lift,
            0.25 * wave, 0.3 * lift, -0.45 * wave, 0.0089 * lift, 0.0089 * lift]


def approach(current, target, step):
    return [old + max(-step, min(step, new - old)) for old, new in zip(current, target)]


class ArmDemo(Node):
    def __init__(self):
        super().__init__("vehicle_manipulator_demo")
        self.enabled = bool(self.declare_parameter("demo_enabled", True).value)
        self.moving = False
        self.have_feedback = False
        self.stowed = False
        self.positions = [0.0] * len(JOINT_NAMES)
        self.phase = 0.0
        self.last_tick = None
        self.publisher = self.create_publisher(JointTrajectory, "/manipulator/joint_trajectory", 1)
        self.create_subscription(JointState, "/joint_states", self.feedback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.command, 10)
        self.last_command = None
        self.create_service(SetBool, "/manipulator/demo_enabled", self.set_enabled)
        self.create_timer(0.05, self.tick)

    def feedback(self, msg):
        if len(msg.name) != len(msg.position):
            return
        state = dict(zip(msg.name, msg.position))
        if all(name in state for name in JOINT_NAMES):
            self.positions = [state[name] for name in JOINT_NAMES]
            self.have_feedback = True

    def command(self, msg):
        # Chassis commands avoid false movement caused by parked-wheel jitter.
        self.moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        self.last_command = self.get_clock().now()

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.phase = 0.0
        response.success = True
        response.message = "Demonstration enabled" if self.enabled else "Returning to home"
        return response

    def tick(self):
        now = self.get_clock().now()
        if not self.have_feedback or now.nanoseconds == 0:
            return
        dt = 0.05 if self.last_tick is None else (now.nanoseconds - self.last_tick) / 1e9
        self.last_tick = now.nanoseconds
        if dt == 0.0:
            # Several timer callbacks can share one /clock message.
            return
        if dt < 0.0 or dt > 0.5:
            self.phase = 0.0
            return
        moving = self.moving and self.last_command is not None and (
            now.nanoseconds - self.last_command.nanoseconds < 1_000_000_000)
        active = self.enabled and not moving
        if active:
            self.stowed = False
            self.phase += dt
        else:
            self.phase = 0.0
        target = demo_positions(self.phase) if active else [0.0] * len(JOINT_NAMES)
        if not active and self.stowed:
            # Once stowed, do not repeatedly reset model/joint poses in motion.
            return
        message = JointTrajectory()
        # The standard Gazebo plugin preserves the model's world pose while
        # setting only these joints. It does not publish any chassis command.
        message.header.frame_id = "world"
        message.header.stamp = now.to_msg()
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = approach(self.positions, target, 0.6 * dt)
        message.points = [point]
        self.publisher.publish(message)
        if not active and not any(point.positions):
            self.stowed = True


def main():
    rclpy.init()
    node = ArmDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
