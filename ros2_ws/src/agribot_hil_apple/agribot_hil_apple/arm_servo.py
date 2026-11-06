"""Persistent position-actuator adapter for Gazebo's official pose plugin.

The input has standard absolute ROS trajectory timing. The output uses one
configuration at a time, avoiding the legacy plugin's incremental-time quirk.
This is a kinematic simulation actuator, not a torque-controlled real arm.
"""
import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_preflight import JOINTS, FINGERS, STOW


def command_times(message, names):
    if (not message.points or not message.joint_names or
            len(set(message.joint_names)) != len(message.joint_names) or
            not set(message.joint_names) <= set(names)):
        raise ValueError('Invalid simulation arm joint names or empty trajectory')
    times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in message.points]
    if (any(len(p.positions) != len(message.joint_names) or
            not all(math.isfinite(v) for v in p.positions) for p in message.points) or
            times[0] < 0. or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('Invalid simulation arm trajectory sizes, values or timestamps')
    return times


class ArmServo(Node):
    def __init__(self):
        super().__init__('hil_arm_position_servo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.names = JOINTS+FINGERS
        self.target = dict(zip(self.names, STOW))
        self.command, self.times, self.start = None, [], 0.
        self.publisher = self.create_publisher(JointTrajectory, '/manipulator/joint_trajectory', 2)
        self.create_subscription(JointTrajectory, '/hil/arm/command', self.receive, 2)
        self.create_timer(.02, self.update)

    def receive(self, message):
        try:
            times = command_times(message, self.names)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.command, self.times = message, times
        self.start = max(message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                         self.get_clock().now().nanoseconds/1e9)

    def update(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        elapsed = now.nanoseconds/1e9-self.start
        if self.command is not None and elapsed >= self.times[0]:
            index = min(bisect.bisect_right(self.times, elapsed)-1, len(self.times)-1)
            self.target.update(zip(self.command.joint_names, self.command.points[index].positions))
            if index == len(self.times)-1:
                self.command = None
        message = JointTrajectory(joint_names=self.names,
                                  points=[JointTrajectoryPoint(positions=[self.target[n] for n in self.names])])
        message.header.frame_id = 'hil_ackermann::base_link'
        message.header.stamp = now.to_msg()
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = ArmServo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
