"""Initialize real tree fixtures and the collision-free arm assembly pose."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from linkattacher_msgs.srv import AttachLink
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    mission = json.loads((Path.home()/'.cache/agribot/hil_apple/rdk/mission.json').read_text())
    rclpy.init()
    node = Node('hil_scene_bootstrap', parameter_overrides=[Parameter('use_sim_time', value=True)])
    states = {}
    subscription = node.create_subscription(JointState, '/joint_states',
        lambda msg: states.update(zip(msg.name, msg.position)), 20)
    publisher = node.create_publisher(JointTrajectory, '/manipulator/joint_trajectory', 1)
    attachment = node.create_client(AttachLink, '/ATTACHLINK')
    try:
        if not attachment.wait_for_service(timeout_sec=20.):
            raise RuntimeError('Physical attachment plugin is unavailable')
        for index in range(len(mission['apples'])):
            request = AttachLink.Request(model1_name='orchard_geometry', link1_name='orchard_world_link',
                model2_name=f'harvest_apple_{index+1}', link2_name='fruit')
            future = attachment.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            if not future.done() or not future.result().success:
                raise RuntimeError(f'Cannot initialize tree fixture {index+1}')
        names = [f'manipulator_joint{i}' for i in range(1, 7)] + [
            'manipulator_finger_joint1', 'manipulator_finger_joint2']
        deadline = time.monotonic()+20.
        while not all(name in states for name in names):
            if time.monotonic() > deadline:
                raise TimeoutError('Mounted arm feedback is missing')
            rclpy.spin_once(node, timeout_sec=.1)
        target = [0., 0., 1.57, 0., 0., 0., .0089, .0089]
        message = JointTrajectory()
        message.header.frame_id = 'world'
        message.header.stamp = node.get_clock().now().to_msg()
        message.joint_names = names
        message.points = [JointTrajectoryPoint(positions=target)]
        publisher.publish(message)
        deadline = time.monotonic()+5.
        while max(abs(states[name]-value) for name, value in zip(names, target)) > .01:
            if time.monotonic() > deadline:
                raise TimeoutError('Initial arm assembly pose was not applied')
            rclpy.spin_once(node, timeout_sec=.05)
        node.get_logger().info('Tree fixtures and safe initial arm assembly pose are ready')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
