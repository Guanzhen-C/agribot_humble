"""Simulation object-pose observations for the manipulation HIL baseline, not LIO."""
import json
from pathlib import Path

import numpy as np
import rclpy
from gazebo_msgs.msg import LinkStates
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


class FruitObserver(Node):
    def __init__(self):
        super().__init__('hil_fruit_observer')
        manifest = json.loads((Path.home()/'.cache/agribot/hil_apple/rdk/mission.json').read_text())
        self.stem_offsets = manifest.get('fruit_socket_to_branch_m', {})
        self.publisher = self.create_publisher(String, '/hil/fruit/observations', 2)
        self.subscription = self.create_subscription(LinkStates, '/gazebo/link_states', self.receive, 2)

    def receive(self, message):
        indices = {name: index for index, name in enumerate(message.name)}
        base_index = indices.get('hil_ackermann::base_link')
        if base_index is None:
            return
        base = message.pose[base_index]
        rotation = Rotation.from_quat([base.orientation.x, base.orientation.y,
                                       base.orientation.z, base.orientation.w])
        origin = np.array([base.position.x, base.position.y, base.position.z])
        fruits = []
        for name, index in indices.items():
            if not name.startswith('harvest_apple_') or not name.endswith('::fruit'):
                continue
            pose = message.pose[index]
            fruit_rotation = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                                pose.orientation.z, pose.orientation.w])
            socket = np.array([pose.position.x, pose.position.y, pose.position.z])
            grasp = socket + fruit_rotation.apply([0., 0., .02])
            fruits.append({'model': name.split('::')[0],
                           'socket_to_branch_m': self.stem_offsets.get(name.split('::')[0], .04),
                           'grasp_in_base': rotation.inv().apply(grasp-origin).tolist(),
                           'stem_axis_in_base': rotation.inv().apply(fruit_rotation.apply([0., 0., 1.])).tolist(),
                           'socket_in_world': socket.tolist(),
                           'orientation_in_world': fruit_rotation.as_quat().tolist()})
        twist = message.twist[base_index]
        self.publisher.publish(String(data=json.dumps({
            'source': 'gazebo_object_pose_manipulation_baseline_not_visual_detection',
            'stamp': self.get_clock().now().nanoseconds/1e9, 'frame': 'base_link',
            'base_in_world': [*origin.tolist(), *rotation.as_quat().tolist()],
            'base_speed': float(np.linalg.norm([twist.linear.x, twist.linear.y, twist.linear.z])),
            'base_yaw_rate': twist.angular.z, 'fruits': sorted(fruits, key=lambda f: f['model'])})))


def main():
    rclpy.init()
    node = FruitObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
