"""Refresh only task fruit models; preserve the vehicle and live camera view."""
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity


def main():
    rclpy.init()
    node = Node('hil_refresh_fruit_visuals')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Gazebo service is not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Gazebo service timed out')
        return future.result()

    try:
        delete = node.create_client(DeleteEntity, '/delete_entity')
        spawn = node.create_client(SpawnEntity, '/spawn_entity')
        root = ET.parse(Path.home() / '.cache/agribot/hil_apple/world.sdf')
        for model in root.findall('./world/model'):
            name = model.attrib['name']
            if not name.startswith('harvest_apple_'):
                continue
            pose = list(map(float, model.find('pose').text.split()))
            model.remove(model.find('pose'))
            request = DeleteEntity.Request()
            request.name = name
            result = call(delete, request)
            if not result.success and 'does not exist' not in result.status_message:
                raise RuntimeError(result.status_message)
            sdf = ET.Element('sdf', version='1.7')
            sdf.append(model)
            request = SpawnEntity.Request()
            request.name = name
            request.xml = ET.tostring(sdf, encoding='unicode')
            request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = pose[:3]
            request.initial_pose.orientation.w = 1.
            result = call(spawn, request)
            if not result.success:
                raise RuntimeError(result.status_message)
            print(name, result.status_message, flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
