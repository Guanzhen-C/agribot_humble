"""Restart navigation without resetting the running LIVO/RTK estimator."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path.home() / 'agribot_hil_ws/scenario'
    sim = {'use_sim_time': True}
    nodes = [Node(package='nav2_map_server', executable='map_server', name='map_server',
                  parameters=[sim, {'yaml_filename': str(root / 'map.yaml')}], output='screen')]
    for package, name in (('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server')):
        nodes.append(Node(package=package, executable=name, name=name,
                          parameters=[str(root / 'nav2.yaml'), sim],
                          remappings=[('cmd_vel', '/hil/navigation/cmd_vel')], output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='hil_lifecycle_manager', parameters=[sim, {'autostart': True,
                      'bond_timeout': 10., 'node_names': ['map_server', 'planner_server', 'controller_server']}],
                      output='screen'))
    return LaunchDescription(nodes)
