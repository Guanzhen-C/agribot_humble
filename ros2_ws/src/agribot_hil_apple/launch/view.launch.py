"""Read-only full orchard map, current/saved paths and the existing RViz view."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('report'),
        Node(package='nav2_map_server', executable='map_server', name='hil_full_map_server',
             parameters=[sim, {'yaml_filename': str(Path(share('scout_navigation'))/'maps/orchard_v2_map6.yaml'),
                                'topic_name': '/hil/visualization/full_map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='hil_full_map_manager',
             parameters=[sim, {'autostart': True, 'node_names': ['hil_full_map_server']}]),
        Node(package='agribot_hil_apple', executable='mission_view',
             arguments=[LaunchConfiguration('report')], parameters=[sim]),
        Node(package='rviz2', executable='rviz2', parameters=[sim],
             arguments=['-d', str(Path(share('agribot_hil_apple'))/'config/hil_navigation.rviz')]),
    ])
