import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(package_path, 'config'),
            description='Directory containing the FAST-LIO parameter file',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='mid360.yaml',
            description='FAST-LIO parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock',
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            emulate_tty=True,
            prefix='gdb -ex run --args',
            parameters=[
                PathJoinSubstitution([config_path, config_file]),
                {'use_sim_time': use_sim_time},
            ],
            on_exit=Shutdown(reason='FAST-LIO GDB session ended'),
        ),
    ])
