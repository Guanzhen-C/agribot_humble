"""Separate tracked spraying HIL commissioning pad; old apple launch is intact."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    root = Path.home()/'.cache/agribot/isobus/tracked_work_scene'
    state = Path.home()/'.local/state/agribot-isobus-hil-20261109'
    description = Path(share('agribot_vehicle_description'))
    plugins = [str(state/'plugins'), str(description.parent.parent/'lib'),
               str(Path(share('velodyne_gazebo_plugins')).parent.parent/'lib')]
    compat = description.parent.parent/'lib/agribot_gazebo_transport_compat'
    bridge = Node(package='agribot_hil_apple', executable='tracked_gazebo_can_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('native_bridge')), parameters=[{
            'use_sim_time': True,
            'motion_authorized': ParameterValue(LaunchConfiguration('motion_authorized'), value_type=bool),
            'implement_kind': LaunchConfiguration('implement_kind'),
            'work_authorized': ParameterValue(LaunchConfiguration('work_authorized'), value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('native_bridge', default_value='true'),
        DeclareLaunchArgument('motion_authorized', default_value='false'),
        DeclareLaunchArgument('work_authorized', default_value='false'),
        DeclareLaunchArgument('implement_kind', default_value='sprayer'),
        DeclareLaunchArgument('entity_name', default_value='hil_tracked_sprayer'),
        DeclareLaunchArgument('scene_root', default_value=str(root)),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_z', default_value='.24'),
        DeclareLaunchArgument('spawn_yaw', default_value='0'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11357'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', ':'.join(plugins)+':'+os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        SetEnvironmentVariable('LD_LIBRARY_PATH', str(compat)+':'+os.environ.get('LD_LIBRARY_PATH', '')),
        ExecuteProcess(cmd=['gzserver', '--verbose', PathJoinSubstitution([LaunchConfiguration('scene_root'), 'world.sdf']),
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen', arguments=[
            '-entity', LaunchConfiguration('entity_name'), '-file', PathJoinSubstitution([LaunchConfiguration('scene_root'), 'vehicle.sdf']),
            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw')]),
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Native CAN actuator bridge exited'))])),
    ])
