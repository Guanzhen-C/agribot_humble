"""PC physical CAN plant, virtual sensors and articulated tractor scene."""
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('agribot_hil_hitch'))
    # Preparing a scene is explicit; never silently substitute a stale cache.
    scene = Path(__import__('os').environ['HITCH_SCENE_ROOT'])
    powertrain = json.loads((scene/'provenance.json').read_text()).get('powertrain_force_plant', False)
    server = ExecuteProcess(cmd=['gzserver', '--verbose', str(scene/'world.sdf'),
                                  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')
    bridge = ExecuteProcess(cmd=['/usr/bin/python3', str(package/'scripts/pc_bridge.py'),
        *(['--powertrain'] if powertrain else []), '--trace', LaunchConfiguration('can_trace')], output='screen')
    return LaunchDescription([DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('can_trace'),
        server, bridge,
        ExecuteProcess(cmd=['gzclient', '--verbose'], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{
            'use_sim_time': True, 'robot_description': (scene/'vehicle.urdf').read_text()}], output='screen'),
        Node(package='agribot_ackermann_mppi', executable='sim_navsat_bridge.py', name='sim_navsat_bridge',
             parameters=[str(scene/'rdk/gnss_sim.yaml'), {'use_sim_time': True}], output='screen'),
        *[RegisterEventHandler(OnProcessExit(target_action=item,
            on_exit=[EmitEvent(event=Shutdown(reason='A critical physical hitch plant process exited'))]))
          for item in (server, bridge)]])
