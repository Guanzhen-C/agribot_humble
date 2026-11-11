"""X5 navigation and new bench protocol; no real vehicle driver is started."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path.home()/'agribot_hitch_hil_ws'
    scripts = root/'src/agribot_hil_hitch/scripts'
    scenario = root/'scenario'
    mux = ExecuteProcess(cmd=['/usr/bin/python3', str(scripts/'usb_mux.py'),
                              '--trace', LaunchConfiguration('can_trace')], output='screen')
    client = ExecuteProcess(cmd=['/usr/bin/python3', str(scripts/'rdk_client.py'),
        '--endpoint', str(root/'build-protocol/hil_hitch_endpoint'), '--trace', LaunchConfiguration('client_trace'),
        '--ros-args', '-p', 'use_sim_time:=true', '-p', ['powertrain_enabled:=', LaunchConfiguration('powertrain_enabled')]], output='screen')
    sim = {'use_sim_time': True}
    nodes = [Node(package='agribot_hardware_bringup', executable='rtk_eskf_localization',
                  name='rtk_eskf_localization', parameters=[str(scenario/'kf_gins.yaml'), sim], output='screen'),
             Node(package='agribot_hardware_bringup', executable='navsat_pose_bridge.py',
                  parameters=[sim, {'tf_mode': 'odom_to_base', 'odom_topic': '/odometry/filtered_navsat'}], output='screen'),
             Node(package='agribot_hardware_bringup', executable='differential_obstacle_height_filter',
                  name='hitch_obstacle_height_filter', parameters=[sim, {'input_topic': '/lidar/points',
                    'output_topic': '/configured/obstacle_points', 'base_frame': 'base_link',
                    'min_height': -.566006673086, 'max_height': 2.28489651996853}], output='screen'),
             Node(package='nav2_map_server', executable='map_server', name='map_server',
                  parameters=[sim, {'yaml_filename': str(scenario/'map.yaml')}], output='screen')]
    for package, name in (('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server')):
        nodes.append(Node(package=package, executable=name, name=name,
                          parameters=[str(scenario/'nav2.yaml'), sim], remappings=[('cmd_vel', '/nav2/cmd_vel')], output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='hitch_lifecycle', parameters=[sim, {'autostart': True, 'bond_timeout': 60.,
          'node_names': ['map_server', 'planner_server', 'controller_server']}], output='screen'))
    return LaunchDescription([DeclareLaunchArgument('can_trace'), DeclareLaunchArgument('client_trace'),
        DeclareLaunchArgument('powertrain_enabled', default_value='false'), mux, client,
        *nodes, *[RegisterEventHandler(OnProcessExit(target_action=item,
          on_exit=[EmitEvent(event=Shutdown(reason='A critical hitch HIL process exited'))])) for item in (mux, client)]])
