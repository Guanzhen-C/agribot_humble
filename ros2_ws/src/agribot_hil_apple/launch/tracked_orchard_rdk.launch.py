"""X5 real localization/navigation algorithms plus native drive/ISOBUS owner."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def scenario_nodes(context):
    root = Path(LaunchConfiguration('scene_root').perform(context))
    mission = json.loads((root/'mission.json').read_text())
    sim = {'use_sim_time': True}
    base = Path(share('agribot_hil_apple'))/'launch/tracked_work_rdk.launch.py'
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base)), launch_arguments={
            'work_authorized': LaunchConfiguration('work_authorized'),
            'implement_kind': mission.get('implement_kind', 'sprayer'),
            'trace': LaunchConfiguration('trace'), 'use_sim_time': 'true', 'localization_guard': 'true'}.items()),
        Node(package='agribot_hil_apple', executable='tracked_drive_guard', output='screen'),
        Node(package='image_transport', executable='republish', name='hil_camera_decoder',
             arguments=['compressed', 'raw'], parameters=[sim], remappings=[
                 ('in/compressed', '/hil/camera/image/compressed'), ('out', '/hil/camera/decoded')]),
        Node(package='fast_livo', executable='fastlivo_mapping', name='fastlivo_mapping',
             parameters=[str(root/'fastlivo.yaml'), str(root/'camera.yaml'), sim], output='screen'),
        Node(package='agribot_hardware_bringup', executable='fastlio_odom_bridge.py', name='fastlio_odom_bridge',
             parameters=[str(root/'odom_bridge.yaml'), sim], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='odom_to_livo',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'camera_init'], parameters=[sim]),
        Node(package='agribot_hardware_bringup', executable='fastlivo_rtk_fusion', name='fastlivo_rtk_fusion',
             parameters=[str(root/'fusion.yaml'), sim, {'georeference_file': str(root/'georeference.yaml'),
                 'map_file': '', 'allow_missing_georeference': False}], output='screen'),
        Node(package='agribot_hardware_bringup', executable='differential_obstacle_height_filter',
             name='hil_obstacle_filter', parameters=[sim, {'input_topic': '/hil/lidar/self_filtered',
                 'output_topic': '/configured/obstacle_points', 'base_frame': 'hil_base_level',
                 'min_height': .08, 'max_height': .55, 'transform_timeout_sec': .1}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[sim, {'yaml_filename': str(root/'map.yaml')}]),
    ]
    for package, name in (('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server')):
        nodes.append(Node(package=package, executable=name, name=name,
            parameters=[str(root/'nav2.yaml'), sim], remappings=[('cmd_vel', '/hil/navigation/cmd_vel')], output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='hil_lifecycle_manager', parameters=[sim, {'autostart': True, 'bond_timeout': 60.,
            'node_names': ['map_server', 'planner_server', 'controller_server']}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_root', default_value=str(Path.home()/'agribot_isobus_hil_ws/scenario_orchard')),
        DeclareLaunchArgument('work_authorized', default_value='false'),
        DeclareLaunchArgument('trace', default_value=str(Path.home()/'.local/state/agribot-isobus-hil-20261109/orchard-can.json')),
        OpaqueFunction(function=scenario_nodes),
    ])
