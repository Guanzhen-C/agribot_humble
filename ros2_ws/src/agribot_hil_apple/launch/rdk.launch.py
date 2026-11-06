"""X5: real LIVO/RTK, Smac, MPPI and the existing native chassis driver."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path.home() / 'agribot_hil_ws/scenario'
    sim = {'use_sim_time': True}
    nodes = [
        Node(package='image_transport', executable='republish', name='hil_camera_decoder',
             arguments=['compressed', 'raw'], parameters=[sim],
             remappings=[('in/compressed', '/hil/camera/image/compressed'),
                         ('out', '/hil/camera/decoded')], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             name='hil_arm_move_group', parameters=[str(root / 'arm_moveit.yaml'), sim], output='screen'),
        Node(package='fast_livo', executable='fastlivo_mapping', name='fastlivo_mapping',
             parameters=[str(root / 'fastlivo.yaml'), str(root / 'camera.yaml'), sim], output='screen'),
        Node(package='agribot_hardware_bringup', executable='fastlio_odom_bridge.py',
             name='fastlio_odom_bridge', parameters=[str(root / 'odom_bridge.yaml'), sim], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='odom_to_livo',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'camera_init'], parameters=[sim]),
        Node(package='agribot_hardware_bringup', executable='fastlivo_rtk_fusion',
             name='fastlivo_rtk_fusion', parameters=[str(root / 'fusion.yaml'), sim,
             {'georeference_file': str(root / 'georeference.yaml'), 'map_file': '',
              'allow_missing_georeference': False}], output='screen'),
        Node(package='agribot_hardware_bringup', executable='differential_obstacle_height_filter',
             name='hil_obstacle_filter', parameters=[sim, {'input_topic': '/hil/lidar/self_filtered',
             'output_topic': '/configured/obstacle_points', 'base_frame': 'hil_base_level',
             'min_height': .08, 'max_height': .55, 'transform_timeout_sec': .1}], output='screen'),
        Node(package='agribot_hardware_bringup', executable='ackermann_chassis_can_node',
             parameters=[str(root / 'chassis.yaml'), sim], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[sim, {'yaml_filename': str(root / 'map.yaml')}], output='screen'),
    ]
    for package, name in (('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server')):
        nodes.append(Node(package=package, executable=name, name=name,
                          parameters=[str(root / 'nav2.yaml'), sim],
                          remappings=[('cmd_vel', '/hil/navigation/cmd_vel')], output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='hil_lifecycle_manager', parameters=[sim, {'autostart': True,
                      # X5 Smac table rebuilds can occupy the parameter callback
                      # for 22 s; allow that bounded operation without resetting Nav2.
                      'bond_timeout': 60., 'node_names': ['map_server', 'planner_server', 'controller_server']}],
                      output='screen'))
    return LaunchDescription(nodes)
