"""PC: physics, sensors and native CAN actuator only; no navigation nodes."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path.home() / '.cache/agribot/hil_apple'
    mission = json.loads((root / 'rdk/mission.json').read_text())
    description = Path(share('agribot_vehicle_description'))
    vehicle = json.loads((description/'generated/ackermann_current/vehicle_config.json').read_text())
    lidar = next(sensor for sensor in vehicle['sensors'] if sensor['id'] == 'lidar')
    imu = next(sensor for sensor in vehicle['sensors'] if sensor['id'] == 'imu')
    ackermann = Path(share('agribot_ackermann_mppi'))
    plugin_dirs = [str(Path(share(p)).parent.parent / 'lib') for p in (
        'agribot_ackermann_mppi', 'agribot_vehicle_description', 'velodyne_gazebo_plugins',
        'ros2_linkattacher')]
    compat = description.parent.parent / 'lib/agribot_gazebo_transport_compat'
    hil_plugins = Path.home()/'.local/state/agribot-hil-20261105/plugins'
    if not (hil_plugins/'libagribot_hil_gripper_hold.so').is_file():
        raise RuntimeError('Build the HIL transit hold plugin with scripts/build_sim_plugins.sh first')
    plugin_dirs.append(str(hil_plugins))
    ld = str(compat) + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    spawn = mission['spawn']
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('motion_authorized', default_value='false'),
        DeclareLaunchArgument('usb_serial', default_value='003000224856511520343739'),
        # Overrides are for isolated arm commissioning, not full mission acceptance.
        DeclareLaunchArgument('spawn_x', default_value=str(spawn[0])),
        DeclareLaunchArgument('spawn_y', default_value=str(spawn[1])),
        DeclareLaunchArgument('spawn_z', default_value=str(spawn[2])),
        DeclareLaunchArgument('spawn_yaw', default_value=str(spawn[3])),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11355'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', ':'.join(plugin_dirs) + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld),
        ExecuteProcess(cmd=['gzserver', '--verbose', str(root / 'world.sdf'),
                            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'hil_ackermann', '-file', str(root / 'vehicle.sdf'),
                        '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                        '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw')]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': (root / 'vehicle.urdf').read_text()}]),
        Node(package='image_transport', executable='republish', name='hil_camera_encoder',
             arguments=['raw', 'compressed'], parameters=[{'use_sim_time': True,
                         'out.format': 'png', 'out.png_level': 1}],
             remappings=[('in', '/camera/rgb/image_raw'),
                         ('out/compressed', '/hil/camera/image/compressed')], output='screen'),
        Node(package='agribot_vehicle_description', executable='simulation_lidar_time_adapter.py',
             parameters=[{'use_sim_time': True, 'input_topic': '/hil/lidar/self_filtered'}], output='screen'),
        Node(package='agribot_hil_apple', executable='gravity_reference',
             parameters=[{'use_sim_time': True, 'imu_frame': imu['frameId'],
                          'base_to_imu_rpy': imu['pose']['rpy']}], output='screen'),
        Node(package='agribot_vehicle_description', executable='simulation_vehicle_cloud_filter.py',
             name='hil_vehicle_cloud_filter', parameters=[{'use_sim_time': True,
             'input_topic': '/lidar/points', 'output_topic': '/hil/lidar/self_filtered',
             'lidar_frame': lidar['frameId'], 'lidar_xyz': lidar['pose']['xyz'],
             'lidar_rpy': lidar['pose']['rpy'], 'self_min': [-.19, -.40, -.2],
             'self_max': [.69, .40, .36]}], output='screen'),
        Node(package='agribot_ackermann_mppi', executable='sim_navsat_bridge.py', output='screen',
             parameters=[{'use_sim_time': True, 'odom_topic': '/hil/ground_truth',
                          'ground_truth_topic': '/hil/gnss_sensor_truth',
                          'origin_x': spawn[0], 'origin_y': spawn[1], 'origin_z': spawn[2],
                          'origin_yaw': spawn[3], 'base_to_antenna_m': mission['base_to_antenna'],
                          'publish_imu': False}]),
        Node(package='agribot_hil_apple', executable='gazebo_can_bridge', output='screen',
             parameters=[{'use_sim_time': True,
                          'motion_authorized': ParameterValue(LaunchConfiguration('motion_authorized'), value_type=bool),
                          'usb_serial': ParameterValue(['usb:', LaunchConfiguration('usb_serial')], value_type=str)}]),
        Node(package='agribot_hil_apple', executable='scene_bootstrap', output='screen'),
        Node(package='agribot_hil_apple', executable='arm_servo', output='screen'),
        Node(package='agribot_hil_apple', executable='fruit_observer',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
