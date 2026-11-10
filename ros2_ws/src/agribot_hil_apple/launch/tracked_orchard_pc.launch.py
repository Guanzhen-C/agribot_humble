"""PC orchard physics/sensors; actuator commands still arrive only over CAN."""
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
    mission = json.loads((root/'rdk/mission.json').read_text())
    vehicle = json.loads((root/'vehicle_config.json').read_text())
    lidar = next(item for item in vehicle['sensors'] if item['id'] == 'lidar')
    imu = next(item for item in vehicle['sensors'] if item['id'] == 'imu')
    spawn = mission['spawn']
    kind = mission.get('implement_kind', 'sprayer')
    sim = {'use_sim_time': True}
    base = Path(share('agribot_hil_apple'))/'launch/tracked_work_pc.launch.py'
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base)), launch_arguments={
            'gui': LaunchConfiguration('gui'), 'scene_root': str(root),
            'implement_kind': kind, 'entity_name': 'hil_tracked_'+kind,
            'motion_authorized': LaunchConfiguration('motion_authorized'),
            'work_authorized': LaunchConfiguration('work_authorized'),
            'spawn_x': str(spawn[0]), 'spawn_y': str(spawn[1]),
            'spawn_z': str(spawn[2]), 'spawn_yaw': str(spawn[3])}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (root/'vehicle.urdf').read_text()}]),
        Node(package='image_transport', executable='republish', name='hil_camera_encoder',
             arguments=['raw', 'compressed'], parameters=[sim, {'out.format': 'png', 'out.png_level': 1}],
             remappings=[('in', '/camera/rgb/image_raw'), ('out/compressed', '/hil/camera/image/compressed')]),
        Node(package='agribot_vehicle_description', executable='simulation_vehicle_cloud_filter.py',
             name='hil_vehicle_cloud_filter', parameters=[sim, {
                 'input_topic': '/lidar/points', 'output_topic': '/hil/lidar/self_filtered',
                 'lidar_frame': lidar['frameId'], 'lidar_xyz': lidar['pose']['xyz'],
                 'lidar_rpy': lidar['pose']['rpy'], 'self_min': [-.55, -.40, -.25],
                 'self_max': [.67, .40, 1.10],
                 'extra_self_boxes': [-.48, -.26, .40, .28, .26, .90]}]),
        Node(package='agribot_vehicle_description', executable='simulation_lidar_time_adapter.py',
             parameters=[sim, {'input_topic': '/hil/lidar/self_filtered'}]),
        Node(package='agribot_hil_apple', executable='gravity_reference', parameters=[sim, {
            'imu_frame': imu['frameId'], 'base_to_imu_rpy': imu['pose']['rpy']}]),
        Node(package='agribot_ackermann_mppi', executable='sim_navsat_bridge.py', parameters=[sim, {
            'odom_topic': '/hil/ground_truth', 'ground_truth_topic': '/hil/gnss_sensor_truth',
            'origin_x': spawn[0], 'origin_y': spawn[1], 'origin_z': spawn[2], 'origin_yaw': spawn[3],
            'base_to_antenna_m': mission['base_to_antenna'], 'publish_imu': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_root', default_value=str(Path.home()/'.cache/agribot/isobus/tracked_orchard')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('motion_authorized', default_value='false'),
        DeclareLaunchArgument('work_authorized', default_value='false'),
        OpaqueFunction(function=scenario_nodes),
    ])
