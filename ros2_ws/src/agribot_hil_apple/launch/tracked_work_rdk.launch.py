"""X5: one physical CAN owner, unchanged native tracked driver, ISOBUS client."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    state = Path.home()/'.local/state/agribot-isobus-hil-20261109'
    native = Path(share('agribot_hardware_bringup'))/'differential/config/chassis_can.yaml'
    mux = ExecuteProcess(cmd=[str(Path(get_package_prefix('agribot_hil_apple'))/
            'lib/agribot_hil_apple/isobus_can_mux'), '--output', LaunchConfiguration('trace')], output='screen')
    driver = Node(package='agribot_hardware_bringup', executable='differential_chassis_can_node',
             name='hil_native_tracked_driver', parameters=[str(native), {
                 'can_transport': 'socketcan', 'can_interface': 'vcan73',
                 'command_topic': '/hil/drive/command', 'odom_topic': '/hil/wheel/odometry',
                 'max_command_level': 800., 'max_linear_velocity': .42,
                 'max_angular_velocity': .7,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'require_localization_ready': ParameterValue(LaunchConfiguration('localization_guard'), value_type=bool),
                 'localization_ready_topic': '/fastlivo_rtk/ready',
                 'localization_ready_timeout_sec': 2.5}], output='screen')
    client = Node(package='agribot_hil_apple', executable='isobus_work_client', parameters=[{
            'endpoint_executable': LaunchConfiguration('endpoint'),
            'work_authorized': ParameterValue(LaunchConfiguration('work_authorized'), value_type=bool),
            'implement_kind': LaunchConfiguration('implement_kind')}], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('work_authorized', default_value='false'),
        DeclareLaunchArgument('implement_kind', default_value='sprayer'),
        DeclareLaunchArgument('localization_guard', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('trace', default_value=str(state/'can-mux.json')),
        DeclareLaunchArgument('endpoint', default_value=str(state/'build-host/hil_isobus_endpoint')),
        mux, driver, client,
        *[RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A critical physical HIL endpoint exited'))]))
          for process in (mux, driver, client)],
    ])
