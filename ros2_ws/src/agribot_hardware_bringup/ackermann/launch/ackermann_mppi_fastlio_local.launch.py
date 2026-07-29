import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="5.0"),
            DeclareLaunchArgument("enable_can_output", default_value="false"),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value="true",
            ),
            DeclareLaunchArgument("chassis_driver", default_value="ackermann_can"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5C2C079857-if00"
                ),
            ),
            DeclareLaunchArgument(
                "command_input_topic", default_value="/nav2/cmd_vel_safe"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share, "launch", "vehicle_autonomy.launch.py"
                    )
                ),
                launch_arguments={
                    "vehicle_type": "ackermann",
                    "controller": "mppi",
                    "localization": "fastlio",
                    "navigation_mode": "local",
                    "start_rtk": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": os.path.join(
                        hardware_share, "rviz", "navigation_local.rviz"
                    ),
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "map": "",
                    "fastlio_nav2_params": os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "nav2_params_ackermann_fastlio_local.yaml",
                    ),
                    "enable_can_output": LaunchConfiguration("enable_can_output"),
                    "enable_chassis_output": LaunchConfiguration(
                        "enable_chassis_output"
                    ),
                    "chassis_driver": LaunchConfiguration("chassis_driver"),
                    "can_interface": LaunchConfiguration("can_interface"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "command_input_topic": LaunchConfiguration("command_input_topic"),
                }.items(),
            ),
        ]
    )
