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
            DeclareLaunchArgument(
                "use_lightweight_vehicle_model", default_value="false"
            ),
            DeclareLaunchArgument("navigation_delay", default_value="5.0"),
            DeclareLaunchArgument(
                "map", description="Absolute path to the real-vehicle Nav2 map YAML"
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("enable_can_output", default_value="false"),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value=LaunchConfiguration("enable_can_output"),
            ),
            DeclareLaunchArgument("chassis_driver", default_value="ackermann_can"),
            DeclareLaunchArgument("can_transport", default_value="zqwl_cdc"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "zqwl_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00"
                ),
            ),
            DeclareLaunchArgument("zqwl_channel", default_value="0"),
            DeclareLaunchArgument("zqwl_bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5C2C079857-if00"
                ),
            ),
            DeclareLaunchArgument(
                "command_input_topic", default_value="/nav2/cmd_vel"
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
                    "localization": "navsat",
                    "start_rtk": "true",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "rviz": LaunchConfiguration("rviz"),
                    "use_lightweight_vehicle_model": LaunchConfiguration(
                        "use_lightweight_vehicle_model"
                    ),
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "map": LaunchConfiguration("map"),
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                    "enable_can_output": LaunchConfiguration("enable_can_output"),
                    "enable_chassis_output": LaunchConfiguration(
                        "enable_chassis_output"
                    ),
                    "chassis_driver": LaunchConfiguration("chassis_driver"),
                    "can_transport": LaunchConfiguration("can_transport"),
                    "can_interface": LaunchConfiguration("can_interface"),
                    "zqwl_port": LaunchConfiguration("zqwl_port"),
                    "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                    "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "command_input_topic": LaunchConfiguration("command_input_topic"),
                }.items(),
            ),
        ]
    )
