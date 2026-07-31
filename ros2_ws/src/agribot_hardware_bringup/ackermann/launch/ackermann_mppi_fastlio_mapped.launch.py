import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    map_base = LaunchConfiguration("map_base")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base",
                description=(
                    "Absolute saved-map path without the .yaml extension"
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="[0.0, 0.0, 0.0]",
                description="Initial [map_x, map_y, map_yaw] rear-axle pose",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="8.0"),
            DeclareLaunchArgument("map_start_delay", default_value="5.0"),
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
                    "localization": "fastlio",
                    "navigation_mode": "localization",
                    "start_rtk": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": os.path.join(
                        hardware_share, "rviz", "navigation.rviz"
                    ),
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "map_start_delay": LaunchConfiguration("map_start_delay"),
                    "map": PythonExpression(["'", map_base, ".yaml'"]),
                    "initial_pose": LaunchConfiguration("initial_pose"),
                    "fastlio_nav2_params": os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "nav2_params_ackermann_fastlio_mapped.yaml",
                    ),
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
