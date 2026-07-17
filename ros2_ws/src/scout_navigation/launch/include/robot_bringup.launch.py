import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scout_bringup_share = get_package_share_directory("scout_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="can1"),
            DeclareLaunchArgument("agilex_joystick", default_value="false"),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("odom_topic_name", default_value="odom"),
            DeclareLaunchArgument("pub_tf", default_value="false"),
            DeclareLaunchArgument("base_enabled", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_bringup_share, "launch", "start.launch.py")
                ),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "agilex_joystick": LaunchConfiguration("agilex_joystick"),
                    "base_enabled": LaunchConfiguration("base_enabled"),
                    "control_enabled": LaunchConfiguration("control_enabled"),
                    "simulated_robot": LaunchConfiguration("simulated_robot"),
                    "odom_topic_name": LaunchConfiguration("odom_topic_name"),
                    "pub_tf": LaunchConfiguration("pub_tf"),
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_map_to_odom": "false",
                }.items(),
            ),
        ]
    )
