import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scout_navigation_share = get_package_share_directory("scout_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    scout_navigation_share, "config", "slam_toolbox_online_async.yaml"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_robot", default_value="false"),
            DeclareLaunchArgument("port_name", default_value="can1"),
            DeclareLaunchArgument("agilex_joystick", default_value="false"),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("odom_topic_name", default_value="odom"),
            DeclareLaunchArgument("pub_tf", default_value="false"),
            DeclareLaunchArgument("base_enabled", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_navigation_share, "launch", "include", "robot_bringup.launch.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_robot")),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "agilex_joystick": LaunchConfiguration("agilex_joystick"),
                    "simulated_robot": LaunchConfiguration("simulated_robot"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "odom_topic_name": LaunchConfiguration("odom_topic_name"),
                    "pub_tf": LaunchConfiguration("pub_tf"),
                    "base_enabled": LaunchConfiguration("base_enabled"),
                    "control_enabled": LaunchConfiguration("control_enabled"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_navigation_share, "launch", "include", "slam_toolbox.launch.py"
                    )
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
