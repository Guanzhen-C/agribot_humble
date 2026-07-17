import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("scout_control")
    localization_default = os.path.join(package_share, "config", "localization.yaml")
    twist_mux_default = os.path.join(package_share, "config", "twist_mux.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_ekf", default_value="true"),
            DeclareLaunchArgument("enable_twist_mux", default_value="true"),
            DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("localization_config", default_value=localization_default),
            DeclareLaunchArgument("twist_mux_config", default_value=twist_mux_default),
            DeclareLaunchArgument("ekf_publish_tf", default_value="true"),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("localization_config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"publish_tf": LaunchConfiguration("ekf_publish_tf")},
                ],
                condition=IfCondition(LaunchConfiguration("enable_ekf")),
            ),
            Node(
                package="scout_control",
                executable="cmd_vel_mux.py",
                name="twist_mux",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("twist_mux_config"),
                        "output_topic": LaunchConfiguration("cmd_vel_out_topic"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_twist_mux")),
            ),
        ]
    )
