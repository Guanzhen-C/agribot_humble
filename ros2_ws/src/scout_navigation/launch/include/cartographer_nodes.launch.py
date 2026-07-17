import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("scout_navigation"), "config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    "scout.lua",
                ],
                remappings=[
                    ("scan", LaunchConfiguration("scan_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                    ("odom", LaunchConfiguration("odom_topic")),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                arguments=["-resolution", LaunchConfiguration("resolution")],
            ),
        ]
    )
