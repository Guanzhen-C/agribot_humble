import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("agribot_bev")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(
                    package_share, "config", "surround_bev.yaml"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("fuse_pointcloud", default_value="false"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/points"),
            Node(
                package="agribot_bev",
                executable="surround_bev",
                name="surround_bev",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "fuse_pointcloud": LaunchConfiguration("fuse_pointcloud"),
                        "pointcloud_topic": LaunchConfiguration(
                            "pointcloud_topic"
                        ),
                    },
                ],
            ),
        ]
    )
