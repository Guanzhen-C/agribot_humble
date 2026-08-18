from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("agribot_mobile_app")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "mobile_gateway.yaml"),
            ),
            DeclareLaunchArgument(
                "runtime_profiles",
                default_value=os.path.join(share, "config", "runtime_profiles.yaml"),
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Keep the vehicle ROS graph on this computer",
            ),
            Node(
                package="agribot_mobile_app",
                executable="mobile_gateway",
                name="mobile_gateway",
                output="screen",
                additional_env={
                    "ROS_LOCALHOST_ONLY": LaunchConfiguration("ros_localhost_only"),
                },
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "runtime_profiles": LaunchConfiguration("runtime_profiles"),
                    },
                ],
            ),
        ]
    )
