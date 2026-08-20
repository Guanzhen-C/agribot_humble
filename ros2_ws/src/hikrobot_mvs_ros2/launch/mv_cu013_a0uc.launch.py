import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hikrobot_mvs_ros2")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(share, "config", "mv_cu013_a0uc.yaml"),
            ),
            DeclareLaunchArgument("serial_number", default_value="DB0447659"),
            DeclareLaunchArgument("trigger_enable", default_value="false"),
            DeclareLaunchArgument("frame_rate", default_value="10.0"),
            Node(
                package="hikrobot_mvs_ros2",
                executable="hikrobot_mvs_camera_node",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "serial_number": LaunchConfiguration("serial_number"),
                        "trigger_enable": LaunchConfiguration("trigger_enable"),
                        "acquisition_frame_rate": LaunchConfiguration("frame_rate"),
                    },
                ],
            ),
        ]
    )
