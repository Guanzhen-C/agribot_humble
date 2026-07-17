import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_share = get_package_share_directory("scout_control")
    config_default = os.path.join(package_share, "config", "teleop_logitech.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("config_file", default_value=config_default),
            GroupAction(
                [
                    PushRosNamespace("joy_teleop"),
                    Node(
                        package="joy",
                        executable="joy_node",
                        name="joy_node",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("config_file"),
                            {"dev": LaunchConfiguration("joy_dev")},
                        ],
                    ),
                    Node(
                        package="teleop_twist_joy",
                        executable="teleop_node",
                        name="teleop_twist_joy",
                        output="screen",
                        parameters=[LaunchConfiguration("config_file")],
                    ),
                ]
            ),
        ]
    )
