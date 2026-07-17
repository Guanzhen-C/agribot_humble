import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_share = get_package_share_directory("scout_control")
    config_default = os.path.join(package_share, "config", "teleop_logitech.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("config_file", default_value=config_default),
            DeclareLaunchArgument("is_scout_control", default_value="true"),
            DeclareLaunchArgument("scale_linear", default_value="0.4"),
            DeclareLaunchArgument("scale_angular", default_value="0.6"),
            DeclareLaunchArgument("scale_linear_turbo", default_value="1.0"),
            DeclareLaunchArgument("scale_angular_turbo", default_value="1.2"),
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
                        condition=UnlessCondition(LaunchConfiguration("is_scout_control")),
                    ),
                    Node(
                        package="scout_control",
                        executable="joystick_translator.py",
                        name="joystick_translator",
                        output="screen",
                        parameters=[
                            {
                                "joystick_topic": "/joy_teleop/joy",
                                "vehicle_control_topic": "/scout_control",
                            }
                        ],
                        condition=IfCondition(LaunchConfiguration("is_scout_control")),
                    ),
                    Node(
                        package="scout_control",
                        executable="scout_control_translator.py",
                        name="scout_control_translator",
                        output="screen",
                        parameters=[
                            {
                                "vehicle_control_topic": "/scout_control",
                                "cmd_vel_topic": "/joy_teleop/cmd_vel",
                                "scale_linear": LaunchConfiguration("scale_linear"),
                                "scale_angular": LaunchConfiguration("scale_angular"),
                                "scale_linear_turbo": LaunchConfiguration("scale_linear_turbo"),
                                "scale_angular_turbo": LaunchConfiguration("scale_angular_turbo"),
                            }
                        ],
                        condition=IfCondition(LaunchConfiguration("is_scout_control")),
                    ),
                ]
            ),
        ]
    )
