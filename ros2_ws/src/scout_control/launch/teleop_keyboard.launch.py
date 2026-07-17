from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/joy_teleop/cmd_vel"),
            Node(
                package="scout_control",
                executable="teleop_keyboard.py",
                name="teleop_keyboard",
                output="screen",
                parameters=[{"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")}],
            ),
        ]
    )
