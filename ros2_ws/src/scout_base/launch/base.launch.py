from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="can0"),
            DeclareLaunchArgument("is_scout_mini", default_value="false"),
            DeclareLaunchArgument("is_scout_omni", default_value="false"),
            DeclareLaunchArgument("agilex_joystick", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value=""),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("max_linear_velocity", default_value="0.8"),
            DeclareLaunchArgument("max_angular_velocity", default_value="0.65"),
            DeclareLaunchArgument("odom_topic_name", default_value="odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("pub_tf", default_value="true"),
            Node(
                package="scout_base",
                executable="scout_base_node",
                output="screen",
                parameters=[
                    {
                        "port_name": LaunchConfiguration("port_name"),
                        "is_scout_mini": LaunchConfiguration("is_scout_mini"),
                        "is_scout_omni": LaunchConfiguration("is_scout_omni"),
                        "agilex_joystick": LaunchConfiguration("agilex_joystick"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "simulated_robot": LaunchConfiguration("simulated_robot"),
                        "command_timeout_sec": LaunchConfiguration("command_timeout_sec"),
                        "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
                        "max_angular_velocity": LaunchConfiguration("max_angular_velocity"),
                        "odom_topic_name": LaunchConfiguration("odom_topic_name"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "pub_tf": LaunchConfiguration("pub_tf"),
                    }
                ],
            ),
        ]
    )
