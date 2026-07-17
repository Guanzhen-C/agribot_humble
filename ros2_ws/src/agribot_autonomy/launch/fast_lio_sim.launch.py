import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agribot_share = get_package_share_directory("agribot_autonomy")
    fast_lio_share = get_package_share_directory("fast_lio")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("is_simulation", default_value="false"),
            DeclareLaunchArgument(
                "fastlio_config_file",
                default_value=os.path.join(agribot_share, "config", "fast_lio_sim.yaml"),
            ),
            DeclareLaunchArgument("fastlio_visualize", default_value="false"),
            DeclareLaunchArgument("fastlio_input_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("fastlio_output_odom_topic", default_value="/fastlio/odometry"),
            DeclareLaunchArgument("fastlio_input_odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("fastlio_input_body_frame", default_value="body"),
            DeclareLaunchArgument("fastlio_output_odom_frame", default_value="odom"),
            DeclareLaunchArgument("fastlio_output_base_frame", default_value="base_link"),
            DeclareLaunchArgument("fastlio_stamp_with_current_time", default_value="false"),
            DeclareLaunchArgument("fastlio_publish_tf", default_value="true"),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlio_config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="imu_frame_bridge.py",
                name="imu_frame_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("is_simulation")),
            ),
            Node(
                package="agribot_autonomy",
                executable="fastlio_odom_bridge.py",
                name="fastlio_odom_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_odom_topic": LaunchConfiguration("fastlio_input_odom_topic"),
                        "output_odom_topic": LaunchConfiguration("fastlio_output_odom_topic"),
                        "input_odom_frame": LaunchConfiguration("fastlio_input_odom_frame"),
                        "input_body_frame": LaunchConfiguration("fastlio_input_body_frame"),
                        "output_odom_frame": LaunchConfiguration("fastlio_output_odom_frame"),
                        "output_base_frame": LaunchConfiguration("fastlio_output_base_frame"),
                        "is_simulation": LaunchConfiguration("is_simulation"),
                        "stamp_with_current_time": LaunchConfiguration(
                            "fastlio_stamp_with_current_time"
                        ),
                        "publish_tf": LaunchConfiguration("fastlio_publish_tf"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(fast_lio_share, "rviz", "fastlio.rviz")],
                condition=IfCondition(LaunchConfiguration("fastlio_visualize")),
                output="screen",
            ),
        ]
    )
