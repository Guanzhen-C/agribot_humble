import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agribot_share = get_package_share_directory("agribot_autonomy")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("initial_pose_x", default_value="2.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="36.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="/points"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_gazebo_share, "launch", "scout_orchard_world.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "x": LaunchConfiguration("initial_pose_x"),
                    "y": LaunchConfiguration("initial_pose_y"),
                    "yaw": LaunchConfiguration("initial_pose_yaw"),
                    "laser_3d_enabled": "true",
                    "laser_3d_topic": LaunchConfiguration("laser_3d_topic"),
                    "publish_odom_tf": "false",
                    "publish_simulated_odom": "false",
                    "publish_ground_truth": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(agribot_share, "rviz", "robot_map_global_plan_only.rviz"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
