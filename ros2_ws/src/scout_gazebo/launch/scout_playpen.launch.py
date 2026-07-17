import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("scout_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    viz_share = get_package_share_directory("scout_viz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value=os.path.join(gazebo_share, "worlds", "clearpath_playpen.world"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world_name"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "verbose": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "spawn_scout.launch.py")
                ),
                launch_arguments={
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "z": LaunchConfiguration("z"),
                    "yaw": LaunchConfiguration("yaw"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_joint_states": LaunchConfiguration("publish_joint_states"),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(viz_share, "rviz", "robot.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
