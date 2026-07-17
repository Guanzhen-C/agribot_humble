import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _maybe_include_ydlidar(context):
    enabled = LaunchConfiguration("ydlidar_enabled").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []

    try:
        ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "scout_bringup: ydlidar_enabled is true but package "
                    "'ydlidar_ros2_driver' is not installed, skipping lidar bringup."
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_share, "launch", "ydlidar_launch.py")
            )
        )
    ]


def generate_launch_description():
    base_share = get_package_share_directory("scout_base")
    bringup_share = get_package_share_directory("scout_bringup")
    control_share = get_package_share_directory("scout_control")
    desc_share = get_package_share_directory("scout_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="can1"),
            DeclareLaunchArgument("agilex_joystick", default_value="false"),
            DeclareLaunchArgument("base_enabled", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument(
                "realsense_enabled",
                default_value=EnvironmentVariable(
                    "SCOUT_REALSENSE_ENABLED", default_value="true"
                ),
            ),
            DeclareLaunchArgument(
                "ydlidar_enabled",
                default_value=EnvironmentVariable(
                    "SCOUT_YDLIDAR_ENABLED", default_value="false"
                ),
            ),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("odom_topic_name", default_value="odom"),
            DeclareLaunchArgument("pub_tf", default_value="false"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_map_to_odom", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(base_share, "launch", "base.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("base_enabled")),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "agilex_joystick": LaunchConfiguration("agilex_joystick"),
                    "simulated_robot": LaunchConfiguration("simulated_robot"),
                    "odom_topic_name": LaunchConfiguration("odom_topic_name"),
                    "pub_tf": LaunchConfiguration("pub_tf"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_share, "launch", "control.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("control_enabled")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "cmd_vel_out_topic": "/cmd_vel",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "include", "realsense.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("realsense_enabled")),
                launch_arguments={
                    "camera": "camera",
                    "depth_profile": "640,480,15",
                    "color_profile": "640,480,15",
                    "infra_profile": "640,480,15",
                    "fisheye_profile": "640,480,15",
                }.items(),
            ),
            OpaqueFunction(function=_maybe_include_ydlidar),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "description.launch.py")
                ),
                launch_arguments={
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_robot_state": "true",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=IfCondition(LaunchConfiguration("publish_map_to_odom")),
                output="screen",
            ),
        ]
    )
