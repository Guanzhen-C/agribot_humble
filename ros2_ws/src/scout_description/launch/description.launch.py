import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("scout_description")
    xacro_exec = os.path.join(get_package_prefix("xacro"), "bin", "xacro")
    description_file = os.path.join(package_share, "urdf", "scout.urdf.xacro")
    default_extras = os.path.join(package_share, "urdf", "empty.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    robot_description = ParameterValue(
        Command(
            [
                xacro_exec,
                " ",
                description_file,
                " ",
                "robot_namespace:=",
                LaunchConfiguration("robot_namespace"),
                " ",
                "urdf_extras:=",
                LaunchConfiguration("urdf_extras"),
                " ",
                "laser_enabled:=",
                LaunchConfiguration("laser_enabled"),
                " ",
                "laser_3d_enabled:=",
                LaunchConfiguration("laser_3d_enabled"),
                " ",
                "laser_3d_xyz:=",
                '"',
                LaunchConfiguration("laser_3d_xyz"),
                '"',
                " ",
                "laser_3d_rpy:=",
                '"',
                LaunchConfiguration("laser_3d_rpy"),
                '"',
                " ",
                "laser_3d_topic:=",
                LaunchConfiguration("laser_3d_topic"),
                " ",
                "laser_3d_update_rate:=",
                LaunchConfiguration("laser_3d_update_rate"),
                " ",
                "laser_3d_horizontal_samples:=",
                LaunchConfiguration("laser_3d_horizontal_samples"),
                " ",
                "laser_3d_vertical_samples:=",
                LaunchConfiguration("laser_3d_vertical_samples"),
                " ",
                "laser_3d_min_range:=",
                LaunchConfiguration("laser_3d_min_range"),
                " ",
                "laser_3d_max_range:=",
                LaunchConfiguration("laser_3d_max_range"),
                " ",
                "publish_odom_tf:=",
                LaunchConfiguration("publish_odom_tf"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument("urdf_extras", default_value=default_extras),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_robot_state", default_value="true"),
            DeclareLaunchArgument("laser_enabled", default_value="true"),
            DeclareLaunchArgument("laser_3d_enabled", default_value="false"),
            DeclareLaunchArgument("laser_3d_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="points"),
            DeclareLaunchArgument("laser_3d_update_rate", default_value="10"),
            DeclareLaunchArgument("laser_3d_horizontal_samples", default_value="720"),
            DeclareLaunchArgument("laser_3d_vertical_samples", default_value="16"),
            DeclareLaunchArgument("laser_3d_min_range", default_value="0.3"),
            DeclareLaunchArgument("laser_3d_max_range", default_value="25.0"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(publish_robot_state),
            ),
        ]
    )
