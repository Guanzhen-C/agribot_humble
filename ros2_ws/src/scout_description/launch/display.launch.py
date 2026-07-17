import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("scout_description")
    rviz_default = os.path.join(package_share, "rviz", "model_display.rviz")
    description_launch = os.path.join(package_share, "launch", "description.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument(
                "urdf_extras",
                default_value=os.path.join(package_share, "urdf", "empty.urdf"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                    "urdf_extras": LaunchConfiguration("urdf_extras"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_robot_state": "true",
                }.items(),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
