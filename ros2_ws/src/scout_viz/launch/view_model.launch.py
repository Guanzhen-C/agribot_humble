import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viz_share = get_package_share_directory("scout_viz")
    desc_share = get_package_share_directory("scout_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument(
                "urdf_extras",
                default_value=os.path.join(desc_share, "urdf", "empty.urdf"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(viz_share, "rviz", "model.rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "description.launch.py")
                ),
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
