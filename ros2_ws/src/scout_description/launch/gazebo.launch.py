import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("scout_description")
    description_launch = os.path.join(package_share, "launch", "description.launch.py")
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument(
                "urdf_extras",
                default_value=os.path.join(package_share, "urdf", "empty.urdf"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
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
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "scout",
                    "-topic",
                    "robot_description",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                    "-Y",
                    yaw,
                ],
                output="screen",
            ),
        ]
    )
