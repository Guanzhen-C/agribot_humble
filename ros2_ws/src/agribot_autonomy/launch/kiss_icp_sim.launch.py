import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kiss_icp_share = get_package_share_directory("kiss_icp")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_pointcloud_relay", default_value="true"),
            DeclareLaunchArgument("input_pointcloud_topic", default_value="/points"),
            DeclareLaunchArgument("kiss_input_topic", default_value="/kiss/points"),
            DeclareLaunchArgument("kiss_pointcloud_topic", default_value="/kiss/points"),
            DeclareLaunchArgument("kiss_pointcloud_frame", default_value="kiss_lidar"),
            DeclareLaunchArgument("kiss_base_frame", default_value=""),
            DeclareLaunchArgument("kiss_odom_frame", default_value="kiss_odom"),
            DeclareLaunchArgument("kiss_visualize", default_value="false"),
            DeclareLaunchArgument(
                "kiss_config_file",
                default_value=os.path.join(kiss_icp_share, "config", "config.yaml"),
            ),
            DeclareLaunchArgument("kiss_position_covariance", default_value="0.1"),
            DeclareLaunchArgument("kiss_orientation_covariance", default_value="0.1"),
            Node(
                package="agribot_autonomy",
                executable="pointcloud_frame_relay.py",
                name="kiss_pointcloud_frame_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_pointcloud_topic"),
                        "output_topic": LaunchConfiguration("kiss_pointcloud_topic"),
                        "output_frame_id": LaunchConfiguration("kiss_pointcloud_frame"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_pointcloud_relay")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(kiss_icp_share, "launch", "odometry.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "topic": LaunchConfiguration("kiss_input_topic"),
                    "visualize": LaunchConfiguration("kiss_visualize"),
                    "base_frame": LaunchConfiguration("kiss_base_frame"),
                    "lidar_odom_frame": LaunchConfiguration("kiss_odom_frame"),
                    "publish_odom_tf": "true",
                    "invert_odom_tf": "false",
                    "position_covariance": LaunchConfiguration("kiss_position_covariance"),
                    "orientation_covariance": LaunchConfiguration(
                        "kiss_orientation_covariance"
                    ),
                    "config_file": LaunchConfiguration("kiss_config_file"),
                }.items(),
            ),
        ]
    )
