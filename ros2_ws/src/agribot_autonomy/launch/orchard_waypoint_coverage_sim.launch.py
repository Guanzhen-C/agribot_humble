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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_xvfb", default_value="false"),
            DeclareLaunchArgument("laser_3d_enabled", default_value="true"),
            DeclareLaunchArgument("laser_3d_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="/points"),
            DeclareLaunchArgument("laser_3d_update_rate", default_value="5"),
            DeclareLaunchArgument("laser_3d_horizontal_samples", default_value="360"),
            DeclareLaunchArgument("laser_3d_vertical_samples", default_value="16"),
            DeclareLaunchArgument("laser_3d_min_range", default_value="0.3"),
            DeclareLaunchArgument("laser_3d_max_range", default_value="25.0"),
            DeclareLaunchArgument("launch_kiss_icp", default_value="false"),
            DeclareLaunchArgument("launch_fast_lio", default_value="true"),
            DeclareLaunchArgument("use_kiss_localization", default_value="false"),
            DeclareLaunchArgument("use_fastlio_localization", default_value="true"),
            DeclareLaunchArgument("use_ground_truth_localization", default_value="false"),
            DeclareLaunchArgument("use_static_map", default_value="false"),
            DeclareLaunchArgument("nav_odom_topic", default_value="/fastlio/odometry"),
            DeclareLaunchArgument("kiss_visualize", default_value="false"),
            DeclareLaunchArgument("fastlio_visualize", default_value="false"),
            DeclareLaunchArgument(
                "kiss_config_file",
                default_value=os.path.join(
                    agribot_share, "config", "kiss_icp_sim.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_config_file",
                default_value=os.path.join(agribot_share, "config", "fast_lio_sim.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(agribot_share, "rviz", "robot_map.rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_gazebo_share, "launch", "scout_orchard_world.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": "false",
                    "headless": LaunchConfiguration("headless"),
                    "use_xvfb": LaunchConfiguration("use_xvfb"),
                    "use_sim_time": "true",
                    "x": "2.0",
                    "y": "36.0",
                    "yaw": "0.0",
                    "laser_3d_enabled": LaunchConfiguration("laser_3d_enabled"),
                    "laser_3d_xyz": LaunchConfiguration("laser_3d_xyz"),
                    "laser_3d_rpy": LaunchConfiguration("laser_3d_rpy"),
                    "laser_3d_topic": LaunchConfiguration("laser_3d_topic"),
                    "laser_3d_update_rate": LaunchConfiguration("laser_3d_update_rate"),
                    "laser_3d_horizontal_samples": LaunchConfiguration(
                        "laser_3d_horizontal_samples"
                    ),
                    "laser_3d_vertical_samples": LaunchConfiguration(
                        "laser_3d_vertical_samples"
                    ),
                    "laser_3d_min_range": LaunchConfiguration("laser_3d_min_range"),
                    "laser_3d_max_range": LaunchConfiguration("laser_3d_max_range"),
                    "publish_odom_tf": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(agribot_share, "launch", "kiss_icp_sim.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "use_pointcloud_relay": "false",
                    "input_pointcloud_topic": LaunchConfiguration("laser_3d_topic"),
                    "kiss_input_topic": LaunchConfiguration("laser_3d_topic"),
                    "kiss_base_frame": "base_link",
                    "kiss_odom_frame": "odom",
                    "kiss_visualize": LaunchConfiguration("kiss_visualize"),
                    "kiss_config_file": LaunchConfiguration("kiss_config_file"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_kiss_icp")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(agribot_share, "launch", "fast_lio_sim.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "fastlio_config_file": LaunchConfiguration("fastlio_config_file"),
                    "fastlio_visualize": LaunchConfiguration("fastlio_visualize"),
                    "fastlio_output_odom_topic": LaunchConfiguration("nav_odom_topic"),
                    "fastlio_output_odom_frame": "odom",
                    "fastlio_output_base_frame": "base_link",
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_fast_lio")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        agribot_share, "launch", "orchard_waypoint_coverage.launch.py"
                    )
                ),
                launch_arguments={
                    "use_ground_truth_localization": LaunchConfiguration(
                        "use_ground_truth_localization"
                    ),
                    "use_kiss_localization": LaunchConfiguration("use_kiss_localization"),
                    "use_fastlio_localization": LaunchConfiguration(
                        "use_fastlio_localization"
                    ),
                    "use_static_map": LaunchConfiguration("use_static_map"),
                    "nav_odom_topic": LaunchConfiguration("nav_odom_topic"),
                    "use_sim_time": "true",
                    "start_control": "true",
                    "control_enable_ekf": "false",
                    "control_enable_twist_mux": "true",
                    "waypoint_file": os.path.join(
                        agribot_share, "config", "orchard_waypoints_default_start.yaml"
                    ),
                    "initial_pose_x": "2.0",
                    "initial_pose_y": "36.0",
                    "initial_pose_z": "0.146336",
                    "initial_pose_yaw": "0.0",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
