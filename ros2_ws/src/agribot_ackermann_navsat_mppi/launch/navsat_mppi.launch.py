import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("agribot_ackermann_navsat_mppi")
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    ackermann_share = get_package_share_directory("agribot_ackermann_mppi")
    navigation_share = get_package_share_directory("scout_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_chassis_output", default_value="false"),
            DeclareLaunchArgument("navigation_delay", default_value="3.0"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    navigation_share, "maps", "orchard_v2_map6.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(
                    ackermann_share,
                    "config",
                    "nav2_params_ackermann_navsat_static.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "localization_params",
                default_value=os.path.join(
                    package_share, "config", "kf_gins_n300pro.yaml"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hardware_share, "launch", "sensors.launch.py")
                ),
                launch_arguments={
                    "start_lidar": "true",
                    "start_imu": "true",
                    "start_rtk": "true",
                    "rviz": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_sensors")),
            ),
            Node(
                package="agribot_rl_nav",
                executable="rtk_eskf_localization",
                name="rtk_eskf_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("localization_params"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="agribot_rl_nav",
                executable="navsat_pose_bridge.py",
                name="navsat_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": "/odometry/filtered_navsat",
                        "pose_topic": "/localization_pose",
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "tf_mode": "odom_to_base",
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "yaml_filename": LaunchConfiguration("map")}
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "autostart": autostart},
                    {"node_names": ["map_server"]},
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("navigation_delay"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "autostart": autostart,
                            "params_file": LaunchConfiguration("nav2_params"),
                            "odom_topic": "/odometry/filtered_navsat",
                            "default_nav_to_pose_bt_xml": os.path.join(
                                ackermann_share,
                                "behavior_trees",
                                "navigate_w_replanning_ackermann_no_spin.xml",
                            ),
                            "default_nav_through_poses_bt_xml": os.path.join(
                                ackermann_share,
                                "behavior_trees",
                                "navigate_through_poses_w_replanning_ackermann.xml",
                            ),
                        }.items(),
                    )
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="ackermann_collision_monitor",
                output="screen",
                parameters=[
                    os.path.join(package_share, "config", "collision_monitor.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "autostart": autostart},
                    {"node_names": ["ackermann_collision_monitor"]},
                ],
            ),
            Node(
                package="scout_control",
                executable="cmd_vel_mux.py",
                name="physical_cmd_vel_gate",
                output="screen",
                parameters=[
                    {
                        "config_file": os.path.join(
                            package_share, "config", "cmd_vel_mux.yaml"
                        ),
                        "output_topic": "/cmd_vel",
                        "publish_rate": 20.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_chassis_output")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(hardware_share, "rviz", "navigation.rviz")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
