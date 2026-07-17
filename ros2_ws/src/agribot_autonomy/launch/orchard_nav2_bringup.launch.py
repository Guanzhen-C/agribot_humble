import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    agribot_share = get_package_share_directory("agribot_autonomy")
    scout_desc_share = get_package_share_directory("scout_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")

    map_default = os.path.join(agribot_share, "maps", "orchard_v2_map6.yaml")
    params_default = os.path.join(agribot_share, "config", "nav2_params.yaml")
    waypoint_default = os.path.join(agribot_share, "config", "orchard_waypoints_inrow.yaml")
    rviz_default = os.path.join(agribot_share, "rviz", "robot_map.rviz")
    description_file = os.path.join(scout_desc_share, "urdf", "scout.urdf")
    robot_description = Path(description_file).read_text(encoding="utf-8")
    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                "default_nav_to_pose_bt_xml"
            )
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_default),
            DeclareLaunchArgument("params_file", default_value=params_default),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    nav2_bt_navigator_share,
                    "behavior_trees",
                    "navigate_to_pose_w_replanning_and_recovery.xml",
                ),
            ),
            DeclareLaunchArgument("waypoint_file", default_value=waypoint_default),
            DeclareLaunchArgument("port_name", default_value="can0"),
            DeclareLaunchArgument("simulated_robot", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("run_waypoints", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("scout_base"), "launch", "base.launch.py")
                ),
                launch_arguments={
                    "port_name": LaunchConfiguration("port_name"),
                    "simulated_robot": LaunchConfiguration("simulated_robot"),
                    "pub_tf": "true",
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_simulator",
                arguments=[
                    LaunchConfiguration("initial_pose_x"),
                    LaunchConfiguration("initial_pose_y"),
                    "0.0",
                    LaunchConfiguration("initial_pose_yaw"),
                    "0.0",
                    "0.0",
                    "map",
                    "odom",
                ],
                condition=IfCondition(LaunchConfiguration("simulated_robot")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": configured_nav2_params,
                }.items(),
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                output="screen",
                parameters=[
                    {
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "startup_delay": 5.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                    }
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="snake_waypoint_runner.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "waypoint_file": LaunchConfiguration("waypoint_file"),
                        "startup_delay": 8.0,
                        "navigation_mode": "follow_path",
                        "frame_id": "map",
                        "action_name": "navigate_to_pose",
                        "path_action_name": "follow_path",
                        "controller_id": "FollowPath",
                        "path_step": 0.5,
                        "stop_on_failure": True,
                        "retries_per_waypoint": 0,
                        "proximity_advance_enabled": False,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("run_waypoints")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
