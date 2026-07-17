import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    autonomy_share = get_package_share_directory("agribot_autonomy")
    control_share = get_package_share_directory("scout_control")
    gazebo_share = get_package_share_directory("scout_gazebo")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")

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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_xvfb", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(autonomy_share, "rviz", "robot_map.rviz"),
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        ".local",
                        "share",
                        "agribot",
                        "models",
                        "depth_bc",
                        "latest",
                        "policy.ts",
                    ]
                ),
            ),
            DeclareLaunchArgument("seq_len", default_value="8"),
            DeclareLaunchArgument("chunk_replan_interval", default_value="2"),
            DeclareLaunchArgument("debug_log", default_value="false"),
            DeclareLaunchArgument("stop_without_plan", default_value="true"),
            DeclareLaunchArgument("map", default_value=os.path.join(autonomy_share, "maps", "orchard_v2_map6.yaml")),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(autonomy_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    nav2_bt_navigator_share,
                    "behavior_trees",
                    "navigate_to_pose_w_replanning_and_recovery.xml",
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "scout_orchard_world.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": "false",
                    "headless": LaunchConfiguration("headless"),
                    "use_xvfb": LaunchConfiguration("use_xvfb"),
                    "x": "2.0",
                    "y": "36.0",
                    "yaw": "0.0",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_share, "launch", "control.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "enable_ekf": "false",
                    "enable_twist_mux": "true",
                    "cmd_vel_out_topic": "/cmd_vel",
                }.items(),
            ),
            GroupAction(
                actions=[
                    SetRemap(src="cmd_vel", dst="/nav2/cmd_vel"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
                        ),
                        launch_arguments={
                            "map": LaunchConfiguration("map"),
                            "use_sim_time": "true",
                            "params_file": configured_nav2_params,
                        }.items(),
                    ),
                ]
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_localization.py",
                name="ground_truth_localization",
                output="screen",
                parameters=[
                    {
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "pose_topic": "/amcl_pose",
                    }
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_printer.py",
                name="ground_truth_printer",
                output="screen",
                parameters=[{"topic": "/base_pose_ground_truth", "print_rate": 2.0}],
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                name="initial_pose_sender",
                output="screen",
                parameters=[
                    {
                        "x": 2.0,
                        "y": 36.0,
                        "yaw": 0.0,
                        "frame_id": "map",
                        "topic": "/initialpose",
                        "startup_delay": 5.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                    }
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="snake_waypoint_runner.py",
                name="snake_waypoint_runner",
                output="screen",
                parameters=[
                    {
                        "waypoint_file": os.path.join(
                            autonomy_share, "config", "orchard_waypoints_default_start.yaml"
                        ),
                        "startup_delay": 8.0,
                        "action_name": "navigate_to_pose",
                        "goal_topic": "/current_goal",
                        "frame_id": "map",
                        "stop_on_failure": True,
                        "retries_per_waypoint": 0,
                    }
                ],
            ),
            Node(
                package="agribot_rl_nav",
                executable="depth_rl_policy_node.py",
                name="depth_rl_policy_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "depth_topic": "/camera/depth/image_rect_raw",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "global_plan_topic": "/plan",
                        "goal_topic": "/current_goal",
                        "cmd_vel_topic": "/rl/cmd_vel",
                        "control_hz": 10.0,
                        "max_depth": 8.0,
                        "max_linear_speed": 0.55,
                        "max_angular_speed": 1.0,
                        "goal_distance_clip": 40.0,
                        "path_point_count": 5,
                        "plan_stride": 4,
                        "seq_len": LaunchConfiguration("seq_len"),
                        "chunk_replan_interval": LaunchConfiguration("chunk_replan_interval"),
                        "stop_without_plan": LaunchConfiguration("stop_without_plan"),
                        "debug_log": LaunchConfiguration("debug_log"),
                    }
                ],
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
