import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    agribot_share = get_package_share_directory("agribot_autonomy")
    scout_control_share = get_package_share_directory("scout_control")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")

    static_params_default = os.path.join(agribot_share, "config", "nav2_params.yaml")
    mapless_params_default = os.path.join(agribot_share, "config", "nav2_params_mapless.yaml")
    map_default = os.path.join(agribot_share, "maps", "orchard_v2_map6.yaml")
    waypoint_default = os.path.join(agribot_share, "config", "orchard_waypoints_default_start.yaml")

    use_ground_truth_localization = LaunchConfiguration("use_ground_truth_localization")
    use_kiss_localization = LaunchConfiguration("use_kiss_localization")
    use_fastlio_localization = LaunchConfiguration("use_fastlio_localization")
    use_static_map = LaunchConfiguration("use_static_map")
    scan_topic = LaunchConfiguration("scan_topic")

    amcl_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_ground_truth_localization,
                "' != 'true' and '",
                use_kiss_localization,
                "' != 'true' and '",
                use_fastlio_localization,
                "' != 'true'",
            ]
        )
    )
    static_ground_truth_condition = IfCondition(
        PythonExpression(
            ["'", use_ground_truth_localization, "' == 'true' and '", use_static_map, "' == 'true'"]
        )
    )
    mapless_ground_truth_condition = IfCondition(
        PythonExpression(
            ["'", use_ground_truth_localization, "' == 'true' and '", use_static_map, "' != 'true'"]
        )
    )
    static_kiss_condition = IfCondition(
        PythonExpression(
            ["'", use_kiss_localization, "' == 'true' and '", use_static_map, "' == 'true'"]
        )
    )
    mapless_kiss_condition = IfCondition(
        PythonExpression(
            ["'", use_kiss_localization, "' == 'true' and '", use_static_map, "' != 'true'"]
        )
    )
    static_fastlio_condition = IfCondition(
        PythonExpression(
            ["'", use_fastlio_localization, "' == 'true' and '", use_static_map, "' == 'true'"]
        )
    )
    mapless_fastlio_condition = IfCondition(
        PythonExpression(
            ["'", use_fastlio_localization, "' == 'true' and '", use_static_map, "' != 'true'"]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_static_map", default_value="false"),
            DeclareLaunchArgument("map", default_value=map_default),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("use_ground_truth_localization", default_value="true"),
            DeclareLaunchArgument("use_kiss_localization", default_value="false"),
            DeclareLaunchArgument("use_fastlio_localization", default_value="false"),
            DeclareLaunchArgument("nav_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("waypoint_file", default_value=waypoint_default),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="false"),
            DeclareLaunchArgument("control_enable_ekf", default_value="false"),
            DeclareLaunchArgument("control_enable_twist_mux", default_value="true"),
            DeclareLaunchArgument("static_nav2_params_file", default_value=static_params_default),
            DeclareLaunchArgument("mapless_nav2_params_file", default_value=mapless_params_default),
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
                    os.path.join(scout_control_share, "launch", "control.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_control")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_ekf": LaunchConfiguration("control_enable_ekf"),
                    "enable_twist_mux": LaunchConfiguration("control_enable_twist_mux"),
                }.items(),
            ),
            GroupAction(
                condition=amcl_condition,
                actions=[
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(scout_navigation_share, "launch", "amcl_navigation.launch.py")
                        ),
                        launch_arguments={
                            "map": LaunchConfiguration("map"),
                            "params_file": LaunchConfiguration("static_nav2_params_file"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "start_robot": "false",
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=static_ground_truth_condition,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": LaunchConfiguration("map"),
                                "use_sim_time": LaunchConfiguration("use_sim_time"),
                            }
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map_server",
                        output="screen",
                        parameters=[
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                            {"autostart": LaunchConfiguration("autostart")},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("static_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=mapless_ground_truth_condition,
                actions=[
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("mapless_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=static_kiss_condition,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": LaunchConfiguration("map"),
                                "use_sim_time": LaunchConfiguration("use_sim_time"),
                            }
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map_server",
                        output="screen",
                        parameters=[
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                            {"autostart": LaunchConfiguration("autostart")},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("static_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=mapless_kiss_condition,
                actions=[
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("mapless_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=static_fastlio_condition,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": LaunchConfiguration("map"),
                                "use_sim_time": LaunchConfiguration("use_sim_time"),
                            }
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map_server",
                        output="screen",
                        parameters=[
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                            {"autostart": LaunchConfiguration("autostart")},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("static_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                condition=mapless_fastlio_condition,
                actions=[
                    SetRemap(src="scan", dst=scan_topic),
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                scout_navigation_share,
                                "launch",
                                "include",
                                "navigation_only.launch.py",
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("mapless_nav2_params_file"),
                            "odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                "default_nav_to_pose_bt_xml"
                            ),
                        }.items(),
                    ),
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                name="initial_pose_sender",
                output="screen",
                parameters=[
                    {
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "z": LaunchConfiguration("initial_pose_z"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "frame_id": "map",
                        "topic": "/initialpose",
                        "startup_delay": 6.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_ground_truth_localization, "' != 'true'"]
                    )
                ),
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_localization.py",
                name="ground_truth_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "pose_topic": "/amcl_pose",
                    }
                ],
                condition=IfCondition(use_ground_truth_localization),
            ),
            Node(
                package="agribot_autonomy",
                executable="kiss_localization.py",
                name="kiss_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "odom_topic": "/kiss/odometry",
                        "initial_pose_topic": "/initialpose",
                        "pose_topic": "/amcl_pose",
                        "planar_mode": False,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=IfCondition(use_kiss_localization),
            ),
            Node(
                package="agribot_autonomy",
                executable="kiss_localization.py",
                name="fastlio_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "odom_topic": LaunchConfiguration("nav_odom_topic"),
                        "initial_pose_topic": "/initialpose",
                        "pose_topic": "/amcl_pose",
                        "planar_mode": False,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=IfCondition(use_fastlio_localization),
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_printer.py",
                name="ground_truth_printer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "topic": "/base_pose_ground_truth",
                        "print_rate": 2.0,
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
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "waypoint_file": LaunchConfiguration("waypoint_file"),
                        "startup_delay": 8.0,
                        "navigation_mode": "follow_path",
                        "action_name": "navigate_to_pose",
                        "path_action_name": "follow_path",
                        "controller_id": "FollowPath",
                        "path_step": 0.5,
                        "frame_id": "map",
                        "stop_on_failure": False,
                        "retries_per_waypoint": 2,
                        "transition_delay": 2.0,
                        "advance_distance": 2.0,
                        "proximity_advance_enabled": False,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
            ),
        ]
    )
