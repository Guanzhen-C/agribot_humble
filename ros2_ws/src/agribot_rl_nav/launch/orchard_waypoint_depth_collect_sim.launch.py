import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _clean_colon_env(name):
    raw = os.environ.get(name, "")
    if not raw:
        return raw

    kept = []
    for part in raw.split(":"):
        if not part:
            continue
        if part.startswith("/opt/ros/") and not part.startswith("/opt/ros/humble"):
            continue
        if part.endswith("/devel") or "/devel/" in part:
            continue
        kept.append(part)
    return ":".join(kept)


def generate_launch_description():
    autonomy_share = get_package_share_directory("agribot_autonomy")
    control_share = get_package_share_directory("scout_control")
    gazebo_share = get_package_share_directory("scout_gazebo")
    rl_nav_share = get_package_share_directory("agribot_rl_nav")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    scout_viz_share = get_package_share_directory("scout_viz")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")

    use_static_map = LaunchConfiguration("use_static_map")
    localization_mode = LaunchConfiguration("localization_mode")
    enable_slam_map = LaunchConfiguration("enable_slam_map")
    slam_mode = LaunchConfiguration("slam_mode")
    offboard_algorithm = LaunchConfiguration("offboard_algorithm")
    local_navigation_condition = IfCondition(
        PythonExpression(["'", offboard_algorithm, "' != 'true'"])
    )

    static_map_condition = IfCondition(use_static_map)
    mapless_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_static_map,
                "' != 'true' and '",
                offboard_algorithm,
                "' != 'true'",
            ]
        )
    )
    static_amcl_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_static_map,
                "' == 'true' and '",
                localization_mode,
                "' == 'amcl'",
            ]
        )
    )
    static_ground_truth_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_static_map,
                "' == 'true' and '",
                localization_mode,
                "' == 'ground_truth'",
            ]
        )
    )
    static_navsat_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_static_map,
                "' == 'true' and '",
                localization_mode,
                "' == 'navsat'",
            ]
        )
    )
    static_fastlio_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_static_map,
                "' == 'true' and '",
                localization_mode,
                "' == 'fast_lio'",
            ]
        )
    )
    ground_truth_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'ground_truth'",
            ]
        )
    )
    navsat_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'navsat'",
            ]
        )
    )
    fastlio_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'fast_lio'",
            ]
        )
    )
    local_fastlio_runtime_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'fast_lio' and '",
                offboard_algorithm,
                "' != 'true'",
            ]
        )
    )
    offboard_fastlio_display_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'fast_lio' and '",
                offboard_algorithm,
                "' == 'true'",
            ]
        )
    )
    local_navsat_runtime_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'navsat' and '",
                offboard_algorithm,
                "' != 'true'",
            ]
        )
    )
    offboard_navsat_display_condition = IfCondition(
        PythonExpression(
            [
                "'",
                localization_mode,
                "' == 'navsat' and '",
                offboard_algorithm,
                "' == 'true'",
            ]
        )
    )
    slam_gmapping_condition = IfCondition(
        PythonExpression(
            ["'", enable_slam_map, "' == 'true' and '", slam_mode, "' == 'gmapping'"]
        )
    )
    slam_ground_truth_condition = IfCondition(
        PythonExpression(
            ["'", enable_slam_map, "' == 'true' and '", slam_mode, "' == 'ground_truth'"]
        )
    )

    map_path = PathJoinSubstitution(
        [LaunchConfiguration("map_file_location"), LaunchConfiguration("map_file")]
    )

    def _map_server_groups(context):
        resolved_map_path = map_path.perform(context)
        resolved_nav_odom_topic = LaunchConfiguration("nav_odom_topic").perform(context)
        resolved_navigation_delay = float(
            LaunchConfiguration("navigation_delay").perform(context)
        )
        resolved_static_nav2_params = LaunchConfiguration("static_nav2_params_file").perform(context)
        resolved_fastlio_static_nav2_params = LaunchConfiguration(
            "fastlio_static_nav2_params_file"
        ).perform(context)
        resolved_navsat_static_nav2_params = LaunchConfiguration(
            "navsat_static_nav2_params_file"
        ).perform(context)
        resolved_default_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml").perform(context)

        static_navsat_localization_gate = Node(
            package="agribot_autonomy",
            executable="topic_ready_gate.py",
            name="static_navsat_localization_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "topic": "/amcl_pose",
                    "message_type": "pose",
                    "timeout_sec": resolved_navigation_delay + 20.0,
                }
            ],
        )
        static_fastlio_localization_gate = Node(
            package="agribot_autonomy",
            executable="topic_ready_gate.py",
            name="static_fastlio_localization_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "topic": "/amcl_pose",
                    "message_type": "pose",
                    "timeout_sec": resolved_navigation_delay + 20.0,
                }
            ],
        )

        return [
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
                                "yaml_filename": resolved_map_path,
                                "use_sim_time": True,
                            }
                        ],
                    ),
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package="nav2_lifecycle_manager",
                                executable="lifecycle_manager",
                                name="lifecycle_manager_map_server",
                                output="screen",
                                parameters=[
                                    {"use_sim_time": True},
                                    {"autostart": True},
                                    {"node_names": ["map_server"]},
                                ],
                            ),
                        ],
                    ),
                    TimerAction(
                        period=resolved_navigation_delay,
                        condition=local_navigation_condition,
                        actions=[
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
                                    "use_sim_time": "true",
                                    "autostart": "true",
                                    "params_file": resolved_static_nav2_params,
                                    "odom_topic": resolved_nav_odom_topic,
                                    "default_nav_to_pose_bt_xml": resolved_default_bt_xml,
                                }.items(),
                            )
                        ],
                    ),
                ],
            ),
            GroupAction(
                condition=static_navsat_condition,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": resolved_map_path,
                                "use_sim_time": True,
                            }
                        ],
                    ),
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package="nav2_lifecycle_manager",
                                executable="lifecycle_manager",
                                name="lifecycle_manager_map_server",
                                output="screen",
                                parameters=[
                                    {"use_sim_time": True},
                                    {"autostart": True},
                                    {"node_names": ["map_server"]},
                                ],
                            ),
                        ],
                    ),
                    TimerAction(
                        period=resolved_navigation_delay,
                        condition=local_navigation_condition,
                        actions=[static_navsat_localization_gate],
                    ),
                    RegisterEventHandler(
                        event_handler=OnProcessExit(
                            target_action=static_navsat_localization_gate,
                            on_exit=[
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
                                        "use_sim_time": "true",
                                        "autostart": "true",
                                        "params_file": resolved_navsat_static_nav2_params,
                                        "odom_topic": resolved_nav_odom_topic,
                                        "default_nav_to_pose_bt_xml": resolved_default_bt_xml,
                                    }.items(),
                                )
                            ],
                        ),
                        condition=local_navigation_condition,
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
                                "yaml_filename": resolved_map_path,
                                "use_sim_time": True,
                            }
                        ],
                    ),
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package="nav2_lifecycle_manager",
                                executable="lifecycle_manager",
                                name="lifecycle_manager_map_server",
                                output="screen",
                                parameters=[
                                    {"use_sim_time": True},
                                    {"autostart": True},
                                    {"node_names": ["map_server"]},
                                ],
                            ),
                        ],
                    ),
                    TimerAction(
                        period=resolved_navigation_delay,
                        condition=local_navigation_condition,
                        actions=[static_fastlio_localization_gate],
                    ),
                    RegisterEventHandler(
                        event_handler=OnProcessExit(
                            target_action=static_fastlio_localization_gate,
                            on_exit=[
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
                                        "use_sim_time": "true",
                                        "autostart": "true",
                                        "params_file": resolved_fastlio_static_nav2_params,
                                        "odom_topic": resolved_nav_odom_topic,
                                        "default_nav_to_pose_bt_xml": resolved_default_bt_xml,
                                    }.items(),
                                )
                            ],
                        ),
                        condition=local_navigation_condition,
                    ),
                ],
            ),
        ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", _clean_colon_env("LD_LIBRARY_PATH")),
            SetEnvironmentVariable("PYTHONPATH", _clean_colon_env("PYTHONPATH")),
            SetEnvironmentVariable("AMENT_PREFIX_PATH", _clean_colon_env("AMENT_PREFIX_PATH")),
            SetEnvironmentVariable("CMAKE_PREFIX_PATH", _clean_colon_env("CMAKE_PREFIX_PATH")),
            SetEnvironmentVariable("COLCON_PREFIX_PATH", _clean_colon_env("COLCON_PREFIX_PATH")),
            SetEnvironmentVariable("PKG_CONFIG_PATH", _clean_colon_env("PKG_CONFIG_PATH")),
            SetEnvironmentVariable("ROS_DISTRO", "humble"),
            SetEnvironmentVariable("ROS_VERSION", "2"),
            SetEnvironmentVariable("ROS_PYTHON_VERSION", "3"),
            SetLaunchConfiguration("autostart", "true"),
            SetLaunchConfiguration(
                "nav_odom_topic",
                PythonExpression(
                    ["'/fastlio/odometry' if '", localization_mode, "' == 'fast_lio' else '/odom'"]
                ),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            SetLaunchConfiguration("top_level_rviz", LaunchConfiguration("rviz")),
            DeclareLaunchArgument("rviz_start_delay", default_value="5.0"),
            DeclareLaunchArgument("nav_start_delay", default_value="12.0"),
            SetLaunchConfiguration(
                "navigation_delay",
                PythonExpression(
                    [
                        "'28.0' if '",
                        localization_mode,
                        "' == 'fast_lio' else "
                        "('22.0' if '",
                        localization_mode,
                        "' == 'navsat' else '",
                        LaunchConfiguration("nav_start_delay"),
                        "')",
                    ]
                ),
            ),
            DeclareLaunchArgument("enable_collector", default_value="false"),
            DeclareLaunchArgument("collector_start_delay", default_value="20.0"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    autonomy_share, "rviz", "robot_map_global_plan_only.rviz"
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "output_dir",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        ".local",
                        "share",
                        "agribot",
                        "data",
                        "depth_rl",
                    ]
                ),
            ),
            DeclareLaunchArgument("shard_size", default_value="128"),
            DeclareLaunchArgument("sample_hz", default_value="10.0"),
            DeclareLaunchArgument("frame_skip", default_value="2"),
            DeclareLaunchArgument("enable_slam_map", default_value="false"),
            DeclareLaunchArgument("slam_mode", default_value="gmapping"),
            DeclareLaunchArgument("use_static_map", default_value="false"),
            DeclareLaunchArgument("localization_mode", default_value="amcl"),
            DeclareLaunchArgument("offboard_algorithm", default_value="false"),
            DeclareLaunchArgument(
                "fastlio_config_file",
                default_value=os.path.join(autonomy_share, "config", "fast_lio_sim_tuned.yaml"),
            ),
            DeclareLaunchArgument("fastlio_visualize", default_value="false"),
            DeclareLaunchArgument(
                "map_file_location",
                default_value=os.path.join(scout_navigation_share, "maps"),
            ),
            DeclareLaunchArgument("map_file", default_value="orchard_v2_map6.yaml"),
            DeclareLaunchArgument(
                "static_nav2_params_file",
                default_value=os.path.join(autonomy_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "mapless_nav2_params_file",
                default_value=os.path.join(autonomy_share, "config", "nav2_params_mapless.yaml"),
            ),
            DeclareLaunchArgument(
                "fastlio_static_nav2_params_file",
                default_value=os.path.join(
                    autonomy_share, "config", "nav2_params_fastlio_static.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "navsat_static_nav2_params_file",
                default_value=os.path.join(rl_nav_share, "config", "nav2_params_navsat_static.yaml"),
            ),
            DeclareLaunchArgument(
                "navsat_ekf_params_file",
                default_value=os.path.join(rl_nav_share, "config", "navsat_kf_gins_map.yaml"),
            ),
            DeclareLaunchArgument("navsat_pose_topic", default_value="/odometry/gps"),
            DeclareLaunchArgument("navsat_pose_message_type", default_value="odometry"),
            DeclareLaunchArgument("navsat_imu_topic", default_value="/imu/data_corrected"),
            DeclareLaunchArgument("navsat_reference_lat", default_value="30.5"),
            DeclareLaunchArgument("navsat_reference_lon", default_value="114.0"),
            DeclareLaunchArgument("navsat_reference_alt", default_value="20.0"),
            DeclareLaunchArgument(
                "navsat_auto_reference_from_first_noah_gnss",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    scout_navigation_share, "config", "slam_toolbox_online_async.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    autonomy_share,
                    "behavior_trees",
                    "navigate_w_replanning_no_spin.xml",
                ),
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="2.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="36.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.139246"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("waypoint_transform_enabled", default_value="false"),
            DeclareLaunchArgument("laser_3d_enabled", default_value="true"),
            DeclareLaunchArgument("laser_3d_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("laser_3d_topic", default_value="/points"),
            DeclareLaunchArgument("laser_3d_update_rate", default_value="5"),
            DeclareLaunchArgument("laser_3d_horizontal_samples", default_value="360"),
            DeclareLaunchArgument("laser_3d_vertical_samples", default_value="16"),
            DeclareLaunchArgument("laser_3d_min_range", default_value="0.3"),
            DeclareLaunchArgument("laser_3d_max_range", default_value="25.0"),
            DeclareLaunchArgument("slam_namespace", default_value="slam_mapping"),
            DeclareLaunchArgument("slam_map_frame", default_value="slam_map"),
            DeclareLaunchArgument("slam_odom_frame", default_value="slam_odom"),
            DeclareLaunchArgument("slam_base_frame", default_value="slam_base_link"),
            DeclareLaunchArgument("slam_scan_topic", default_value="/scan"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "scout_orchard_world.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": "false",
                    "headless": LaunchConfiguration("headless"),
                    "use_xvfb": "false",
                    "use_sim_time": "true",
                    "x": LaunchConfiguration("initial_pose_x"),
                    "y": LaunchConfiguration("initial_pose_y"),
                    "z": LaunchConfiguration("initial_pose_z"),
                    "yaw": LaunchConfiguration("initial_pose_yaw"),
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
                    "publish_odom_tf": PythonExpression(
                        ["'false' if '", localization_mode, "' == 'fast_lio' else 'true'"]
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(autonomy_share, "launch", "fast_lio_sim.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "is_simulation": "true",
                    "fastlio_config_file": LaunchConfiguration("fastlio_config_file"),
                    "fastlio_visualize": LaunchConfiguration("fastlio_visualize"),
                    "fastlio_output_odom_topic": LaunchConfiguration("nav_odom_topic"),
                    "fastlio_output_odom_frame": "odom",
                    "fastlio_output_base_frame": "base_link",
                    "fastlio_stamp_with_current_time": "true",
                }.items(),
                condition=local_fastlio_runtime_condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_share, "launch", "control.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "enable_ekf": "false",
                    "ekf_publish_tf": PythonExpression(
                        ["'false' if '", localization_mode, "' == 'navsat' else 'true'"]
                    ),
                    "enable_twist_mux": "true",
                    "cmd_vel_out_topic": "/cmd_vel",
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", offboard_algorithm, "' != 'true'"])
                ),
            ),
            GroupAction(
                condition=static_amcl_condition,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_bringup_share, "launch", "localization_launch.py")
                        ),
                        launch_arguments={
                            "map": map_path,
                            "params_file": LaunchConfiguration("static_nav2_params_file"),
                            "use_sim_time": "true",
                            "autostart": "true",
                        }.items(),
                    ),
                    TimerAction(
                        period=LaunchConfiguration("navigation_delay"),
                        condition=local_navigation_condition,
                        actions=[
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
                                    "use_sim_time": "true",
                                    "autostart": "true",
                                    "params_file": LaunchConfiguration("static_nav2_params_file"),
                                    "odom_topic": LaunchConfiguration("nav_odom_topic"),
                                    "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                        "default_nav_to_pose_bt_xml"
                                    ),
                                }.items(),
                            )
                        ],
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
                        "use_sim_time": True,
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "z": LaunchConfiguration("initial_pose_z"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "frame_id": "map",
                        "topic": "/initialpose",
                        "startup_delay": 6.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                    }
                ],
                condition=static_amcl_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                name="initial_pose_sender_navsat",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "z": LaunchConfiguration("initial_pose_z"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "frame_id": "map",
                        "topic": "/initialpose",
                        "startup_delay": 6.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                    }
                ],
                condition=static_navsat_condition,
            ),
            # map_server groups use OpaqueFunction to resolve PathJoinSubstitution
            # (ROS 2 Humble Node parameters do not support launch substitutions)
            OpaqueFunction(function=_map_server_groups),
            GroupAction(
                condition=mapless_condition,
                actions=[
                    TimerAction(
                        period=LaunchConfiguration("navigation_delay"),
                        actions=[
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
                                    "use_sim_time": "true",
                                    "autostart": "true",
                                    "params_file": LaunchConfiguration("mapless_nav2_params_file"),
                                    "odom_topic": LaunchConfiguration("nav_odom_topic"),
                                    "default_nav_to_pose_bt_xml": LaunchConfiguration(
                                        "default_nav_to_pose_bt_xml"
                                    ),
                                }.items(),
                            )
                        ],
                    ),
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_localization.py",
                name="ground_truth_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "pose_topic": "/amcl_pose",
                    }
                ],
                condition=ground_truth_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_printer.py",
                name="ground_truth_printer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": "/base_pose_ground_truth",
                        "print_rate": 2.0,
                    }
                ],
                condition=ground_truth_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_display_bridge.py",
                name="ground_truth_display_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "pose_topic": "/amcl_pose",
                        "stop_topic": "/fastlio_pose",
                        "stop_topic_type": "pose",
                        "publish_rate": 20.0,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=offboard_fastlio_display_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="imu_frame_bridge.py",
                name="imu_frame_bridge",
                output="screen",
                condition=local_navsat_runtime_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="navsat_to_local_odom.py",
                name="navsat_to_local_odom",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "fix_topic": "/navsat/fix",
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "yaw_source_topic": "/base_pose_ground_truth",
                        "yaw_source_message_type": "odometry",
                        "invert_gazebo_axes": True,
                        "zero_altitude": False,
                        "publish_pose_and_tf": False,
                        "yaw_variance": 0.02,
                        "origin_x": LaunchConfiguration("initial_pose_x"),
                        "origin_y": LaunchConfiguration("initial_pose_y"),
                        "origin_z": LaunchConfiguration("initial_pose_z"),
                        "origin_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=local_navsat_runtime_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="rtk_eskf_localization",
                name="rtk_eskf_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("navsat_ekf_params_file"),
                    {
                        "use_sim_time": True,
                        "imu_topic": ParameterValue(LaunchConfiguration("navsat_imu_topic"), value_type=str),
                        "pose_topic": ParameterValue(LaunchConfiguration("navsat_pose_topic"), value_type=str),
                        "pose_message_type": ParameterValue(
                            LaunchConfiguration("navsat_pose_message_type"),
                            value_type=str,
                        ),
                        "reference_lat_deg": ParameterValue(
                            LaunchConfiguration("navsat_reference_lat"),
                            value_type=float,
                        ),
                        "reference_lon_deg": ParameterValue(
                            LaunchConfiguration("navsat_reference_lon"),
                            value_type=float,
                        ),
                        "reference_alt_m": ParameterValue(
                            LaunchConfiguration("navsat_reference_alt"),
                            value_type=float,
                        ),
                        "auto_reference_from_first_noah_gnss": ParameterValue(
                            LaunchConfiguration("navsat_auto_reference_from_first_noah_gnss"),
                            value_type=bool,
                        ),
                        "initial_pose_x": ParameterValue(LaunchConfiguration("initial_pose_x"), value_type=float),
                        "initial_pose_y": ParameterValue(LaunchConfiguration("initial_pose_y"), value_type=float),
                        "initial_pose_z": ParameterValue(
                            LaunchConfiguration("initial_pose_z"),
                            value_type=float,
                        ),
                        "initial_pose_yaw": ParameterValue(
                            LaunchConfiguration("initial_pose_yaw"),
                            value_type=float,
                        ),
                    }
                ],
                condition=local_navsat_runtime_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="navsat_pose_bridge.py",
                name="navsat_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": "/odometry/filtered_navsat",
                        "pose_topic": "/amcl_pose",
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                    }
                ],
                condition=local_navsat_runtime_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="localization_pose_printer.py",
                name="navsat_pose_printer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": "/amcl_pose",
                        "message_type": "pose",
                        "print_rate": 2.0,
                        "label": "Navsat localization",
                    }
                ],
                condition=local_navsat_runtime_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="ground_truth_display_bridge.py",
                name="navsat_display_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "pose_topic": "/amcl_pose",
                        "stop_topic": "/navsat_pose",
                        "stop_topic_type": "pose",
                        "publish_rate": 20.0,
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=offboard_navsat_display_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="kiss_localization.py",
                name="fastlio_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
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
                        "stamp_with_current_time": True,
                    }
                ],
                condition=local_fastlio_runtime_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="localization_pose_printer.py",
                name="fastlio_pose_printer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": "/amcl_pose",
                        "message_type": "pose",
                        "print_rate": 2.0,
                        "label": "FAST-LIO localization",
                    }
                ],
                condition=local_fastlio_runtime_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="snake_waypoint_runner.py",
                name="snake_waypoint_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "waypoint_file": os.path.join(
                            autonomy_share, "config", "orchard_waypoints_default_start.yaml"
                        ),
                        "navigation_mode": "follow_path",
                        "startup_delay": PythonExpression(
                            [
                                "'36.0' if '",
                                localization_mode,
                                "' == 'fast_lio' else ('24.0' if '",
                                localization_mode,
                                "' == 'navsat' else ('30.0' if '",
                                localization_mode,
                                "' == 'amcl' else '16.0'))",
                            ]
                        ),
                        "action_name": "navigate_to_pose",
                        "path_action_name": "follow_path",
                        "controller_id": "FollowPath",
                        "path_step": 0.5,
                        "goal_topic": "/current_goal",
                        "frame_id": PythonExpression(
                            [
                                "'odom' if '",
                                localization_mode,
                                "' == 'navsat' and '",
                                use_static_map,
                                "' != 'true' else 'map'",
                            ]
                        ),
                        "stop_on_failure": False,
                        "retries_per_waypoint": 2,
                        "transition_delay": 2.0,
                        "waypoint_transform_enabled": PythonExpression(
                            [
                                "'true' if '",
                                localization_mode,
                                "' == 'navsat' and '",
                                use_static_map,
                                "' != 'true' else '",
                                LaunchConfiguration("waypoint_transform_enabled"),
                                "'",
                            ]
                        ),
                        "waypoint_source_origin_x": LaunchConfiguration("initial_pose_x"),
                        "waypoint_source_origin_y": LaunchConfiguration("initial_pose_y"),
                        "waypoint_source_origin_yaw": LaunchConfiguration("initial_pose_yaw"),
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                        "require_pose_before_start": PythonExpression(
                            [
                                "'true' if ('",
                                localization_mode,
                                "' == 'fast_lio' or '",
                                localization_mode,
                                "' == 'navsat') else 'false'",
                            ]
                        ),
                    }
                ],
                condition=IfCondition(
                    PythonExpression(["'", offboard_algorithm, "' != 'true'"])
                ),
            ),
            GroupAction(
                condition=slam_gmapping_condition,
                actions=[
                    PushRosNamespace(LaunchConfiguration("slam_namespace")),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="slam_odom_bridge",
                        arguments=[
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "odom",
                            LaunchConfiguration("slam_odom_frame"),
                        ],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="slam_base_bridge",
                        arguments=[
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "0",
                            "base_link",
                            LaunchConfiguration("slam_base_frame"),
                        ],
                    ),
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("slam_params_file"),
                            {
                                "use_sim_time": True,
                                "odom_frame": LaunchConfiguration("slam_odom_frame"),
                                "base_frame": LaunchConfiguration("slam_base_frame"),
                                "map_frame": LaunchConfiguration("slam_map_frame"),
                                "scan_topic": LaunchConfiguration("slam_scan_topic"),
                            },
                        ],
                    ),
                ],
            ),
            GroupAction(
                condition=slam_ground_truth_condition,
                actions=[
                    PushRosNamespace(LaunchConfiguration("slam_namespace")),
                    Node(
                        package="agribot_rl_nav",
                        executable="ground_truth_scan_mapper.py",
                        name="ground_truth_scan_mapper",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "map_frame": LaunchConfiguration("slam_map_frame"),
                                "ground_truth_topic": "/base_pose_ground_truth",
                                "scan_topic": LaunchConfiguration("slam_scan_topic"),
                                "resolution": 0.05,
                                "origin_x": -20.0,
                                "origin_y": -20.0,
                                "width": 140.0,
                                "height": 140.0,
                                "max_usable_range": 5.5,
                                "max_range": 6.1,
                                "beam_stride": 1,
                                "publish_hz": 1.0,
                                "base_frame": "base_link",
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("rviz_start_delay"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(scout_viz_share, "launch", "view_robot.launch.py")
                        ),
                        launch_arguments={
                            "rviz_config": LaunchConfiguration("rviz_config"),
                        }.items(),
                        condition=IfCondition(LaunchConfiguration("top_level_rviz")),
                    ),
                ],
            ),
        ]
    )
