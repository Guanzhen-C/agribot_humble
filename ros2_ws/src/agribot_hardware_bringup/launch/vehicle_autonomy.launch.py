import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_arguments(context):
    localization = LaunchConfiguration("localization").perform(context).lower()
    chassis_driver = LaunchConfiguration("chassis_driver").perform(context).lower()
    enable_can = LaunchConfiguration("enable_can_output").perform(context).lower()

    if localization not in ("navsat", "fastlio"):
        raise RuntimeError("localization must be 'navsat' or 'fastlio'")
    if chassis_driver not in ("none", "scout", "simulated"):
        raise RuntimeError(
            "chassis_driver must be 'none', 'scout' or 'simulated'"
        )
    if enable_can in ("true", "1", "yes", "on") and chassis_driver == "none":
        raise RuntimeError(
            "enable_can_output:=true requires an explicitly selected chassis_driver"
        )
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    ackermann_share = get_package_share_directory("agribot_ackermann_mppi")
    navigation_share = get_package_share_directory("scout_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    localization = LaunchConfiguration("localization")
    enable_can_output = LaunchConfiguration("enable_can_output")
    chassis_driver = LaunchConfiguration("chassis_driver")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_share, "launch", "sensors.launch.py")
        ),
        launch_arguments={
            "start_lidar": "true",
            "start_imu": "true",
            "start_rtk": LaunchConfiguration("start_rtk"),
            "rviz": "false",
            "lidar_config": LaunchConfiguration("lidar_config"),
            "imu_config": LaunchConfiguration("imu_config"),
            "rtk_config": LaunchConfiguration("rtk_config"),
            "mount_config": LaunchConfiguration("mount_config"),
            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_sensors")),
    )

    navsat_localization = GroupAction(
        actions=[
            Node(
                package="agribot_rl_nav",
                executable="rtk_eskf_localization",
                name="rtk_eskf_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("navsat_localization_config"),
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
        ],
        condition=LaunchConfigurationEquals("localization", "navsat"),
    )

    fastlio_localization = GroupAction(
        actions=[
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlio_config"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="fastlio_odom_bridge.py",
                name="fastlio_odom_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlio_bridge_config"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_fastlio_odom",
                arguments=[
                    "--x",
                    LaunchConfiguration("map_to_odom_x"),
                    "--y",
                    LaunchConfiguration("map_to_odom_y"),
                    "--z",
                    LaunchConfiguration("map_to_odom_z"),
                    "--roll",
                    LaunchConfiguration("map_to_odom_roll"),
                    "--pitch",
                    LaunchConfiguration("map_to_odom_pitch"),
                    "--yaw",
                    LaunchConfiguration("map_to_odom_yaw"),
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
        ],
        condition=LaunchConfigurationEquals("localization", "fastlio"),
    )

    navsat_navigation = TimerAction(
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
                    "params_file": LaunchConfiguration("navsat_nav2_params"),
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
        condition=LaunchConfigurationEquals("localization", "navsat"),
    )

    fastlio_navigation = TimerAction(
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
                    "params_file": LaunchConfiguration("fastlio_nav2_params"),
                    "odom_topic": "/fastlio/odometry",
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
        condition=LaunchConfigurationEquals("localization", "fastlio"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="navsat"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="5.0"),
            DeclareLaunchArgument("enable_can_output", default_value="false"),
            DeclareLaunchArgument("chassis_driver", default_value="none"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "command_input_topic", default_value="/nav2/cmd_vel_safe"
            ),
            DeclareLaunchArgument("map_to_odom_x", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_y", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_z", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_roll", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_pitch", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    navigation_share, "maps", "orchard_v2_map6.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "lidar_config",
                default_value=os.path.join(hardware_share, "config", "c16.yaml"),
            ),
            DeclareLaunchArgument(
                "imu_config",
                default_value=os.path.join(
                    get_package_share_directory("hipnuc_imu"),
                    "config",
                    "n300pro.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "rtk_config",
                default_value=os.path.join(hardware_share, "config", "rtk_nmea.yaml"),
            ),
            DeclareLaunchArgument(
                "mount_config",
                default_value=os.path.join(
                    hardware_share, "config", "sensor_mounts.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "navsat_localization_config",
                default_value=os.path.join(
                    hardware_share, "config", "kf_gins_n300pro.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_config",
                default_value=os.path.join(
                    hardware_share, "config", "fast_lio_c16.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_bridge_config",
                default_value=os.path.join(
                    hardware_share, "config", "fastlio_bridge.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "navsat_nav2_params",
                default_value=os.path.join(
                    ackermann_share,
                    "config",
                    "nav2_params_ackermann_navsat_static.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_nav2_params",
                default_value=os.path.join(
                    ackermann_share,
                    "config",
                    "nav2_params_ackermann_fastlio_static.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=os.path.join(
                    hardware_share, "config", "vehicle_safety.yaml"
                ),
            ),
            OpaqueFunction(function=_validate_arguments),
            sensors,
            navsat_localization,
            fastlio_localization,
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
            navsat_navigation,
            fastlio_navigation,
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="ackermann_collision_monitor",
                output="screen",
                parameters=[
                    os.path.join(hardware_share, "config", "collision_monitor.yaml"),
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
                package="agribot_hardware_bringup",
                executable="vehicle_preflight.py",
                name="vehicle_preflight",
                output="screen",
                parameters=[
                    LaunchConfiguration("safety_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "localization_mode": localization,
                        "require_can": False,
                        "require_can_interface": PythonExpression(
                            [
                                "'",
                                enable_can_output,
                                "'.lower() in ('true', '1', 'yes', 'on') and '",
                                chassis_driver,
                                "' == 'scout'",
                            ]
                        ),
                        "require_chassis_feedback": enable_can_output,
                        "can_interface": LaunchConfiguration("can_interface"),
                    },
                ],
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="vehicle_command_gate.py",
                name="vehicle_command_gate",
                output="screen",
                parameters=[
                    LaunchConfiguration("safety_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "initially_enabled": enable_can_output,
                        "input_topic": LaunchConfiguration("command_input_topic"),
                    },
                ],
            ),
            GroupAction(
                actions=[
                    Node(
                        package="scout_base",
                        executable="scout_base_node",
                        name="scout_base_node",
                        output="screen",
                        parameters=[
                            {
                                "port_name": LaunchConfiguration("can_interface"),
                                "cmd_vel_topic": "/hardware/cmd_vel",
                                "simulated_robot": False,
                                "pub_tf": False,
                                "odom_topic_name": "/wheel/odometry",
                                "odom_frame": "wheel_odom",
                                "base_frame": "base_link",
                                "command_timeout_sec": 0.25,
                                "max_linear_velocity": 0.80,
                                "max_angular_velocity": 0.65,
                            }
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "scout"
                        ),
                    ),
                    Node(
                        package="scout_base",
                        executable="scout_base_node",
                        name="simulated_chassis",
                        output="screen",
                        parameters=[
                            {
                                "cmd_vel_topic": "/hardware/cmd_vel",
                                "simulated_robot": True,
                                "pub_tf": False,
                                "odom_topic_name": "/wheel/odometry",
                                "odom_frame": "wheel_odom",
                                "base_frame": "base_link",
                                "command_timeout_sec": 0.25,
                                "max_linear_velocity": 0.80,
                                "max_angular_velocity": 0.65,
                            }
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "simulated"
                        ),
                    ),
                ],
                condition=IfCondition(enable_can_output),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(hardware_share, "rviz", "navigation.rviz"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
