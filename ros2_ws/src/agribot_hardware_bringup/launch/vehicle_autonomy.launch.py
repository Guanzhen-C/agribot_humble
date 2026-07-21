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
    localization = LaunchConfiguration("localization").perform(context)
    vehicle_type = LaunchConfiguration("vehicle_type").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    chassis_driver = LaunchConfiguration("chassis_driver").perform(context)
    enable_can = LaunchConfiguration("enable_can_output").perform(context).lower()
    allow_reference_ackermann = LaunchConfiguration(
        "allow_unverified_ackermann_protocol"
    ).perform(context).lower()
    output_enabled = enable_can in ("true", "1", "yes", "on")

    if localization not in ("navsat", "fastlio"):
        raise RuntimeError("localization must be 'navsat' or 'fastlio'")
    if vehicle_type not in ("differential", "ackermann"):
        raise RuntimeError("vehicle_type must be 'differential' or 'ackermann'")
    if controller not in ("dwb", "mppi"):
        raise RuntimeError("controller must be 'dwb' or 'mppi'")
    if vehicle_type == "differential" and controller != "dwb":
        raise RuntimeError("differential vehicle currently requires controller:=dwb")
    if vehicle_type == "ackermann" and controller != "mppi":
        raise RuntimeError("ackermann vehicle currently requires controller:=mppi")
    if chassis_driver not in (
        "none",
        "differential_can",
        "ackermann_can",
    ):
        raise RuntimeError(
            "chassis_driver must be none, differential_can or ackermann_can"
        )
    if output_enabled and chassis_driver == "none":
        raise RuntimeError(
            "enable_can_output:=true requires an explicitly selected chassis_driver"
        )
    if (
        output_enabled
        and vehicle_type == "differential"
        and chassis_driver != "differential_can"
    ):
        raise RuntimeError("differential vehicle requires a differential chassis driver")
    if (
        output_enabled
        and vehicle_type == "ackermann"
        and chassis_driver != "ackermann_can"
    ):
        raise RuntimeError("ackermann vehicle requires an Ackermann chassis driver")
    if (
        output_enabled
        and chassis_driver == "ackermann_can"
        and allow_reference_ackermann not in ("true", "1", "yes", "on")
    ):
        raise RuntimeError(
            "ackermann_can is a reference layout and requires explicit "
            "allow_unverified_ackermann_protocol:=true"
        )
    return []


def _selection_condition(vehicle_type, controller, localization):
    return IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("vehicle_type"),
                "' == '",
                vehicle_type,
                "' and '",
                LaunchConfiguration("controller"),
                "' == '",
                controller,
                "' and '",
                LaunchConfiguration("localization"),
                "' == '",
                localization,
                "'",
            ]
        )
    )


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    navigation_launch = os.path.join(
        hardware_share, "launch", "include", "navigation_only.launch.py"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    localization = LaunchConfiguration("localization")
    vehicle_type = LaunchConfiguration("vehicle_type")
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
                package="agribot_hardware_bringup",
                executable="rtk_eskf_localization",
                name="rtk_eskf_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("navsat_localization_config"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="agribot_hardware_bringup",
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
                package="agribot_hardware_bringup",
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

    ackermann_navsat_navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": LaunchConfiguration("navsat_nav2_params"),
                    "odom_topic": "/odometry/filtered_navsat",
                    "default_nav_to_pose_bt_xml": os.path.join(
                        hardware_share,
                        "ackermann",
                        "behavior_trees",
                        "navigate_w_replanning_ackermann_no_spin.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        hardware_share,
                        "ackermann",
                        "behavior_trees",
                        "navigate_through_poses_w_replanning_ackermann.xml",
                    ),
                }.items(),
            )
        ],
        condition=_selection_condition("ackermann", "mppi", "navsat"),
    )

    ackermann_fastlio_navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": LaunchConfiguration("fastlio_nav2_params"),
                    "odom_topic": "/fastlio/odometry",
                    "default_nav_to_pose_bt_xml": os.path.join(
                        hardware_share,
                        "ackermann",
                        "behavior_trees",
                        "navigate_w_replanning_ackermann_no_spin.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        hardware_share,
                        "ackermann",
                        "behavior_trees",
                        "navigate_through_poses_w_replanning_ackermann.xml",
                    ),
                }.items(),
            )
        ],
        condition=_selection_condition("ackermann", "mppi", "fastlio"),
    )

    differential_navsat_navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": LaunchConfiguration("dwb_navsat_nav2_params"),
                    "odom_topic": "/odometry/filtered_navsat",
                }.items(),
            )
        ],
        condition=_selection_condition("differential", "dwb", "navsat"),
    )

    differential_fastlio_navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": LaunchConfiguration("dwb_fastlio_nav2_params"),
                    "odom_topic": "/fastlio/odometry",
                }.items(),
            )
        ],
        condition=_selection_condition("differential", "dwb", "fastlio"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="navsat"),
            DeclareLaunchArgument("vehicle_type", default_value="ackermann"),
            DeclareLaunchArgument("controller", default_value="mppi"),
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
                "allow_unverified_ackermann_protocol", default_value="false"
            ),
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
                    hardware_share, "maps", "orchard_v2_map6.yaml"
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
                    hardware_share,
                    "ackermann",
                    "config",
                    "nav2_params_ackermann_navsat_static.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_nav2_params",
                default_value=os.path.join(
                    hardware_share,
                    "ackermann",
                    "config",
                    "nav2_params_ackermann_fastlio_static.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "dwb_navsat_nav2_params",
                default_value=os.path.join(
                    hardware_share,
                    "differential",
                    "config",
                    "nav2_dwb_navsat.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "dwb_fastlio_nav2_params",
                default_value=os.path.join(
                    hardware_share,
                    "differential",
                    "config",
                    "nav2_dwb_fastlio.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "differential_chassis_can_config",
                default_value=os.path.join(
                    hardware_share, "differential", "config", "chassis_can.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "ackermann_chassis_can_config",
                default_value=os.path.join(
                    hardware_share, "ackermann", "config", "chassis_can.yaml"
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
            ackermann_navsat_navigation,
            ackermann_fastlio_navigation,
            differential_navsat_navigation,
            differential_fastlio_navigation,
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="vehicle_collision_monitor",
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
                    {"node_names": ["vehicle_collision_monitor"]},
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
                                "' in ('differential_can', 'ackermann_can')",
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
                        "require_hardware_e_stop": PythonExpression(
                            [
                                "'",
                                enable_can_output,
                                "'.lower() in ('true', '1', 'yes', 'on') and '",
                                chassis_driver,
                                "' in ('differential_can', 'ackermann_can')",
                            ]
                        ),
                        "max_angular_velocity": PythonExpression(
                            [
                                "1.4 if '",
                                vehicle_type,
                                "' == 'differential' else 0.65",
                            ]
                        ),
                    },
                ],
            ),
            GroupAction(
                actions=[
                    Node(
                        package="agribot_hardware_bringup",
                        executable="differential_chassis_can_node",
                        name="differential_chassis_can",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("differential_chassis_can_config"),
                            {
                                "can_interface": LaunchConfiguration("can_interface"),
                            },
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "differential_can"
                        ),
                    ),
                    Node(
                        package="agribot_hardware_bringup",
                        executable="ackermann_chassis_can_node",
                        name="ackermann_chassis_can",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("ackermann_chassis_can_config"),
                            {
                                "can_interface": LaunchConfiguration("can_interface"),
                                "allow_unverified_protocol": LaunchConfiguration(
                                    "allow_unverified_ackermann_protocol"
                                ),
                            },
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "ackermann_can"
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
