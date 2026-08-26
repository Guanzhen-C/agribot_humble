import os
from pathlib import Path

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
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context)
    vehicle_type = LaunchConfiguration("vehicle_type").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    chassis_driver = LaunchConfiguration("chassis_driver").perform(context)
    can_transport = LaunchConfiguration("can_transport").perform(context)
    map_path = LaunchConfiguration("map").perform(context)
    pcd_map_base_path = LaunchConfiguration("pcd_map_base").perform(context)
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    initialization_source = LaunchConfiguration("initialization_source").perform(
        context
    )
    map_georeference_file = LaunchConfiguration("map_georeference_file").perform(
        context
    )
    enable_chassis = (
        LaunchConfiguration("enable_chassis_output").perform(context).lower()
    )
    detailed_model = LaunchConfiguration(
        "use_detailed_vehicle_model"
    ).perform(context).lower()
    output_enabled = enable_chassis in ("true", "1", "yes", "on")

    if localization not in ("navsat", "fastlio"):
        raise RuntimeError("localization must be 'navsat' or 'fastlio'")
    if navigation_mode not in ("static", "local", "mapping", "localization"):
        raise RuntimeError(
            "navigation_mode must be 'static', 'local', 'mapping' or 'localization'"
        )
    if navigation_mode in ("local", "mapping") and localization != "fastlio":
        raise RuntimeError(
            f"{navigation_mode} navigation currently requires localization:=fastlio"
        )
    if navigation_mode in ("static", "localization") and not map_path:
        raise RuntimeError(
            f"{navigation_mode} navigation requires "
            "map:=/absolute/path/to/map.yaml"
        )
    if (
        navigation_mode == "static"
        and localization == "fastlio"
        and vehicle_type == "ackermann"
    ):
        raise RuntimeError(
            "Ackermann FAST-LIO static mode was removed; use navigation_mode:="
            "localization with a saved map"
        )
    if navigation_mode == "mapping" and not pcd_map_base_path:
        raise RuntimeError(
            "3D mapping requires pcd_map_base:=/absolute/path/to/map_name"
        )
    if navigation_mode == "localization" and not pcd_map_file:
        raise RuntimeError(
            "mapped 3D localization requires "
            "pcd_map_file:=/absolute/path/to/map.pcd"
        )
    if initialization_source not in ("manual", "lidar", "rtk"):
        raise RuntimeError(
            "initialization_source must be 'manual', 'lidar' or 'rtk'"
        )
    if (
        navigation_mode == "localization"
        and initialization_source == "rtk"
        and not map_georeference_file
    ):
        raise RuntimeError(
            "RTK map initialization requires map_georeference_file"
        )
    if vehicle_type not in ("differential", "ackermann"):
        raise RuntimeError("vehicle_type must be 'differential' or 'ackermann'")
    if controller != "mppi":
        raise RuntimeError(
            "physical vehicle bringup requires controller:=mppi; "
            "the legacy DWB configuration has been removed"
        )
    if chassis_driver not in (
        "none",
        "differential_can",
        "ackermann_can",
        "ackermann_serial",
    ):
        raise RuntimeError(
            "chassis_driver must be none, differential_can, ackermann_can "
            "or ackermann_serial"
        )
    if can_transport not in ("socketcan", "zqwl_cdc"):
        raise RuntimeError("can_transport must be 'socketcan' or 'zqwl_cdc'")
    if detailed_model not in (
        "true",
        "false",
        "1",
        "0",
        "yes",
        "no",
        "on",
        "off",
    ):
        raise RuntimeError("use_detailed_vehicle_model must be a boolean")
    if output_enabled and chassis_driver == "none":
        raise RuntimeError(
            "enable_chassis_output:=true requires an explicitly selected chassis_driver"
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
        and chassis_driver not in ("ackermann_can", "ackermann_serial")
    ):
        raise RuntimeError("ackermann vehicle requires an Ackermann chassis driver")
    return []


def _launch_ackermann_robot_state_publisher(
    context, *, hardware_share, use_sim_time
):
    if LaunchConfiguration("vehicle_type").perform(context) != "ackermann":
        return []
    use_detailed_model = LaunchConfiguration(
        "use_detailed_vehicle_model"
    ).perform(context).lower() in ("true", "1", "yes", "on")
    if not use_detailed_model:
        return []

    robot_description = Path(
        os.path.join(hardware_share, "urdf", "ackermann_vehicle.urdf")
    ).read_text(encoding="utf-8")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ackermann_robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                }
            ],
        )
    ]


def _selection_condition(vehicle_type, controller, localization):
    return IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_navigation"),
                "'.lower() in ('true', '1', 'yes', 'on') and '",
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
    enable_chassis_output = LaunchConfiguration("enable_chassis_output")

    sensors = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hardware_share, "launch", "sensors.launch.py")
                ),
                launch_arguments={
                    "start_lidar": "true",
                    "start_imu": "true",
                    "start_rtk": LaunchConfiguration("start_rtk"),
                    "lidar_forward_point_offset_sec": LaunchConfiguration(
                        "lidar_forward_point_offset_sec"
                    ),
                    "rviz": "false",
                    "lidar_config": LaunchConfiguration("lidar_config"),
                    "imu_config": LaunchConfiguration("imu_config"),
                    "rtk_config": LaunchConfiguration("rtk_config"),
                    "mount_config": LaunchConfiguration("mount_config"),
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_sensors")),
            )
        ],
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
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": LaunchConfiguration("navsat_output_frame"),
                        "auto_reference_from_first_navsat_fix": LaunchConfiguration(
                            "navsat_auto_reference_from_first_fix"
                        ),
                        "reference_lat_deg": LaunchConfiguration(
                            "navsat_reference_latitude_deg"
                        ),
                        "reference_lon_deg": LaunchConfiguration(
                            "navsat_reference_longitude_deg"
                        ),
                        "reference_alt_m": LaunchConfiguration(
                            "navsat_reference_altitude_m"
                        ),
                        "initial_pose_x": LaunchConfiguration(
                            "navsat_reference_map_x"
                        ),
                        "initial_pose_y": LaunchConfiguration(
                            "navsat_reference_map_y"
                        ),
                        "initial_pose_z": LaunchConfiguration(
                            "navsat_reference_map_z"
                        ),
                        "map_to_ned_yaw_deg": LaunchConfiguration(
                            "navsat_map_from_enu_yaw_deg"
                        ),
                    },
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
                        "pose_topic": LaunchConfiguration("navsat_pose_topic"),
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "tf_mode": LaunchConfiguration("navsat_tf_mode"),
                        "publish_readiness": LaunchConfiguration(
                            "navsat_publish_readiness"
                        ),
                        "ready_topic": LaunchConfiguration("navsat_ready_topic"),
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
            # The odom bridge preserves FAST-LIO's camera_init world coordinates.
            # This alias connects its deskewed body cloud to the Nav2 TF tree.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_fastlio_world",
                arguments=[
                    "--frame-id",
                    "odom",
                    "--child-frame-id",
                    "camera_init",
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
                condition=LaunchConfigurationEquals("navigation_mode", "static"),
            ),
        ],
        condition=LaunchConfigurationEquals("localization", "fastlio"),
    )

    pcd_mapping = TimerAction(
        period=LaunchConfiguration("map_start_delay"),
        actions=[
            Node(
                package="agribot_hardware_bringup",
                executable="pcd_map_builder",
                name="pcd_map_builder",
                output="screen",
                parameters=[
                    LaunchConfiguration("pcd_mapping_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "map_base_path": LaunchConfiguration("pcd_map_base"),
                    },
                ],
            )
        ],
        condition=LaunchConfigurationEquals("navigation_mode", "mapping"),
    )

    mapped_localization = TimerAction(
        period=LaunchConfiguration("map_start_delay"),
        actions=[
            Node(
                package="agribot_hardware_bringup",
                executable="pcd_initial_localizer",
                name="pcd_initial_localizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("pcd_initial_localization_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "map_file_path": LaunchConfiguration("pcd_map_file"),
                        "initial_pose_topic": LaunchConfiguration(
                            "mapped_initial_pose_topic"
                        ),
                        "enable_fpfh": LaunchConfiguration("enable_fpfh"),
                        "automatic_global_localization": LaunchConfiguration(
                            "automatic_global_localization"
                        ),
                    },
                ],
            )
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("navigation_mode"),
                    "' == 'localization' and '",
                    LaunchConfiguration("localization"),
                    "' == 'fastlio'",
                ]
            )
        ),
    )

    mapped_navsat_localization = TimerAction(
        period=LaunchConfiguration("map_start_delay"),
        actions=[
            Node(
                package="agribot_hardware_bringup",
                executable="pcd_initial_localizer",
                name="pcd_initial_localizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("pcd_initial_localization_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "map_file_path": LaunchConfiguration("pcd_map_file"),
                        "initial_pose_topic": LaunchConfiguration(
                            "mapped_initial_pose_topic"
                        ),
                        "enable_fpfh": False,
                        "automatic_global_localization": False,
                        "cloud_topic": "/lidar/points",
                        "cloud_frame": "lidar_link",
                        "odom_topic": "/odometry/filtered_navsat",
                        "base_to_body_xyz": [0.48, 0.0, 0.233],
                        "base_to_body_rpy": [0.0, 0.0, 0.0],
                        "external_ready_topic": LaunchConfiguration(
                            "navsat_ready_topic"
                        ),
                    },
                ],
            )
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("navigation_mode"),
                    "' == 'localization' and '",
                    LaunchConfiguration("localization"),
                    "' == 'navsat'",
                ]
            )
        ),
    )

    mapped_rtk_initializer = TimerAction(
        period=LaunchConfiguration("map_start_delay"),
        actions=[
            Node(
                package="agribot_hardware_bringup",
                executable="rtk_map_initializer",
                name="rtk_map_initializer",
                output="screen",
                parameters=[
                    LaunchConfiguration("rtk_map_initializer_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "georeference_file": LaunchConfiguration(
                            "map_georeference_file"
                        ),
                        "map_file": LaunchConfiguration("pcd_map_file"),
                        "odometry_topic": LaunchConfiguration(
                            "mapped_odometry_topic"
                        ),
                        "initial_pose_topic": LaunchConfiguration(
                            "mapped_initial_pose_topic"
                        ),
                    },
                ],
            )
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("navigation_mode"),
                    "' == 'localization' and '",
                    LaunchConfiguration("initialization_source"),
                    "' == 'rtk'",
                ]
            )
        ),
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
                    "params_file": LaunchConfiguration(
                        "differential_nav2_params"
                    ),
                    "odom_topic": "/odometry/filtered_navsat",
                    "lattice_filepath": LaunchConfiguration(
                        "differential_lattice_filepath"
                    ),
                }.items(),
            )
        ],
        condition=_selection_condition("differential", "mppi", "navsat"),
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
                    "params_file": LaunchConfiguration(
                        "differential_nav2_params"
                    ),
                    "odom_topic": "/fastlio/odometry",
                    "lattice_filepath": LaunchConfiguration(
                        "differential_lattice_filepath"
                    ),
                }.items(),
            )
        ],
        condition=_selection_condition("differential", "mppi", "fastlio"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="navsat"),
            DeclareLaunchArgument("navigation_mode", default_value="static"),
            DeclareLaunchArgument("vehicle_type", default_value="ackermann"),
            DeclareLaunchArgument("controller", default_value="mppi"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument(
                "lidar_forward_point_offset_sec", default_value="0.02504"
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_detailed_vehicle_model",
                default_value="false",
                description=(
                    "Display the detailed STEP-derived vehicle model in RViz; "
                    "false preserves the original pose/TF-only display"
                ),
            ),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="5.0"),
            DeclareLaunchArgument("map_start_delay", default_value="5.0"),
            DeclareLaunchArgument("enable_can_output", default_value="false"),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value=LaunchConfiguration("enable_can_output"),
                description=(
                    "Enable the selected physical chassis transport; "
                    "enable_can_output is retained as a compatibility alias"
                ),
            ),
            DeclareLaunchArgument("chassis_driver", default_value="none"),
            DeclareLaunchArgument("can_transport", default_value="socketcan"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "zqwl_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00"
                ),
            ),
            DeclareLaunchArgument("zqwl_channel", default_value="0"),
            DeclareLaunchArgument("zqwl_bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5C2C079857-if00"
                ),
            ),
            DeclareLaunchArgument(
                "command_input_topic", default_value="/nav2/cmd_vel"
            ),
            DeclareLaunchArgument("map_to_odom_x", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_y", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_z", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_roll", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_pitch", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description=(
                    "Absolute path to the real-vehicle Nav2 map YAML; required "
                    "for static and mapped FAST-LIO modes"
                ),
            ),
            DeclareLaunchArgument("pcd_map_base", default_value=""),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_georeference_file", default_value=""),
            DeclareLaunchArgument(
                "mapped_initial_pose_topic", default_value="/initialpose"
            ),
            DeclareLaunchArgument(
                "mapped_odometry_topic", default_value="/fastlio/odometry"
            ),
            DeclareLaunchArgument("navsat_output_frame", default_value="map"),
            DeclareLaunchArgument(
                "navsat_pose_topic", default_value="/localization_pose"
            ),
            DeclareLaunchArgument("navsat_tf_mode", default_value="odom_to_base"),
            DeclareLaunchArgument(
                "navsat_publish_readiness", default_value="true"
            ),
            DeclareLaunchArgument(
                "navsat_ready_topic", default_value="/localization/ready"
            ),
            DeclareLaunchArgument(
                "navsat_auto_reference_from_first_fix", default_value="true"
            ),
            DeclareLaunchArgument("navsat_reference_latitude_deg", default_value="0.0"),
            DeclareLaunchArgument("navsat_reference_longitude_deg", default_value="0.0"),
            DeclareLaunchArgument("navsat_reference_altitude_m", default_value="0.0"),
            DeclareLaunchArgument("navsat_reference_map_x", default_value="0.0"),
            DeclareLaunchArgument("navsat_reference_map_y", default_value="0.0"),
            DeclareLaunchArgument("navsat_reference_map_z", default_value="0.0"),
            DeclareLaunchArgument("navsat_map_from_enu_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument(
                "initialization_source",
                default_value="manual",
                description=(
                    "Initial map pose source: manual RViz pose, lidar global FPFH, "
                    "or fixed RTK followed by NDT/GICP"
                ),
            ),
            DeclareLaunchArgument(
                "enable_fpfh",
                default_value="false",
                description=(
                    "Run RViz-guided FPFH coarse registration before initial "
                    "NDT and GICP refinement"
                ),
            ),
            DeclareLaunchArgument(
                "automatic_global_localization", default_value="false"
            ),
            DeclareLaunchArgument(
                "require_localization_ready",
                default_value="false",
                description=(
                    "Inhibit physical chassis motion unless a fresh "
                    "/localization/ready heartbeat is true"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    hardware_share, "rviz", "navigation.rviz"
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
                "pcd_mapping_config",
                default_value=os.path.join(
                    hardware_share, "config", "pcd_mapping.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "pcd_initial_localization_config",
                default_value=os.path.join(
                    hardware_share, "config", "pcd_initial_localization.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rtk_map_initializer_config",
                default_value=os.path.join(
                    hardware_share, "config", "rtk_map_initializer.yaml"
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
                    "nav2_params_ackermann_fastlio_mapped.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "differential_nav2_params",
                default_value=os.path.join(
                    hardware_share,
                    "differential",
                    "config",
                    "nav2_params_differential_fastlivo_mapped.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "differential_lattice_filepath",
                default_value=os.path.join(
                    hardware_share,
                    "differential",
                    "config",
                    "motion_primitives",
                    "diff_5cm.json",
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
                "ackermann_chassis_serial_config",
                default_value=os.path.join(
                    hardware_share, "ackermann", "config", "chassis_serial.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "ackermann_joint_state_config",
                default_value=os.path.join(
                    hardware_share,
                    "ackermann",
                    "config",
                    "joint_state_publisher.yaml",
                ),
            ),
            OpaqueFunction(function=_validate_arguments),
            OpaqueFunction(
                function=_launch_ackermann_robot_state_publisher,
                kwargs={
                    "hardware_share": hardware_share,
                    "use_sim_time": use_sim_time,
                },
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="ackermann_joint_state_publisher",
                name="ackermann_joint_state_publisher",
                output="screen",
                parameters=[
                    LaunchConfiguration("ackermann_joint_state_config"),
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("vehicle_type"),
                            "' == 'ackermann' and '",
                            LaunchConfiguration("use_detailed_vehicle_model"),
                            "'.lower() in ('true', '1', 'yes', 'on')",
                        ]
                    )
                ),
            ),
            sensors,
            navsat_localization,
            fastlio_localization,
            pcd_mapping,
            mapped_localization,
            mapped_navsat_localization,
            mapped_rtk_initializer,
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "yaml_filename": LaunchConfiguration("map")}
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("navigation_mode"),
                            "' in ('static', 'localization')",
                        ]
                    )
                ),
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("navigation_mode"),
                            "' in ('static', 'localization')",
                        ]
                    )
                ),
            ),
            ackermann_navsat_navigation,
            ackermann_fastlio_navigation,
            differential_navsat_navigation,
            differential_fastlio_navigation,
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
                                "can_transport": LaunchConfiguration("can_transport"),
                                "can_interface": LaunchConfiguration("can_interface"),
                                "zqwl_port": LaunchConfiguration("zqwl_port"),
                                "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                                "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                                "command_topic": LaunchConfiguration(
                                    "command_input_topic"
                                ),
                                "require_localization_ready": LaunchConfiguration(
                                    "require_localization_ready"
                                ),
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
                                "can_transport": LaunchConfiguration("can_transport"),
                                "can_interface": LaunchConfiguration("can_interface"),
                                "zqwl_port": LaunchConfiguration("zqwl_port"),
                                "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                                "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                                "command_topic": LaunchConfiguration(
                                    "command_input_topic"
                                ),
                                "require_localization_ready": LaunchConfiguration(
                                    "require_localization_ready"
                                ),
                            },
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "ackermann_can"
                        ),
                    ),
                    Node(
                        package="agribot_hardware_bringup",
                        executable="ackermann_chassis_serial_node",
                        name="ackermann_chassis_serial",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("ackermann_chassis_serial_config"),
                            {
                                "port": LaunchConfiguration("serial_port"),
                                "command_topic": LaunchConfiguration(
                                    "command_input_topic"
                                ),
                                "require_localization_ready": LaunchConfiguration(
                                    "require_localization_ready"
                                ),
                            },
                        ],
                        condition=LaunchConfigurationEquals(
                            "chassis_driver", "ackermann_serial"
                        ),
                    ),
                ],
                condition=IfCondition(enable_chassis_output),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
