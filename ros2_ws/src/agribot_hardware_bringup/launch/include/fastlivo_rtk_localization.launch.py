import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_paths(context):
    map_base = Path(LaunchConfiguration("map_base").perform(context)).expanduser()
    initialization_source = LaunchConfiguration("initialization_source").perform(
        context
    )
    allow_missing_georeference = LaunchConfiguration(
        "allow_missing_georeference"
    ).perform(context).lower() in ("true", "1", "yes", "on")
    enable_rtk_initialization = LaunchConfiguration(
        "enable_rtk_initialization"
    ).perform(context).lower() in ("true", "1", "yes", "on")
    required = [map_base.with_suffix(".pcd"), map_base.with_suffix(".yaml")]
    if (
        initialization_source == "rtk"
        or (
            initialization_source == "auto"
            and enable_rtk_initialization
            and not allow_missing_georeference
        )
        or not allow_missing_georeference
    ):
        required.append(Path(f"{map_base}_georeference.yaml"))
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(
            "FAST-LIVO2/RTK融合缺少地图文件: "
            + ", ".join(str(path) for path in missing)
        )
    if initialization_source not in ("auto", "rtk", "manual", "lidar"):
        raise RuntimeError(
            "initialization_source必须是auto、rtk、manual或lidar"
        )
    start_initial_localizer = LaunchConfiguration(
        "start_initial_localizer"
    ).perform(context)
    if (
        initialization_source in ("auto", "rtk")
        and start_initial_localizer.lower() not in ("true", "1", "yes", "on")
    ):
        raise RuntimeError("自动或RTK初始重定位必须启动PCD初始定位器")

    config_arguments = (
        "mount_config",
        "rtk_config",
        "camera_config",
        "fastlivo_lidar_config",
        "fastlivo_camera_config",
        "fastlivo_bridge_config",
        "pcd_initial_localization_config",
        "rtk_map_initializer_config",
        "fastlivo_rtk_fusion_config",
        "visual_initialization_config",
        "camera_calibration_status",
    )
    for argument in config_arguments:
        config_path = Path(
            LaunchConfiguration(argument).perform(context)
        ).expanduser()
        if not config_path.is_file():
            raise RuntimeError(f"定位配置文件不存在({argument}): {config_path}")

    start_fastlivo = LaunchConfiguration("start_fastlivo").perform(
        context
    ).lower() in ("true", "1", "yes", "on")
    camera_driver = LaunchConfiguration("camera_driver").perform(context)
    if start_fastlivo and camera_driver == "hikrobot_mvs":
        status_path = Path(
            LaunchConfiguration("camera_calibration_status").perform(context)
        ).expanduser()
        try:
            status = yaml.safe_load(status_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise RuntimeError(f"无法读取海康相机标定状态: {status_path}: {error}") from error
        required_checks = (
            "lens_installed",
            "intrinsics_calibrated",
            "lidar_camera_extrinsics_calibrated",
            "image_time_offset_calibrated",
        )
        incomplete = [name for name in required_checks if not status.get(name, False)]
        expected_serial = LaunchConfiguration("hikrobot_camera_serial").perform(
            context
        )
        if status.get("serial_number") != expected_serial:
            incomplete.append("serial_number")
        if incomplete:
            raise RuntimeError(
                "海康相机尚未完成FAST-LIVO2标定，禁止启动视觉定位: "
                + ", ".join(incomplete)
                + "；可用start_fastlivo:=false只测试相机取流"
            )
    return []


def _robot_state_publisher(context):
    enabled = LaunchConfiguration("use_detailed_vehicle_model").perform(context)
    if enabled.lower() not in ("true", "1", "yes", "on"):
        return []
    description_path = Path(
        LaunchConfiguration("robot_description_file").perform(context)
    ).expanduser()
    if not description_path.is_file():
        raise RuntimeError(f"车辆URDF不存在: {description_path}")
    description = description_path.read_text(encoding="utf-8")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=LaunchConfiguration("robot_state_publisher_name"),
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_description": description,
                }
            ],
        )
    ]


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    fastlivo_share = get_package_share_directory("fast_livo")
    map_base = LaunchConfiguration("map_base")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_pcd = PythonExpression(["'", map_base, ".pcd'"])
    map_yaml = PythonExpression(["'", map_base, ".yaml'"])
    georeference = PythonExpression(["'", map_base, "_georeference.yaml'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base",
                description="不带扩展名的地图绝对路径",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_driver", default_value="hikrobot_mvs"),
            DeclareLaunchArgument(
                "hikrobot_camera_serial", default_value="DB0447659"
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_enable", default_value="false"
            ),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("start_initial_localizer", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("use_detailed_vehicle_model", default_value="false"),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=os.path.join(
                    hardware_share, "urdf", "ackermann_vehicle.urdf"
                ),
            ),
            DeclareLaunchArgument(
                "robot_state_publisher_name",
                default_value="vehicle_robot_state_publisher",
            ),
            DeclareLaunchArgument(
                "mount_config",
                default_value=os.path.join(
                    hardware_share, "config", "sensor_mounts.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rtk_config",
                default_value=os.path.join(hardware_share, "config", "rtk_nmea.yaml"),
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=os.path.join(
                    hardware_share, "config", "right_camera.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlivo_lidar_config",
                default_value=os.path.join(
                    fastlivo_share, "config", "agribot_c16_astra.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlivo_camera_config",
                default_value=os.path.join(
                    hardware_share, "config", "fastlivo_hikrobot_mv_cu013.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "camera_calibration_status",
                default_value=os.path.join(
                    hardware_share,
                    "ackermann",
                    "config",
                    "hikrobot_camera_calibration_status.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "fastlivo_bridge_config",
                default_value=os.path.join(
                    hardware_share, "config", "fastlivo_bridge.yaml"
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
                "fastlivo_rtk_fusion_config",
                default_value=os.path.join(
                    hardware_share, "config", "fastlivo_rtk_fusion.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "visual_initialization_config",
                default_value=os.path.join(
                    hardware_share, "config", "visual_initialization.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    hardware_share, "rviz", "fastlivo_rtk_localization.rviz"
                ),
            ),
            DeclareLaunchArgument(
                "initialization_source",
                default_value="auto",
                description=(
                    "auto依次尝试RTK、EigenPlaces视觉和手动粗位姿；"
                    "所有粗位姿都必须通过NDT/GICP精配准；"
                    "rtk、manual和lidar保留为单一来源兼容模式"
                ),
            ),
            DeclareLaunchArgument(
                "enable_rtk_initialization", default_value="true"
            ),
            DeclareLaunchArgument(
                "enable_visual_initialization", default_value="true"
            ),
            DeclareLaunchArgument(
                "visual_model_file",
                default_value=os.path.join(
                    hardware_share,
                    "models",
                    "eigenplaces_r18_512_480x640_bayes_e.bin",
                ),
            ),
            DeclareLaunchArgument(
                "visual_database_file",
                default_value=PythonExpression(
                    ["'", map_base, "_visual_index.npz'"]
                ),
            ),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference",
                default_value="false",
                description=(
                    "仅用于无RTK室内地图；允许manual/lidar初始化时不提供地理配准"
                ),
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            OpaqueFunction(function=_validate_paths),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                hardware_share, "launch", "sensors.launch.py"
                            )
                        ),
                        launch_arguments={
                            "start_lidar": "true",
                            "start_imu": "true",
                            "start_rtk": LaunchConfiguration("start_rtk"),
                            "rviz": "false",
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "mount_config": LaunchConfiguration("mount_config"),
                            "rtk_config": LaunchConfiguration("rtk_config"),
                        }.items(),
                        condition=IfCondition(
                            LaunchConfiguration("start_sensors")
                        ),
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share, "launch", "include", "right_camera.launch.py"
                    )
                ),
                launch_arguments={
                    "camera_driver": LaunchConfiguration("camera_driver"),
                    "use_sim_time": use_sim_time,
                    "hikrobot_camera_serial": LaunchConfiguration(
                        "hikrobot_camera_serial"
                    ),
                    "hikrobot_trigger_enable": LaunchConfiguration(
                        "hikrobot_trigger_enable"
                    ),
                    "usb_camera_config": LaunchConfiguration("camera_config"),
                    "right_camera_device": LaunchConfiguration(
                        "right_camera_device"
                    ),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="fastlivo_mapping",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlivo_lidar_config"),
                    LaunchConfiguration("fastlivo_camera_config"),
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(LaunchConfiguration("start_fastlivo")),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlio_odom_bridge.py",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlivo_bridge_config"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_fastlivo_world",
                arguments=["--frame-id", "odom", "--child-frame-id", "camera_init"],
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="pcd_initial_localizer",
                name="pcd_initial_localizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("pcd_initial_localization_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "map_file_path": map_pcd,
                        "cloud_topic": "/lidar/points",
                        "cloud_frame": "lidar_link",
                        "odom_topic": "/fastlivo/odometry",
                        "initial_pose_topic": PythonExpression(
                            [
                                "'/localization/initialpose_prior' if '",
                                LaunchConfiguration("initialization_source"),
                                "' == 'auto' else ("
                                "'/localization/rtk_initialpose' if '",
                                LaunchConfiguration("initialization_source"),
                                "' == 'rtk' else '/initialpose')",
                            ]
                        ),
                        "pose_topic": "/localization_pose",
                        "ready_topic": "/localization/lidar_ready",
                        "status_topic": "/localization/status",
                        "publish_tf": False,
                        "enable_fpfh": PythonExpression(
                            [
                                "'",
                                LaunchConfiguration("initialization_source"),
                                "' == 'lidar' or '",
                                LaunchConfiguration("enable_fpfh"),
                                "'.lower() in ('true', '1', 'yes', 'on')",
                            ]
                        ),
                        "automatic_global_localization": PythonExpression(
                            [
                                "'",
                                LaunchConfiguration("initialization_source"),
                                "' == 'lidar'",
                            ]
                        ),
                    },
                ],
                condition=IfCondition(
                    LaunchConfiguration("start_initial_localizer")
                ),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="rtk_map_initializer",
                name="rtk_map_initializer",
                output="screen",
                parameters=[
                    LaunchConfiguration("rtk_map_initializer_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "georeference_file": georeference,
                        "map_file": map_pcd,
                        "odometry_topic": "/fastlivo/odometry",
                        "initial_pose_topic": "/localization/rtk_initialpose",
                        "localizer_status_topic": "/localization/status",
                        "localizer_ready_topic": "/localization/lidar_ready",
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "('",
                            LaunchConfiguration("initialization_source"),
                            "' == 'rtk') or ('",
                            LaunchConfiguration("initialization_source"),
                            "' == 'auto' and '",
                            LaunchConfiguration("enable_rtk_initialization"),
                            "'.lower() in ('true', '1', 'yes', 'on'))",
                        ]
                    )
                ),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="visual_place_recognizer.py",
                name="visual_place_recognizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("visual_initialization_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "model_file": LaunchConfiguration("visual_model_file"),
                        "database_file": LaunchConfiguration(
                            "visual_database_file"
                        ),
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("initialization_source"),
                            "' == 'auto' and '",
                            LaunchConfiguration("enable_visual_initialization"),
                            "'.lower() in ('true', '1', 'yes', 'on')",
                        ]
                    )
                ),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="initialization_coordinator.py",
                name="initialization_coordinator",
                output="screen",
                parameters=[
                    LaunchConfiguration("visual_initialization_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "rtk_enabled": LaunchConfiguration(
                            "enable_rtk_initialization"
                        ),
                        "visual_enabled": LaunchConfiguration(
                            "enable_visual_initialization"
                        ),
                    },
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("initialization_source"),
                            "' == 'auto'",
                        ]
                    )
                ),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlivo_rtk_fusion",
                name="fastlivo_rtk_fusion",
                output="screen",
                parameters=[
                    LaunchConfiguration("fastlivo_rtk_fusion_config"),
                    {
                        "use_sim_time": use_sim_time,
                        "georeference_file": georeference,
                        "map_file": map_pcd,
                        "allow_missing_georeference": LaunchConfiguration(
                            "allow_missing_georeference"
                        ),
                        # A single RTK position cannot determine yaw. The live
                        # launch only accepts a refined full-pose seed.
                        "auto_initialize_from_fixed_rtk": False,
                    },
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_fastlivo_rtk_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            OpaqueFunction(function=_robot_state_publisher),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fastlivo_rtk_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
