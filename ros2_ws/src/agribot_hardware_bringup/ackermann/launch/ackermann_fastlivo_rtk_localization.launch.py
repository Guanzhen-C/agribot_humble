import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_paths(context):
    map_base = Path(LaunchConfiguration("map_base").perform(context)).expanduser()
    missing = [
        path
        for path in (
            map_base.with_suffix(".pcd"),
            map_base.with_suffix(".yaml"),
            Path(f"{map_base}_georeference.yaml"),
        )
        if not path.is_file()
    ]
    if missing:
        raise RuntimeError(
            "FAST-LIVO2/RTK融合缺少地图文件: "
            + ", ".join(str(path) for path in missing)
        )
    return []


def _robot_state_publisher(context, hardware_share):
    enabled = LaunchConfiguration("use_detailed_vehicle_model").perform(context)
    if enabled.lower() not in ("true", "1", "yes", "on"):
        return []
    description = Path(
        hardware_share, "urdf", "ackermann_vehicle.urdf"
    ).read_text(encoding="utf-8")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ackermann_robot_state_publisher",
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
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("start_initial_localizer", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("use_detailed_vehicle_model", default_value="false"),
            DeclareLaunchArgument(
                "initialization_source",
                default_value="manual",
                description="manual使用RViz初值；lidar执行一次全图FPFH定位",
            ),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "auto_initialize_from_fixed_rtk",
                default_value="false",
                description="仅当FAST-LIVO2初始yaw已与地图对齐时开启",
            ),
            DeclareLaunchArgument(
                "initial_map_from_odom_yaw_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            OpaqueFunction(function=_validate_paths),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hardware_share, "launch", "sensors.launch.py")
                ),
                launch_arguments={
                    "start_lidar": "true",
                    "start_imu": "true",
                    "start_rtk": "true",
                    "rviz": "false",
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_sensors")),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    os.path.join(hardware_share, "config", "right_camera.yaml"),
                    {
                        "video_device": LaunchConfiguration("right_camera_device"),
                        "use_sim_time": use_sim_time,
                    },
                ],
                remappings=[
                    ("image_raw", "/camera/rgb/image_raw"),
                    ("camera_info", "/camera/rgb/camera_info"),
                ],
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="fastlivo_mapping",
                output="screen",
                parameters=[
                    os.path.join(fastlivo_share, "config", "agribot_c16_astra.yaml"),
                    os.path.join(fastlivo_share, "config", "agribot_astra_640.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(LaunchConfiguration("start_fastlivo")),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlio_odom_bridge.py",
                output="screen",
                parameters=[
                    os.path.join(hardware_share, "config", "fastlivo_bridge.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_fastlivo_world",
                arguments=["--frame-id", "odom", "--child-frame-id", "camera_init"],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="agribot_hardware_bringup",
                        executable="pcd_initial_localizer",
                        name="pcd_initial_localizer",
                        output="screen",
                        parameters=[
                            os.path.join(
                                hardware_share,
                                "config",
                                "pcd_initial_localization.yaml",
                            ),
                            {
                                "use_sim_time": use_sim_time,
                                "map_file_path": map_pcd,
                                "cloud_topic": "/lidar/points",
                                "cloud_frame": "lidar_link",
                                "odom_topic": "/fastlivo/odometry",
                                "pose_topic": "/localization_pose",
                                "ready_topic": "/localization/lidar_ready",
                                "publish_tf": False,
                                "base_to_body_xyz": [0.48, 0.0, 0.233],
                                "base_to_body_rpy": [
                                    -0.007648487,
                                    -0.001835661,
                                    0.000007020,
                                ],
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
                    )
                ],
                condition=IfCondition(
                    LaunchConfiguration("start_initial_localizer")
                ),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlivo_rtk_fusion",
                name="fastlivo_rtk_fusion",
                output="screen",
                parameters=[
                    os.path.join(
                        hardware_share, "config", "fastlivo_rtk_fusion.yaml"
                    ),
                    {
                        "use_sim_time": use_sim_time,
                        "georeference_file": georeference,
                        "map_file": map_pcd,
                        "auto_initialize_from_fixed_rtk": LaunchConfiguration(
                            "auto_initialize_from_fixed_rtk"
                        ),
                        "initial_map_from_odom_yaw_rad": LaunchConfiguration(
                            "initial_map_from_odom_yaw_rad"
                        ),
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
            OpaqueFunction(
                function=lambda context: _robot_state_publisher(
                    context, hardware_share
                )
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="fastlivo_rtk_rviz",
                        arguments=[
                            "-d",
                            os.path.join(
                                hardware_share,
                                "rviz",
                                "fastlivo_rtk_localization.rviz",
                            ),
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
