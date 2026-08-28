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


MOTION_AUTHORIZATION = "ENABLE_DIFFERENTIAL_MOTION"


def _enabled(value):
    return value.lower() in ("true", "1", "yes", "on")


def _validate_arguments(context):
    if LaunchConfiguration("chassis_driver").perform(context) != "differential_can":
        raise RuntimeError("差速车chassis_driver必须是differential_can")

    can_transport = LaunchConfiguration("can_transport").perform(context)
    if can_transport not in ("socketcan", "zqwl_cdc"):
        raise RuntimeError("can_transport必须是socketcan或zqwl_cdc")

    enable_chassis = _enabled(
        LaunchConfiguration("enable_chassis_output").perform(context)
    )
    allow_uncalibrated_camera = _enabled(
        LaunchConfiguration("allow_uncalibrated_camera").perform(context)
    )
    if enable_chassis and allow_uncalibrated_camera:
        raise RuntimeError("真车运动禁止绕过海康相机FAST-LIVO2标定检查")
    if not enable_chassis:
        return []

    if _enabled(LaunchConfiguration("use_sim_time").perform(context)):
        raise RuntimeError("真车底盘输出禁止与use_sim_time:=true同时使用")
    authorization = LaunchConfiguration("motion_authorization").perform(context)
    if authorization != MOTION_AUTHORIZATION:
        raise RuntimeError(
            "启用差速底盘必须显式设置motion_authorization:="
            f"{MOTION_AUTHORIZATION}"
        )

    calibration_path = Path(
        LaunchConfiguration("vehicle_calibration").perform(context)
    ).expanduser()
    try:
        calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RuntimeError(f"无法读取差速车标定文件: {calibration_path}: {error}") from error
    if not isinstance(calibration, dict):
        raise RuntimeError(f"差速车标定文件不是YAML映射: {calibration_path}")
    if calibration.get("vehicle_type") != "differential":
        raise RuntimeError("vehicle_calibration.yaml的vehicle_type必须是differential")
    if calibration.get("calibration_complete") is not True:
        raise RuntimeError(
            "差速车几何、轮系和传感器外参尚未标定，禁止创建底盘输出"
        )

    if can_transport == "zqwl_cdc":
        zqwl_port = Path(LaunchConfiguration("zqwl_port").perform(context))
        if not zqwl_port.exists():
            raise RuntimeError(f"USB-CAN设备不存在: {zqwl_port}")
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")
    differential_share = os.path.join(hardware_share, "differential")
    use_sim_time = LaunchConfiguration("use_sim_time")

    localization_launch = os.path.join(
        hardware_share,
        "launch",
        "differential_fastlivo_rtk_localization.launch.py",
    )
    navigation_launch = os.path.join(
        hardware_share, "launch", "include", "navigation_only.launch.py"
    )

    navigation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": LaunchConfiguration("autostart"),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "odom_topic": "/fastlivo_rtk/odometry",
                    "lattice_filepath": LaunchConfiguration("lattice_filepath"),
                    "default_nav_to_pose_bt_xml": os.path.join(
                        nav2_bt_share,
                        "behavior_trees",
                        "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        differential_share,
                        "behavior_trees",
                        "navigate_through_poses_w_replanning_differential.xml",
                    ),
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    obstacle_height_filter = Node(
        package="agribot_hardware_bringup",
        executable="differential_obstacle_height_filter",
        name="differential_obstacle_height_filter",
        output="screen",
        parameters=[
            os.path.join(
                differential_share, "config", "obstacle_height_filter.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    chassis = Node(
        package="agribot_hardware_bringup",
        executable="differential_chassis_can_node",
        name="differential_chassis_can",
        output="screen",
        parameters=[
            os.path.join(differential_share, "config", "chassis_can.yaml"),
            {
                "use_sim_time": use_sim_time,
                "can_transport": LaunchConfiguration("can_transport"),
                "can_interface": LaunchConfiguration("can_interface"),
                "zqwl_port": LaunchConfiguration("zqwl_port"),
                "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                "command_topic": "/nav2/cmd_vel",
                "require_localization_ready": True,
                "localization_ready_topic": "/fastlivo_rtk/ready",
                "localization_ready_timeout_sec": 1.0,
            },
        ],
        condition=IfCondition(LaunchConfiguration("enable_chassis_output")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base", description="不带扩展名的三维和二维地图绝对路径"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument(
                "lidar_forward_point_offset_sec", default_value="0.05004"
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_driver", default_value="hikrobot_mvs"),
            DeclareLaunchArgument(
                "hikrobot_camera_serial", default_value="DB0447659"
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_enable", default_value="true"
            ),
            DeclareLaunchArgument(
                "allow_uncalibrated_camera", default_value="false"
            ),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(hardware_share, "rviz", "navigation.rviz"),
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "initialization_source", default_value="manual"
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
                    ["'", LaunchConfiguration("map_base"), "_visual_index.npz'"]
                ),
            ),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference", default_value="true"
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    differential_share,
                    "config",
                    "nav2_params_differential_fastlivo_mapped.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "lattice_filepath",
                default_value=os.path.join(
                    differential_share,
                    "config",
                    "motion_primitives",
                    "diff_5cm.json",
                ),
            ),
            DeclareLaunchArgument(
                "vehicle_calibration",
                default_value=os.path.join(
                    differential_share, "config", "vehicle_calibration.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value="false",
                description="默认只运行定位、规划和避障，不创建底盘节点",
            ),
            DeclareLaunchArgument("motion_authorization", default_value=""),
            DeclareLaunchArgument("chassis_driver", default_value="differential_can"),
            DeclareLaunchArgument("can_transport", default_value="zqwl_cdc"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "zqwl_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00"
                ),
            ),
            DeclareLaunchArgument("zqwl_channel", default_value="0"),
            DeclareLaunchArgument("zqwl_bitrate", default_value="250000"),
            OpaqueFunction(function=_validate_arguments),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(localization_launch),
                        launch_arguments={
                            "map_base": LaunchConfiguration("map_base"),
                            "use_sim_time": use_sim_time,
                            "start_sensors": LaunchConfiguration("start_sensors"),
                            "start_rtk": LaunchConfiguration("start_rtk"),
                            "lidar_forward_point_offset_sec": LaunchConfiguration(
                                "lidar_forward_point_offset_sec"
                            ),
                            "start_camera": LaunchConfiguration("start_camera"),
                            "camera_driver": LaunchConfiguration("camera_driver"),
                            "hikrobot_camera_serial": LaunchConfiguration(
                                "hikrobot_camera_serial"
                            ),
                            "hikrobot_trigger_enable": LaunchConfiguration(
                                "hikrobot_trigger_enable"
                            ),
                            "allow_uncalibrated_camera": LaunchConfiguration(
                                "allow_uncalibrated_camera"
                            ),
                            "start_fastlivo": LaunchConfiguration("start_fastlivo"),
                            "start_initial_localizer": "true",
                            "rviz": "false",
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "initialization_source": LaunchConfiguration(
                                "initialization_source"
                            ),
                            "enable_rtk_initialization": LaunchConfiguration(
                                "enable_rtk_initialization"
                            ),
                            "enable_visual_initialization": LaunchConfiguration(
                                "enable_visual_initialization"
                            ),
                            "visual_model_file": LaunchConfiguration(
                                "visual_model_file"
                            ),
                            "visual_database_file": LaunchConfiguration(
                                "visual_database_file"
                            ),
                            "enable_fpfh": LaunchConfiguration("enable_fpfh"),
                            "allow_missing_georeference": LaunchConfiguration(
                                "allow_missing_georeference"
                            ),
                            "right_camera_device": LaunchConfiguration(
                                "right_camera_device"
                            ),
                        }.items(),
                    )
                ],
            ),
            obstacle_height_filter,
            navigation,
            chassis,
            Node(
                package="rviz2",
                executable="rviz2",
                name="differential_fastlivo_rtk_navigation_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
