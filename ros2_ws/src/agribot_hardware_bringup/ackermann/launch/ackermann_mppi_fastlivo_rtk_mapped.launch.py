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
    chassis_driver = LaunchConfiguration("chassis_driver").perform(context)
    enable_chassis_output = LaunchConfiguration(
        "enable_chassis_output"
    ).perform(context).lower() in ("true", "1", "yes", "on")
    if chassis_driver not in ("ackermann_can", "ackermann_serial"):
        raise RuntimeError(
            "chassis_driver必须是ackermann_can或ackermann_serial"
        )
    if enable_chassis_output and chassis_driver == "ackermann_can":
        can_transport = LaunchConfiguration("can_transport").perform(context)
        if can_transport not in ("socketcan", "zqwl_cdc"):
            raise RuntimeError(
                "can_transport必须是socketcan或zqwl_cdc"
            )
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    localization_launch = os.path.join(
        hardware_share,
        "launch",
        "ackermann_fastlivo_rtk_localization.launch.py",
    )
    navigation_launch = os.path.join(
        hardware_share, "launch", "include", "navigation_only.launch.py"
    )

    navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": LaunchConfiguration("autostart"),
                    "params_file": os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "nav2_params_ackermann_fastlio_mapped.yaml",
                    ),
                    "odom_topic": "/fastlivo_rtk/odometry",
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
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    chassis = GroupAction(
        actions=[
            Node(
                package="agribot_hardware_bringup",
                executable="ackermann_chassis_can_node",
                name="ackermann_chassis_can",
                output="screen",
                parameters=[
                    os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "chassis_can.yaml",
                    ),
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
                    os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "chassis_serial.yaml",
                    ),
                    {
                        "use_sim_time": use_sim_time,
                        "port": LaunchConfiguration("serial_port"),
                        "command_topic": "/nav2/cmd_vel",
                        "require_localization_ready": True,
                        "localization_ready_topic": "/fastlivo_rtk/ready",
                    },
                ],
                condition=LaunchConfigurationEquals(
                    "chassis_driver", "ackermann_serial"
                ),
            ),
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
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="8.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    hardware_share, "rviz", "navigation.rviz"
                ),
                description="RViz配置文件；全流程默认包含Navigation 2面板",
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "use_detailed_vehicle_model", default_value="false"
            ),
            DeclareLaunchArgument(
                "initialization_source",
                default_value="manual",
                description=(
                    "manual使用RViz粗位姿后执行NDT/GICP；rtk使用固定解粗位姿；"
                    "lidar执行FPFH全局粗定位"
                ),
            ),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference",
                default_value="true",
                description=(
                    "没有地理配准文件时仍运行FAST-LIVO2融合链，但不接收RTK因子"
                ),
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value="false",
                description="显式开启后才向真车底盘发送Nav2控制命令",
            ),
            DeclareLaunchArgument("chassis_driver", default_value="ackermann_can"),
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
            DeclareLaunchArgument("zqwl_bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5C2C079857-if00"
                ),
            ),
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
                            "start_camera": LaunchConfiguration("start_camera"),
                            "start_fastlivo": LaunchConfiguration("start_fastlivo"),
                            "start_initial_localizer": "true",
                            "rviz": "false",
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "use_detailed_vehicle_model": LaunchConfiguration(
                                "use_detailed_vehicle_model"
                            ),
                            "initialization_source": LaunchConfiguration(
                                "initialization_source"
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
            navigation,
            chassis,
            Node(
                package="agribot_hardware_bringup",
                executable="ackermann_joint_state_publisher",
                name="ackermann_joint_state_publisher",
                output="screen",
                parameters=[
                    os.path.join(
                        hardware_share,
                        "ackermann",
                        "config",
                        "joint_state_publisher.yaml",
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("use_detailed_vehicle_model"),
                            "'.lower() in ('true', '1', 'yes', 'on')",
                        ]
                    )
                ),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="ackermann_fastlivo_rtk_navigation_rviz",
                        arguments=[
                            "-d",
                            LaunchConfiguration("rviz_config"),
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
