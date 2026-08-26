import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    sensors_launch = os.path.join(hardware_share, "launch", "sensors.launch.py")
    mount_config = os.path.join(
        hardware_share, "differential", "config", "sensor_mounts.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
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
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(sensors_launch),
                        launch_arguments={
                            "start_lidar": LaunchConfiguration("start_lidar"),
                            "start_imu": LaunchConfiguration("start_imu"),
                            "start_rtk": LaunchConfiguration("start_rtk"),
                            "lidar_forward_point_offset_sec": LaunchConfiguration(
                                "lidar_forward_point_offset_sec"
                            ),
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "mount_config": mount_config,
                            "rtk_config": os.path.join(
                                hardware_share,
                                "differential",
                                "config",
                                "rtk_nmea.yaml",
                            ),
                            "rviz": "false",
                        }.items(),
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
                    "hikrobot_camera_serial": LaunchConfiguration(
                        "hikrobot_camera_serial"
                    ),
                    "hikrobot_trigger_enable": LaunchConfiguration(
                        "hikrobot_trigger_enable"
                    ),
                    "right_camera_device": LaunchConfiguration(
                        "right_camera_device"
                    ),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(hardware_share, "rviz", "sensors.rviz"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
