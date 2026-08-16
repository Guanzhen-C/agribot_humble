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
            DeclareLaunchArgument("start_camera", default_value="true"),
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
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    os.path.join(hardware_share, "config", "right_camera.yaml"),
                    {"video_device": LaunchConfiguration("right_camera_device")},
                ],
                remappings=[
                    ("image_raw", "/camera/rgb/image_raw"),
                    ("camera_info", "/camera/rgb/camera_info"),
                ],
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
