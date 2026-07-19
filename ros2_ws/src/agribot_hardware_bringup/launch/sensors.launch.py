import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _static_transform(parent, child, xyz, rpy):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child}_static_tf",
        arguments=[
            "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
            "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )


def _launch_setup(context):
    mount_file = LaunchConfiguration("mount_config").perform(context)
    with open(mount_file, "r", encoding="utf-8") as stream:
        mounts = yaml.safe_load(stream)

    actions = []
    for sensor_name in ("imu", "lidar", "rtk"):
        mount = mounts[sensor_name]
        actions.append(
            _static_transform(
                "base_link", mount["child_frame"], mount["xyz"], mount["rpy"]
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("agribot_hardware_bringup")
    hipnuc_share = get_package_share_directory("hipnuc_imu")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "lidar_config", default_value=os.path.join(share, "config", "c16.yaml")
            ),
            DeclareLaunchArgument(
                "imu_config", default_value=os.path.join(hipnuc_share, "config", "n300pro.yaml")
            ),
            DeclareLaunchArgument(
                "rtk_config", default_value=os.path.join(share, "config", "rtk_nmea.yaml")
            ),
            DeclareLaunchArgument(
                "mount_config",
                default_value=os.path.join(share, "config", "sensor_mounts.yaml"),
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("ntrip_port", default_value="8002"),
            OpaqueFunction(function=_launch_setup),
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                name="lslidar_driver_node",
                output="screen",
                parameters=[LaunchConfiguration("lidar_config")],
                condition=IfCondition(LaunchConfiguration("start_lidar")),
            ),
            Node(
                package="hipnuc_imu",
                executable="hipnuc_imu_node",
                name="hipnuc_imu",
                output="screen",
                parameters=[LaunchConfiguration("imu_config")],
                condition=IfCondition(LaunchConfiguration("start_imu")),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="rtk_nmea_node.py",
                name="rtk_nmea",
                output="screen",
                parameters=[
                    LaunchConfiguration("rtk_config"),
                    {
                        "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                        "ntrip_host": EnvironmentVariable("NTRIP_HOST", default_value=""),
                        "ntrip_port": LaunchConfiguration("ntrip_port"),
                        "ntrip_mountpoint": EnvironmentVariable(
                            "NTRIP_MOUNTPOINT", default_value=""
                        ),
                        "ntrip_username": EnvironmentVariable(
                            "NTRIP_USERNAME", default_value=""
                        ),
                        "ntrip_password": EnvironmentVariable(
                            "NTRIP_PASSWORD", default_value=""
                        ),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_rtk")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "rviz", "sensors.rviz")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
