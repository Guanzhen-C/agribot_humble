import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    differential_config = os.path.join(hardware_share, "differential", "config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base",
                description="不带扩展名的地图输出绝对路径",
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
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("map_start_delay", default_value="5.0"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument(
                "bag_output", default_value="/tmp/agribot_differential_mapping"
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share, "launch", "vehicle_autonomy.launch.py"
                    )
                ),
                launch_arguments={
                    "vehicle_type": "differential",
                    "controller": "mppi",
                    "localization": "fastlio",
                    "navigation_mode": "mapping",
                    "start_rtk": LaunchConfiguration("start_rtk"),
                    "start_navigation": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": os.path.join(
                        hardware_share, "rviz", "pcd_mapping.rviz"
                    ),
                    "map_start_delay": LaunchConfiguration("map_start_delay"),
                    "pcd_map_base": LaunchConfiguration("map_base"),
                    "pcd_mapping_config": os.path.join(
                        differential_config, "pcd_mapping.yaml"
                    ),
                    "fastlio_bridge_config": os.path.join(
                        differential_config, "fastlio_bridge.yaml"
                    ),
                    "fastlio_config": os.path.join(
                        differential_config, "fast_lio_c16.yaml"
                    ),
                    "rtk_config": os.path.join(
                        differential_config, "rtk_nmea.yaml"
                    ),
                    "mount_config": os.path.join(
                        differential_config, "sensor_mounts.yaml"
                    ),
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                    "map": "",
                    "enable_chassis_output": "false",
                    "chassis_driver": "none",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share, "launch", "include", "right_camera.launch.py"
                    )
                ),
                launch_arguments={
                    "camera_driver": LaunchConfiguration("camera_driver"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
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
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    LaunchConfiguration("bag_output"),
                    "/lidar/points",
                    "/imu/data",
                    "/rtk/fix",
                    "/rtk/fix_quality",
                    "/rtk/heading_with_covariance",
                    "/rtk/heading_solution",
                    "/camera/rgb/image_raw",
                    "/camera/rgb/camera_info",
                    "/Odometry",
                    "/fastlio/odometry",
                    "/cloud_registered",
                    "/diagnostics",
                    "/tf",
                    "/tf_static",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_bag")),
            ),
        ]
    )
