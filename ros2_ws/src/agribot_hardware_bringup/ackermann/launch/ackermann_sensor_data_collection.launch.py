import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


SENSOR_TOPICS = (
    "/lidar/points",
    "/lslidar_device_info",
    "/time_topic",
    "/imu/data",
    "/imu/magnetic_field",
    "/imu/temperature",
    "/rtk/raw_sentence",
    "/rtk/fix",
    "/rtk/fix_quality",
    "/rtk/gga_utc",
    "/rtk/satellite_count",
    "/rtk/hdop",
    "/rtk/differential_age",
    "/rtk/reference_station_id",
    "/rtk/heading",
    "/rtk/heading_deg",
    "/rtk/heading_valid",
    "/rtk/heading_solution",
    "/rtk/heading_with_covariance",
    "/camera/rgb/image_raw",
    "/camera/rgb/camera_info",
    "/camera/depth/image_raw",
    "/camera/depth/camera_info",
    "/tf",
    "/tf_static",
)


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("ntrip_port", default_value="8002"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_output", default_value="/tmp/agribot_sensor_data"
            ),
            DeclareLaunchArgument(
                "openni2_redist", default_value="/opt/orbbec/openni2"
            ),
            SetEnvironmentVariable(
                "OPENNI2_REDIST", LaunchConfiguration("openni2_redist")
            ),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    LaunchConfiguration("openni2_redist"),
                    ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hardware_share, "launch", "sensors.launch.py")
                ),
                launch_arguments={
                    "start_lidar": LaunchConfiguration("start_lidar"),
                    "start_imu": LaunchConfiguration("start_imu"),
                    "start_rtk": LaunchConfiguration("start_rtk"),
                    "rviz": "false",
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                    "ntrip_port": LaunchConfiguration("ntrip_port"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("openni2_camera"),
                            "launch",
                            "camera_only.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    LaunchConfiguration("bag_output"),
                    *SENSOR_TOPICS,
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_bag")),
            ),
        ]
    )
