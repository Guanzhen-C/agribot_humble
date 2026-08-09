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
from launch_ros.actions import Node
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
    "/teleop/cmd_vel",
    "/wheel/odometry",
    "/scout_status",
    "/hardware/chassis_e_stop",
    "/diagnostics",
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
            DeclareLaunchArgument("enable_chassis_output", default_value="true"),
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
                "command_input_topic", default_value="/teleop/cmd_vel"
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
            Node(
                package="agribot_hardware_bringup",
                executable="ackermann_chassis_can_node",
                name="ackermann_collection_chassis_can",
                output="screen",
                parameters=[
                    os.path.join(
                        hardware_share, "ackermann", "config", "chassis_can.yaml"
                    ),
                    {
                        "can_transport": LaunchConfiguration("can_transport"),
                        "can_interface": LaunchConfiguration("can_interface"),
                        "zqwl_port": LaunchConfiguration("zqwl_port"),
                        "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                        "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                        "command_topic": LaunchConfiguration("command_input_topic"),
                        "require_localization_ready": False,
                    },
                ],
                condition=IfCondition(
                    LaunchConfiguration("enable_chassis_output")
                ),
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
