import fcntl
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_RUNTIME_LOCK = None
REQUIRED_SENSOR_DEVICES = (
    (
        "RTK",
        Path(
            "/dev/serial/by-id/"
            "usb-AirM2M_AirM2M_Compo_000000000001-if06"
        ),
    ),
    (
        "IMU",
        Path(
            "/dev/serial/by-id/"
            "usb-1a86_USB_Single_Serial_5C2C082600-if00"
        ),
    ),
)


def _validate_outdoor_arguments(context):
    locked = {
        "initialization_source": "rtk",
        "enable_fpfh": "false",
        "allow_missing_georeference": "false",
    }
    for name, expected in locked.items():
        actual = LaunchConfiguration(name).perform(context).lower()
        if actual != expected:
            raise RuntimeError(f"差速室外入口要求{name}:={expected}，实际为{actual}")

    for sensor_name, device in REQUIRED_SENSOR_DEVICES:
        if not device.exists():
            raise RuntimeError(f"{sensor_name}串口设备不存在: {device}")
    camera = Path(LaunchConfiguration("right_camera_device").perform(context))
    if not camera.exists():
        raise RuntimeError(f"FAST-LIVO2右目相机设备不存在: {camera}")
    return []


def _acquire_runtime_lock(_context):
    global _RUNTIME_LOCK
    lock = open("/tmp/agribot_differential_outdoor.lock", "w", encoding="ascii")
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock.close()
        raise RuntimeError("已有一套差速室外全流程正在运行") from error
    lock.write(f"{os.getpid()}\n")
    lock.flush()
    _RUNTIME_LOCK = lock
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    full_launch = os.path.join(
        hardware_share,
        "launch",
        "differential_mppi_fastlivo_rtk_mapped.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base", description="不带扩展名的室外地图绝对路径"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("initialization_source", default_value="rtk"),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference", default_value="false"
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            DeclareLaunchArgument(
                "enable_chassis_output", default_value="false"
            ),
            DeclareLaunchArgument("motion_authorization", default_value=""),
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
            OpaqueFunction(function=_validate_outdoor_arguments),
            OpaqueFunction(function=_acquire_runtime_lock),
            LogInfo(
                msg=[
                    "差速室外全流程启动；底盘输出=",
                    LaunchConfiguration("enable_chassis_output"),
                    "。车辆保持静止，等待RTK粗定位和NDT/GICP精配准。",
                ]
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(full_launch),
                        launch_arguments={
                            "map_base": LaunchConfiguration("map_base"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "start_sensors": "true",
                            "start_rtk": "true",
                            "start_camera": "true",
                            "start_fastlivo": "true",
                            "start_navigation": "true",
                            "rviz": LaunchConfiguration("rviz"),
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
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
                            "enable_chassis_output": LaunchConfiguration(
                                "enable_chassis_output"
                            ),
                            "motion_authorization": LaunchConfiguration(
                                "motion_authorization"
                            ),
                            "chassis_driver": "differential_can",
                            "can_transport": LaunchConfiguration("can_transport"),
                            "can_interface": LaunchConfiguration("can_interface"),
                            "zqwl_port": LaunchConfiguration("zqwl_port"),
                            "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                            "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
