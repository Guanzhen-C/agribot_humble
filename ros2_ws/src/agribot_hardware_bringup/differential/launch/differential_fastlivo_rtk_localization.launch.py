import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    differential_config = os.path.join(hardware_share, "differential", "config")
    common_launch = os.path.join(
        hardware_share, "launch", "include", "fastlivo_rtk_localization.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base", description="不带扩展名的三维和二维地图绝对路径"
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
            DeclareLaunchArgument(
                "camera_calibration_status",
                default_value=os.path.join(
                    differential_config,
                    "hikrobot_camera_calibration_status.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "allow_uncalibrated_camera", default_value="false"
            ),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("start_initial_localizer", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "initialization_source", default_value="manual"
            ),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference", default_value="true"
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(common_launch),
                        launch_arguments={
                            "map_base": LaunchConfiguration("map_base"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "start_sensors": LaunchConfiguration("start_sensors"),
                            "start_rtk": LaunchConfiguration("start_rtk"),
                            "start_camera": LaunchConfiguration("start_camera"),
                            "camera_driver": LaunchConfiguration("camera_driver"),
                            "hikrobot_camera_serial": LaunchConfiguration(
                                "hikrobot_camera_serial"
                            ),
                            "hikrobot_trigger_enable": LaunchConfiguration(
                                "hikrobot_trigger_enable"
                            ),
                            "camera_calibration_status": LaunchConfiguration(
                                "camera_calibration_status"
                            ),
                            "allow_uncalibrated_camera": LaunchConfiguration(
                                "allow_uncalibrated_camera"
                            ),
                            "start_fastlivo": LaunchConfiguration("start_fastlivo"),
                            "start_initial_localizer": LaunchConfiguration(
                                "start_initial_localizer"
                            ),
                            "rviz": LaunchConfiguration("rviz"),
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "use_detailed_vehicle_model": "false",
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
                            "mount_config": os.path.join(
                                differential_config, "sensor_mounts.yaml"
                            ),
                            "rtk_config": os.path.join(
                                differential_config, "rtk_nmea.yaml"
                            ),
                            "fastlivo_lidar_config": os.path.join(
                                differential_config, "fastlivo_c16_camera.yaml"
                            ),
                            "fastlivo_bridge_config": os.path.join(
                                differential_config, "fastlivo_bridge.yaml"
                            ),
                            "pcd_initial_localization_config": os.path.join(
                                differential_config, "pcd_initial_localization.yaml"
                            ),
                            "rtk_map_initializer_config": os.path.join(
                                differential_config, "rtk_map_initializer.yaml"
                            ),
                            "fastlivo_rtk_fusion_config": os.path.join(
                                differential_config, "fastlivo_rtk_fusion.yaml"
                            ),
                        }.items(),
                    )
                ],
            ),
        ]
    )
