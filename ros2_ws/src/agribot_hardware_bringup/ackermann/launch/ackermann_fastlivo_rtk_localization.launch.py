import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    common_launch = os.path.join(
        hardware_share, "launch", "include", "fastlivo_rtk_localization.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base", description="不带扩展名的地图绝对路径"
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
                "hikrobot_trigger_enable", default_value="true"
            ),
            DeclareLaunchArgument(
                "camera_calibration_status",
                default_value=os.path.join(
                    hardware_share,
                    "ackermann",
                    "config",
                    "hikrobot_camera_calibration_status.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "allow_uncalibrated_camera", default_value="false"
            ),
            DeclareLaunchArgument("start_fastlivo", default_value="true"),
            DeclareLaunchArgument("fastlivo_dense_map", default_value="false"),
            DeclareLaunchArgument(
                "fastlivo_map_sliding_en", default_value="true"
            ),
            DeclareLaunchArgument("start_initial_localizer", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "use_detailed_vehicle_model", default_value="false"
            ),
            DeclareLaunchArgument(
                "initialization_source", default_value="auto"
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
                "allow_missing_georeference", default_value="false"
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
                            "fastlivo_dense_map": LaunchConfiguration(
                                "fastlivo_dense_map"
                            ),
                            "fastlivo_map_sliding_en": LaunchConfiguration(
                                "fastlivo_map_sliding_en"
                            ),
                            "start_initial_localizer": LaunchConfiguration(
                                "start_initial_localizer"
                            ),
                            "rviz": LaunchConfiguration("rviz"),
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "use_detailed_vehicle_model": LaunchConfiguration(
                                "use_detailed_vehicle_model"
                            ),
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
                            "mount_config": os.path.join(
                                hardware_share, "config", "sensor_mounts.yaml"
                            ),
                            "fastlivo_bridge_config": os.path.join(
                                hardware_share, "config", "fastlivo_bridge.yaml"
                            ),
                            "pcd_initial_localization_config": os.path.join(
                                hardware_share,
                                "ackermann",
                                "config",
                                "pcd_initial_localization.yaml",
                            ),
                            "rtk_map_initializer_config": os.path.join(
                                hardware_share, "config", "rtk_map_initializer.yaml"
                            ),
                            "fastlivo_rtk_fusion_config": os.path.join(
                                hardware_share, "config", "fastlivo_rtk_fusion.yaml"
                            ),
                            "robot_description_file": os.path.join(
                                hardware_share, "urdf", "ackermann_vehicle.urdf"
                            ),
                            "robot_state_publisher_name": (
                                "ackermann_robot_state_publisher"
                            ),
                        }.items(),
                    )
                ],
            ),
        ]
    )
