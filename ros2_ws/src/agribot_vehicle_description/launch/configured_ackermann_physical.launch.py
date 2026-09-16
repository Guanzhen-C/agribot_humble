import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory("agribot_vehicle_description")
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    generated = os.path.join(description_share, "generated", "ackermann_current")
    physical = os.path.join(generated, "config", "physical")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base", description="不带扩展名的三维和二维地图绝对路径"
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("start_rtk", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("use_detailed_vehicle_model", default_value="false"),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value="false",
                description="仿真验收和静态真机检查完成后才可显式设为true",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share,
                        "launch",
                        "ackermann_mppi_fastlivo_rtk_mapped.launch.py",
                    )
                ),
                launch_arguments={
                    "map_base": LaunchConfiguration("map_base"),
                    "rviz": LaunchConfiguration("rviz"),
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "start_rtk": LaunchConfiguration("start_rtk"),
                    "start_camera": LaunchConfiguration("start_camera"),
                    "start_navigation": LaunchConfiguration("start_navigation"),
                    "use_detailed_vehicle_model": LaunchConfiguration(
                        "use_detailed_vehicle_model"
                    ),
                    "enable_chassis_output": LaunchConfiguration(
                        "enable_chassis_output"
                    ),
                    "chassis_driver": LaunchConfiguration("chassis_driver"),
                    "can_transport": LaunchConfiguration("can_transport"),
                    "can_interface": LaunchConfiguration("can_interface"),
                    "zqwl_port": LaunchConfiguration("zqwl_port"),
                    "nav2_params": os.path.join(
                        physical, "nav2_params_ackermann_fastlio_mapped.yaml"
                    ),
                    "mount_config": os.path.join(physical, "sensor_mounts.yaml"),
                    "fastlivo_lidar_config": os.path.join(
                        physical, "agribot_c16_astra.yaml"
                    ),
                    "fastlivo_bridge_config": os.path.join(
                        physical, "fastlivo_bridge.yaml"
                    ),
                    "pcd_initial_localization_config": os.path.join(
                        physical, "pcd_initial_localization.yaml"
                    ),
                    "rtk_map_initializer_config": os.path.join(
                        physical, "rtk_map_initializer.yaml"
                    ),
                    "fastlivo_rtk_fusion_config": os.path.join(
                        physical, "fastlivo_rtk_fusion.yaml"
                    ),
                    "robot_description_file": os.path.join(
                        generated, "urdf", "ackermann_current.urdf.xacro"
                    ),
                    "chassis_can_config": os.path.join(
                        physical, "chassis_can.yaml"
                    ),
                    "chassis_serial_config": os.path.join(
                        physical, "chassis_serial.yaml"
                    ),
                    "joint_state_config": os.path.join(
                        physical, "joint_state_publisher.yaml"
                    ),
                }.items(),
            ),
        ]
    )
