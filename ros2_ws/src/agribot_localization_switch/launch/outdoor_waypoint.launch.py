"""Reuse the existing differential safety/navigation stack with a localization slot."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hardware = Path(get_package_share_directory("agribot_hardware_bringup"))
    own = Path(get_package_share_directory("agribot_localization_switch"))
    defaults = {"use_sim_time": "false", "rviz": "false", "enable_chassis_output": "false",
        "motion_authorization": "", "initialization_source": "auto", "enable_rtk_initialization": "true",
        "enable_visual_initialization": "true", "enable_fpfh": "false", "allow_missing_georeference": "false",
        "initialization_refinement": "rtk_only", "initialization_wait_for_request": "true",
        "allow_uncalibrated_camera": "false", "enable_ntrip": "false", "can_transport": "zqwl_cdc",
        "zqwl_port": "/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00",
        "can_interface": "can0", "zqwl_channel": "0", "zqwl_bitrate": "250000"}
    args = {key: LaunchConfiguration(key) for key in defaults}
    args["map_base"] = LaunchConfiguration("map_base")
    args["localization_launch_file"] = str(own/"launch/localization.launch.py")
    return LaunchDescription([DeclareLaunchArgument("map_base")]+
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]+[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware/"launch/differential_outdoor_experiment.launch.py")),
                                     launch_arguments=args.items())])
