"""Sensors and map stay running while the estimator processes change."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    hardware = Path(get_package_share_directory("agribot_hardware_bringup"))
    config = hardware/"differential/config"
    value = lambda name: LaunchConfiguration(name).perform(context)
    base = Path(value("map_base")).expanduser().resolve()
    for suffix in (".yaml", ".pgm", ".pcd", "_georeference.yaml"):
        if not Path(str(base)+suffix).is_file():
            raise RuntimeError("Map file missing: "+str(base)+suffix)
    sim = value("use_sim_time").lower() == "true"
    args = {name: value(name) for name in ("start_rtk", "lidar_forward_point_offset_sec", "start_camera",
        "camera_driver", "hikrobot_camera_serial", "hikrobot_trigger_enable", "right_camera_device", "enable_ntrip")}
    args["rviz"] = "false"
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware/"launch/differential_sensor_validation.launch.py")),
                                 launch_arguments=args.items()),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[{"use_sim_time": sim, "yaml_filename": str(base)+".yaml"}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_switch_map",
             parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["map_server"]}]),
        Node(package="agribot_localization_switch", executable="localization_switch", output="screen",
             parameters=[{"use_sim_time": sim, "map_base": str(base),
                          "wait_for_request": value("initialization_wait_for_request").lower() == "true"}]),
    ]


def generate_launch_description():
    defaults = {"use_sim_time": "false", "start_rtk": "true", "start_camera": "true",
        "lidar_forward_point_offset_sec": "0.05004", "camera_driver": "hikrobot_mvs",
        "hikrobot_camera_serial": "DB0447659", "hikrobot_trigger_enable": "true",
        "right_camera_device": "/dev/agribot_right_camera", "enable_ntrip": "false",
        "initialization_wait_for_request": "true"}
    return LaunchDescription([DeclareLaunchArgument("map_base")]+
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]+[OpaqueFunction(function=configure)])
