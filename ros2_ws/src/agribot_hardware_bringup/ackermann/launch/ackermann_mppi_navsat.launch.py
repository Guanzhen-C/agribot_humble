import os
import math
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _fingerprint_file(path):
    value = 14695981039346656037
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            for byte in chunk:
                value ^= byte
                value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def _launch_georeferenced_navsat(context, *, hardware_share):
    georeference_path = Path(
        LaunchConfiguration("map_georeference").perform(context)
    )
    map_path = Path(LaunchConfiguration("map").perform(context))
    initialization_source = LaunchConfiguration(
        "initialization_source", default="rtk"
    ).perform(context)
    if initialization_source not in ("manual", "rtk"):
        raise RuntimeError(
            "NavSat mapped localization supports initialization_source:=manual or rtk"
        )
    if not georeference_path.is_file():
        raise RuntimeError(
            f"map georeference file does not exist: {georeference_path}"
        )
    document = yaml.safe_load(georeference_path.read_text(encoding="utf-8"))
    try:
        if int(document["schema_version"]) != 1:
            raise RuntimeError("unsupported map georeference schema version")
        map_id = str(document["map"]["id"])
        map_fingerprint = str(document["map"]["fingerprint_fnv1a64"])
        reference = document["reference"]
        transform = document["map_from_enu"]
        calibration = document["calibration"]
        xyz = [float(value) for value in transform["xyz"]]
        rpy = [float(value) for value in transform["rpy"]]
        latitude = float(reference["latitude_deg"])
        longitude = float(reference["longitude_deg"])
        altitude = float(reference["altitude_m"])
        horizontal_rmse = float(calibration["horizontal_rmse_m"])
        yaw_rmse = float(calibration["yaw_rmse_deg"])
        yaw_validation_passed = calibration.get(
            "yaw_validation_passed", True
        )
        if not isinstance(yaw_validation_passed, bool):
            raise ValueError("yaw_validation_passed must be boolean")
        sample_count = int(calibration["sample_count"])
        calibration_version = str(calibration["version"])
        calibration_hash = str(calibration["hash"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(f"invalid map georeference: {error}") from error
    if len(xyz) != 3 or len(rpy) != 3 or not all(
        math.isfinite(value)
        for value in xyz + rpy + [latitude, longitude, altitude]
    ):
        raise RuntimeError("map georeference contains invalid coordinates")
    if abs(rpy[0]) > 1.0e-6 or abs(rpy[1]) > 1.0e-6:
        raise RuntimeError("map georeference must be a planar yaw transform")
    if sample_count < 2 or not calibration_version or not calibration_hash:
        raise RuntimeError("map georeference has incomplete calibration provenance")
    if map_path.stem != map_id:
        raise RuntimeError(
            f"Nav2 map '{map_path.stem}' does not match georeference map ID '{map_id}'"
        )
    pcd_path = map_path.with_suffix(".pcd")
    if not pcd_path.is_file():
        raise RuntimeError(f"matching PCD map does not exist: {pcd_path}")
    if _fingerprint_file(pcd_path) != map_fingerprint:
        raise RuntimeError("PCD map fingerprint does not match map georeference")
    if (
        not math.isfinite(horizontal_rmse)
        or not math.isfinite(yaw_rmse)
        or horizontal_rmse < 0.0
        or yaw_rmse < 0.0
        or horizontal_rmse > 0.20
    ):
        raise RuntimeError(
            "map georeference does not meet the NavSat horizontal runtime limit"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_share, "launch", "vehicle_autonomy.launch.py")
            ),
            launch_arguments={
                "vehicle_type": "ackermann",
                "controller": "mppi",
                "localization": "navsat",
                "navigation_mode": "localization",
                "start_rtk": "true",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": LaunchConfiguration("autostart"),
                "start_sensors": LaunchConfiguration("start_sensors"),
                "rviz": LaunchConfiguration("rviz"),
                "use_detailed_vehicle_model": LaunchConfiguration(
                    "use_detailed_vehicle_model"
                ),
                "navigation_delay": LaunchConfiguration("navigation_delay"),
                "map_start_delay": LaunchConfiguration("map_start_delay"),
                "map": LaunchConfiguration("map"),
                "pcd_map_file": str(pcd_path),
                "map_georeference_file": LaunchConfiguration(
                    "map_georeference"
                ),
                "initialization_source": LaunchConfiguration(
                    "initialization_source"
                ),
                "mapped_initial_pose_topic": PythonExpression(
                    [
                        "'/localization/rtk_initialpose' if '",
                        LaunchConfiguration("initialization_source"),
                        "' == 'rtk' else '/initialpose'",
                    ]
                ),
                "mapped_odometry_topic": "/odometry/filtered_navsat",
                "enable_fpfh": "false",
                "automatic_global_localization": "false",
                "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                "require_localization_ready": "true",
                "navsat_output_frame": "odom",
                "navsat_pose_topic": "/navsat/filtered_pose",
                "navsat_tf_mode": "odom_to_base_only",
                "navsat_publish_readiness": "true",
                "navsat_ready_topic": "/localization/navsat_ready",
                "navsat_auto_reference_from_first_fix": "false",
                "navsat_reference_latitude_deg": str(latitude),
                "navsat_reference_longitude_deg": str(longitude),
                "navsat_reference_altitude_m": str(altitude),
                "navsat_reference_map_x": str(xyz[0]),
                "navsat_reference_map_y": str(xyz[1]),
                "navsat_reference_map_z": str(xyz[2]),
                "navsat_map_from_enu_yaw_deg": str(math.degrees(rpy[2])),
                "enable_can_output": LaunchConfiguration("enable_can_output"),
                "enable_chassis_output": LaunchConfiguration(
                    "enable_chassis_output"
                ),
                "chassis_driver": LaunchConfiguration("chassis_driver"),
                "can_transport": LaunchConfiguration("can_transport"),
                "can_interface": LaunchConfiguration("can_interface"),
                "zqwl_port": LaunchConfiguration("zqwl_port"),
                "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                "serial_port": LaunchConfiguration("serial_port"),
                "command_input_topic": LaunchConfiguration("command_input_topic"),
            }.items(),
        )
    ]


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_detailed_vehicle_model", default_value="false"
            ),
            DeclareLaunchArgument("navigation_delay", default_value="8.0"),
            DeclareLaunchArgument("map_start_delay", default_value="5.0"),
            DeclareLaunchArgument(
                "map", description="Absolute path to the real-vehicle Nav2 map YAML"
            ),
            DeclareLaunchArgument(
                "map_georeference",
                description="Georeference YAML generated with the same PCD/Nav2 map",
            ),
            DeclareLaunchArgument(
                "initialization_source",
                default_value="rtk",
                description="Use fixed RTK or RViz manual pose before NDT/GICP",
            ),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument("enable_can_output", default_value="false"),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value=LaunchConfiguration("enable_can_output"),
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
            DeclareLaunchArgument("zqwl_channel", default_value="0"),
            DeclareLaunchArgument("zqwl_bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5C2C079857-if00"
                ),
            ),
            DeclareLaunchArgument(
                "command_input_topic", default_value="/nav2/cmd_vel"
            ),
            OpaqueFunction(
                function=_launch_georeferenced_navsat,
                kwargs={"hardware_share": hardware_share},
            ),
        ]
    )
