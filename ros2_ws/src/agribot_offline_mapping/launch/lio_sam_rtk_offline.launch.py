import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml


def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _inverse_mount(xyz, rpy):
    rotation = _rotation_from_rpy(*rpy)
    inverse_rotation = tuple(zip(*rotation))
    inverse_translation = tuple(
        -sum(inverse_rotation[row][column] * xyz[column] for column in range(3))
        for row in range(3)
    )
    pitch = math.asin(max(-1.0, min(1.0, -inverse_rotation[2][0])))
    roll = math.atan2(inverse_rotation[2][1], inverse_rotation[2][2])
    yaw = math.atan2(inverse_rotation[1][0], inverse_rotation[0][0])
    return inverse_translation, (roll, pitch, yaw)


def _sensor_transform_nodes(context):
    path = Path(
        LaunchConfiguration("sensor_mounts_file").perform(context)
    ).expanduser()
    if not path.is_file():
        raise RuntimeError(f"sensor mounts file not found: {path}")
    try:
        mounts = yaml.safe_load(path.read_text(encoding="utf-8"))
        imu_xyz = tuple(float(value) for value in mounts["imu"]["xyz"])
        imu_rpy = tuple(float(value) for value in mounts["imu"]["rpy"])
        lidar_xyz = tuple(float(value) for value in mounts["lidar"]["xyz"])
        lidar_rpy = tuple(float(value) for value in mounts["lidar"]["rpy"])
        rtk_xyz = tuple(float(value) for value in mounts["rtk"]["xyz"])
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise RuntimeError(f"invalid sensor mounts file {path}: {error}") from error
    lidar_to_base_xyz, lidar_to_base_rpy = _inverse_mount(
        lidar_xyz, lidar_rpy
    )
    base_from_lidar = _rotation_from_rpy(*lidar_rpy)
    lidar_from_base = tuple(zip(*base_from_lidar))
    antenna_minus_lidar = tuple(
        rtk_xyz[index] - lidar_xyz[index] for index in range(3)
    )
    lidar_to_antenna = tuple(
        sum(
            lidar_from_base[row][column] * antenna_minus_lidar[column]
            for column in range(3)
        )
        for row in range(3)
    )

    def values(items):
        return [f"{value:.12g}" for value in items]

    return [
        Node(
            package="agribot_offline_mapping",
            executable="map_georeference_exporter",
            output="screen",
            parameters=[
                LaunchConfiguration("georeference_parameters"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {
                    "map_pcd_file": PythonExpression(
                        ["'", LaunchConfiguration("map_base"), ".pcd'"]
                    ),
                    "output_file": PythonExpression(
                        ["'", LaunchConfiguration("map_base"), "_georeference.yaml'"]
                    ),
                    "source_bag": LaunchConfiguration("source_bag"),
                    "lidar_to_antenna_m": list(lidar_to_antenna),
                    "lidar_to_base_rpy": list(lidar_to_base_rpy),
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_rtk_components")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="offline_base_to_imu",
            arguments=[
                "--x", *values(imu_xyz[:1]),
                "--y", *values(imu_xyz[1:2]),
                "--z", *values(imu_xyz[2:]),
                "--roll", *values(imu_rpy[:1]),
                "--pitch", *values(imu_rpy[1:2]),
                "--yaw", *values(imu_rpy[2:]),
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="offline_lidar_to_base",
            arguments=[
                "--x", *values(lidar_to_base_xyz[:1]),
                "--y", *values(lidar_to_base_xyz[1:2]),
                "--z", *values(lidar_to_base_xyz[2:]),
                "--roll", *values(lidar_to_base_rpy[:1]),
                "--pitch", *values(lidar_to_base_rpy[1:2]),
                "--yaw", *values(lidar_to_base_rpy[2:]),
                "--frame-id", "lidar_link",
                "--child-frame-id", "base_link",
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("agribot_offline_mapping")
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common_parameters = {"use_sim_time": use_sim_time}
    lio_sam_parameters = LaunchConfiguration("lio_sam_parameters")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base",
                description="Absolute output map path without an extension",
            ),
            DeclareLaunchArgument("source_bag", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_lio_sam", default_value="true"),
            DeclareLaunchArgument("start_rtk_components", default_value="true"),
            DeclareLaunchArgument(
                "auto_reference_from_first_fix", default_value="true"
            ),
            DeclareLaunchArgument(
                "reference_latitude_deg", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "reference_longitude_deg", default_value="0.0"
            ),
            DeclareLaunchArgument("reference_altitude_m", default_value="0.0"),
            DeclareLaunchArgument(
                "lio_sam_parameters",
                default_value=os.path.join(
                    package_share, "config", "lio_sam_c16.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "point_adapter_parameters",
                default_value=os.path.join(
                    package_share, "config", "lslidar_lio_sam_adapter.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rtk_adapter_parameters",
                default_value=os.path.join(
                    package_share, "config", "rtk_odometry_adapter.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "georeference_parameters",
                default_value=os.path.join(
                    package_share, "config", "map_georeference_exporter.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "sensor_mounts_file",
                default_value=os.path.join(
                    hardware_share, "config", "sensor_mounts.yaml"
                ),
            ),
            Node(
                package="agribot_offline_mapping",
                executable="lslidar_lio_sam_adapter",
                output="screen",
                parameters=[
                    LaunchConfiguration("point_adapter_parameters"),
                    common_parameters,
                ],
            ),
            Node(
                package="agribot_offline_mapping",
                executable="rtk_odometry_adapter",
                output="screen",
                parameters=[
                    LaunchConfiguration("rtk_adapter_parameters"),
                    common_parameters,
                    {
                        "auto_reference_from_first_fix": LaunchConfiguration(
                            "auto_reference_from_first_fix"
                        ),
                        "reference_latitude_deg": LaunchConfiguration(
                            "reference_latitude_deg"
                        ),
                        "reference_longitude_deg": LaunchConfiguration(
                            "reference_longitude_deg"
                        ),
                        "reference_altitude_m": LaunchConfiguration(
                            "reference_altitude_m"
                        ),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_rtk_components")),
            ),
            OpaqueFunction(function=_sensor_transform_nodes),
            Node(
                package="lio_sam",
                executable="lio_sam_imuPreintegration",
                parameters=[lio_sam_parameters, common_parameters],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_lio_sam")),
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_imageProjection",
                parameters=[lio_sam_parameters, common_parameters],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_lio_sam")),
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_featureExtraction",
                parameters=[lio_sam_parameters, common_parameters],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_lio_sam")),
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_mapOptimization",
                parameters=[lio_sam_parameters, common_parameters],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_lio_sam")),
            ),
        ]
    )
