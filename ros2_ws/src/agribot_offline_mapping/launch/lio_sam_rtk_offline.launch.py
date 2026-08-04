import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("agribot_offline_mapping")
    map_base = LaunchConfiguration("map_base")
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
            DeclareLaunchArgument(
                "auto_reference_from_first_fix", default_value="true"
            ),
            DeclareLaunchArgument("reference_latitude_deg", default_value="0.0"),
            DeclareLaunchArgument("reference_longitude_deg", default_value="0.0"),
            DeclareLaunchArgument("reference_altitude_m", default_value="0.0"),
            DeclareLaunchArgument(
                "lio_sam_parameters",
                default_value=os.path.join(package_share, "config", "lio_sam_c16.yaml"),
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
            ),
            Node(
                package="agribot_offline_mapping",
                executable="map_georeference_exporter",
                output="screen",
                parameters=[
                    LaunchConfiguration("georeference_parameters"),
                    common_parameters,
                    {
                        "map_pcd_file": PythonExpression(["'", map_base, ".pcd'"]),
                        "output_file": PythonExpression(
                            ["'", map_base, "_georeference.yaml'"]
                        ),
                        "source_bag": LaunchConfiguration("source_bag"),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="offline_base_to_imu",
                arguments=[
                    "--x", "0.1425", "--y", "0.0", "--z", "0.143",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "imu_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="offline_base_to_lidar",
                arguments=[
                    "--x", "0.48", "--y", "0.0", "--z", "0.233",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "lidar_link",
                ],
            ),
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
