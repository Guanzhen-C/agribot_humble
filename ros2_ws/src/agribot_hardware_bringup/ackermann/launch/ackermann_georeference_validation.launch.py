import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_map_files(context):
    map_base = Path(LaunchConfiguration("map_base").perform(context))
    required = (
        map_base.with_suffix(".yaml"),
        map_base.with_suffix(".pcd"),
        Path(f"{map_base}_georeference.yaml"),
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(
            "地理配准测试缺少同名地图文件：" + ", ".join(missing)
        )
    return []


def _launch_georeference_bridge(context):
    map_base = Path(LaunchConfiguration("map_base").perform(context))
    mount_config = Path(LaunchConfiguration("mount_config").perform(context))
    with mount_config.open(encoding="utf-8") as stream:
        mounts = yaml.safe_load(stream)
    rtk_xyz = mounts["rtk"]["xyz"]
    if len(rtk_xyz) != 3:
        raise RuntimeError("sensor_mounts.yaml中的RTK坐标必须包含三个数值")

    return [
        Node(
            package="agribot_hardware_bringup",
            executable="georeference_test_bridge",
            name="georeference_test_bridge",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "georeference_file": f"{map_base}_georeference.yaml",
                    "map_file": f"{map_base}.pcd",
                    "initial_pose_topic": "/georeference_test/initialpose",
                    "input_horizontal_std_m": LaunchConfiguration(
                        "input_horizontal_std_m"
                    ),
                    "input_heading_std_deg": LaunchConfiguration(
                        "input_heading_std_deg"
                    ),
                    "base_to_master_antenna_m": [float(value) for value in rtk_xyz],
                }
            ],
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("agribot_hardware_bringup")
    map_base = LaunchConfiguration("map_base")
    test_initial_pose_topic = "/georeference_test/initialpose"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_base",
                description="不带扩展名的地图绝对路径；需要同名PCD、YAML和地理配准文件",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument("map_start_delay", default_value="5.0"),
            DeclareLaunchArgument(
                "mount_config",
                default_value=os.path.join(share, "config", "sensor_mounts.yaml"),
            ),
            DeclareLaunchArgument(
                "input_horizontal_std_m", default_value="0.05"
            ),
            DeclareLaunchArgument("input_heading_std_deg", default_value="2.0"),
            OpaqueFunction(function=_validate_map_files),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "vehicle_autonomy.launch.py")
                ),
                launch_arguments={
                    "vehicle_type": "ackermann",
                    "controller": "mppi",
                    "localization": "fastlio",
                    "navigation_mode": "localization",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "start_navigation": "false",
                    "start_sensors": LaunchConfiguration("start_sensors"),
                    "start_rtk": "false",
                    "mount_config": LaunchConfiguration("mount_config"),
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": os.path.join(
                        share, "rviz", "georeference_validation.rviz"
                    ),
                    "map_start_delay": LaunchConfiguration("map_start_delay"),
                    "map": PythonExpression(["'", map_base, ".yaml'"]),
                    "pcd_map_file": PythonExpression(["'", map_base, ".pcd'"]),
                    "initialization_source": "manual",
                    "mapped_initial_pose_topic": test_initial_pose_topic,
                    "enable_fpfh": LaunchConfiguration("enable_fpfh"),
                    "automatic_global_localization": "false",
                    "require_localization_ready": "true",
                    "enable_chassis_output": "false",
                    "chassis_driver": "none",
                }.items(),
            ),
            OpaqueFunction(function=_launch_georeference_bridge),
        ]
    )
