#!/usr/bin/env python3

"""Open a saved map and collect manual landmark drafts from RViz clicks."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_inputs(context):
    map_yaml = Path(
        LaunchConfiguration("map_yaml").perform(context)
    ).expanduser().resolve()
    output = Path(
        LaunchConfiguration("output_file").perform(context)
    ).expanduser().resolve()
    map_id = LaunchConfiguration("map_id").perform(context).strip()

    if map_yaml.suffix.lower() not in (".yaml", ".yml") or not map_yaml.is_file():
        raise RuntimeError(f"map_yaml不是有效地图YAML: {map_yaml}")
    if output.suffix.lower() not in (".yaml", ".yml"):
        raise RuntimeError("output_file必须使用.yaml或.yml后缀")
    if not map_id:
        raise RuntimeError("map_id不能为空")
    return []


def generate_launch_description():
    package_share = Path(get_package_share_directory("agribot_offline_mapping"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml", description="需要编辑的Nav2二维地图YAML"
            ),
            DeclareLaunchArgument("map_id", description="当前地图的稳定ID"),
            DeclareLaunchArgument(
                "output_file", description="只保存手工地标草稿的YAML文件"
            ),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    package_share / "rviz" / "manual_landmark_editor.rviz"
                ),
            ),
            OpaqueFunction(function=_validate_inputs),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="manual_landmark_map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": LaunchConfiguration("map_yaml"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="manual_landmark_map_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["manual_landmark_map_server"],
                    }
                ],
            ),
            Node(
                package="agribot_offline_mapping",
                executable="manual_landmark_editor.py",
                name="manual_landmark_editor",
                output="screen",
                parameters=[
                    {
                        "map_id": LaunchConfiguration("map_id"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "output_file": LaunchConfiguration("output_file"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="manual_landmark_editor_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
