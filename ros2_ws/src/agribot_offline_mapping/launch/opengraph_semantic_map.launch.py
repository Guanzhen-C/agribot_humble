#!/usr/bin/env python3

"""Display a portable OpenGraph semantic map in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("agribot_offline_mapping"))
    default_rviz_config = package_share / "rviz" / "opengraph_semantic_map.rviz"
    map_yaml = LaunchConfiguration("map_yaml")
    map_enabled = IfCondition(PythonExpression(["'", map_yaml, "' != ''"]))
    navigation_graph = LaunchConfiguration("navigation_graph")
    graph_enabled = IfCondition(
        PythonExpression(["'", navigation_graph, "' != ''"])
    )
    route_plan = LaunchConfiguration("route_plan")
    route_enabled = IfCondition(PythonExpression(["'", route_plan, "' != ''"]))
    return LaunchDescription([
        DeclareLaunchArgument("pcd", description="Portable colored OpenGraph PCD file"),
        DeclareLaunchArgument(
            "metadata", description="Portable OpenGraph semantic_instances.json file"
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="",
            description="Optional Nav2 occupancy map YAML file",
        ),
        DeclareLaunchArgument(
            "navigation_graph",
            default_value="",
            description="Optional semantic navigation graph JSON file",
        ),
        DeclareLaunchArgument(
            "route_plan",
            default_value="",
            description="Optional preview-only semantic route JSON file",
        ),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("semantic_frame_id", default_value="opengraph_map"),
        DeclareLaunchArgument("minimum_detections", default_value="2"),
        DeclareLaunchArgument("maximum_labels", default_value="20"),
        DeclareLaunchArgument("label_scale", default_value="0.35"),
        DeclareLaunchArgument("show_bounding_boxes", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(default_rviz_config),
            description="RViz configuration file",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="opengraph_map_static_transform",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", LaunchConfiguration("frame_id"),
                "--child-frame-id", LaunchConfiguration("semantic_frame_id"),
            ],
        ),
        Node(
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="opengraph_semantic_cloud_publisher",
            output="screen",
            parameters=[{
                "file_name": LaunchConfiguration("pcd"),
                "tf_frame": LaunchConfiguration("semantic_frame_id"),
                "publishing_period_ms": 5000,
            }],
            remappings=[("cloud_pcd", "/opengraph/semantic_cloud")],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="opengraph_occupancy_map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "frame_id": LaunchConfiguration("frame_id"),
            }],
            condition=map_enabled,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="opengraph_map_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["opengraph_occupancy_map_server"],
            }],
            condition=map_enabled,
        ),
        Node(
            package="agribot_offline_mapping",
            executable="publish_opengraph_semantic_map.py",
            name="opengraph_semantic_map_publisher",
            output="screen",
            parameters=[{
                "metadata_file": LaunchConfiguration("metadata"),
                "frame_id": LaunchConfiguration("semantic_frame_id"),
                "minimum_detections": LaunchConfiguration("minimum_detections"),
                "maximum_labels": LaunchConfiguration("maximum_labels"),
                "label_scale": LaunchConfiguration("label_scale"),
                "show_bounding_boxes": LaunchConfiguration("show_bounding_boxes"),
            }],
        ),
        Node(
            package="agribot_offline_mapping",
            executable="publish_semantic_navigation_graph.py",
            name="semantic_navigation_graph_publisher",
            output="screen",
            parameters=[{
                "graph_file": navigation_graph,
                "frame_id": LaunchConfiguration("frame_id"),
                "show_place_labels": True,
                "label_scale": 0.42,
            }],
            condition=graph_enabled,
        ),
        Node(
            package="agribot_offline_mapping",
            executable="publish_semantic_route.py",
            name="semantic_route_publisher",
            output="screen",
            parameters=[{
                "route_file": route_plan,
                "frame_id": LaunchConfiguration("frame_id"),
            }],
            condition=route_enabled,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="opengraph_semantic_map_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
