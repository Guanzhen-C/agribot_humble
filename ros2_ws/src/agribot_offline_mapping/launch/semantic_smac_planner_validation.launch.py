#!/usr/bin/env python3

"""Show semantic routing and Smac Ackermann planning in one dry-run session."""

import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _strict_json(path, description):
    def reject_constant(value):
        raise ValueError(f"non-finite JSON number: {value}")

    def reject_duplicate_keys(pairs):
        document = {}
        for key, value in pairs:
            if key in document:
                raise ValueError(f"duplicate JSON key: {key}")
            document[key] = value
        return document

    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeDecodeError, ValueError) as error:
        raise RuntimeError(f"{description} is not strict JSON: {error}") from error


def _required_file(context, argument, suffixes):
    path = Path(LaunchConfiguration(argument).perform(context)).expanduser().resolve()
    if path.suffix.lower() not in suffixes:
        expected = " or ".join(sorted(suffixes))
        raise RuntimeError(f"{argument} must use {expected}")
    if not path.is_file():
        raise RuntimeError(f"{argument} file does not exist: {path}")
    return path


def _validate_inputs(context):
    _required_file(context, "map", {".yaml", ".yml"})
    graph_path = _required_file(context, "navigation_graph", {".json"})
    route_path = _required_file(context, "route_plan", {".json"})

    show_3d = LaunchConfiguration("show_3d_map").perform(context).lower()
    if show_3d in ("1", "true", "yes", "on"):
        _required_file(context, "pcd_map", {".pcd"})

    graph = _strict_json(graph_path, "semantic navigation graph")
    route = _strict_json(route_path, "semantic route")
    if not isinstance(graph, dict) or graph.get("schema_version") != 3:
        raise RuntimeError("semantic navigation graph must use schema version 3")
    if not isinstance(route, dict) or route.get("schema_version") != 3:
        raise RuntimeError("semantic route must use schema version 3")

    graph_frame = str(graph.get("frame_id", "")).lstrip("/")
    route_frame = str(route.get("frame_id", "")).lstrip("/")
    if not graph_frame or graph_frame != route_frame:
        raise RuntimeError("semantic graph and route use different coordinate frames")

    graph_digest = hashlib.sha256(graph_path.read_bytes()).hexdigest()
    if route.get("graph_sha256") != graph_digest:
        raise RuntimeError(
            "semantic route was generated from a different navigation graph"
        )
    policy = route.get("execution_policy")
    if (
        not isinstance(policy, dict)
        or policy.get("preview_only") is not True
        or policy.get("execution_authorized") is not False
        or policy.get("requires_nav2_path_planning") is not True
    ):
        raise RuntimeError("semantic route is not a validated preview-only plan")
    return []


def generate_launch_description():
    offline_share = Path(get_package_share_directory("agribot_offline_mapping"))
    hardware_share = Path(get_package_share_directory("agribot_hardware_bringup"))
    planner_launch = (
        hardware_share
        / "launch"
        / "ackermann_smac_planner_validation.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", description="Absolute path to the Nav2 map YAML"
            ),
            DeclareLaunchArgument(
                "navigation_graph",
                description="Absolute path to the semantic navigation graph JSON",
            ),
            DeclareLaunchArgument(
                "route_plan",
                description="Absolute path to a preview-only semantic route JSON",
            ),
            DeclareLaunchArgument("pcd_map", default_value=""),
            DeclareLaunchArgument("show_3d_map", default_value="false"),
            DeclareLaunchArgument("show_place_labels", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "route_waypoint_mode", default_value="all_astar"
            ),
            DeclareLaunchArgument("path_output", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    hardware_share
                    / "ackermann"
                    / "config"
                    / "nav2_params_ackermann_planner_only.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    offline_share
                    / "rviz"
                    / "semantic_smac_planner_validation.rviz"
                ),
            ),
            OpaqueFunction(function=_validate_inputs),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(str(planner_launch)),
                        launch_arguments={
                            "map": LaunchConfiguration("map"),
                            "pcd_map": LaunchConfiguration("pcd_map"),
                            "show_3d_map": LaunchConfiguration("show_3d_map"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "params_file": LaunchConfiguration("params_file"),
                            "route_plan": LaunchConfiguration("route_plan"),
                            "route_waypoint_mode": LaunchConfiguration(
                                "route_waypoint_mode"
                            ),
                            "path_output": LaunchConfiguration("path_output"),
                            "rviz": "false",
                        }.items(),
                    )
                ],
            ),
            Node(
                package="agribot_offline_mapping",
                executable="publish_semantic_navigation_graph.py",
                name="semantic_navigation_graph_publisher",
                output="screen",
                parameters=[
                    {
                        "graph_file": LaunchConfiguration("navigation_graph"),
                        "show_place_labels": LaunchConfiguration(
                            "show_place_labels"
                        ),
                    }
                ],
            ),
            Node(
                package="agribot_offline_mapping",
                executable="publish_semantic_route.py",
                name="semantic_route_publisher",
                output="screen",
                parameters=[{"route_file": LaunchConfiguration("route_plan")}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="semantic_smac_planner_validation_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
