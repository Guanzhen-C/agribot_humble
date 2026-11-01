"""Run the actual small tracked car through an imported 3DGS farmland scene."""
import os
import importlib.util
import json
import time
import math
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def route_planner_bindings(bundle):
    source = bundle / "route_planner.yaml"
    if not source.is_file():
        raise ValueError("Exact original Smac planning parameters are required for the 104 preset points")
    parameters = yaml.safe_load(source.read_text())
    costmap = parameters["fuxi_route"]["global_costmap"]["global_costmap"]["ros__parameters"]
    # Nav2 creates its costmap node separately; the parent's map remap is not inherited.
    costmap["static_layer"]["map_topic"] = "/map"
    runtime = bundle / "route_planner_runtime.yaml"
    runtime.write_text(yaml.safe_dump(parameters, sort_keys=False))
    return runtime


def scene_with_sky(bundle, scene):
    manifest = json.loads(scene.read_text())
    source = Path(manifest["world"])
    if not source.is_absolute():
        source = scene.parent / source
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    visual_scene = world.find("scene")
    if visual_scene is None:
        visual_scene = ET.SubElement(world, "scene")
    if visual_scene.find("sky") is None:
        sky = ET.SubElement(visual_scene, "sky")
        ET.SubElement(sky, "time").text = "12"
        ET.SubElement(sky, "sunrise").text = "6"
        ET.SubElement(sky, "sunset").text = "20"
        clouds = ET.SubElement(sky, "clouds")
        ET.SubElement(clouds, "speed").text = "0"
    gui = world.find("gui")
    if gui is not None:
        for plugin in gui.findall("plugin[@name='agribot_camera_preview']"):
            gui.remove(plugin)
    runtime_world = bundle / "sky_world.sdf"
    tree.write(runtime_world, encoding="unicode", xml_declaration=True)
    output = bundle / "sky_scene.json"
    output.write_text(json.dumps({**manifest, "world": runtime_world.name}, ensure_ascii=False, indent=2))
    (bundle / "sky_view.json").write_text(json.dumps({
        "source_world": str(source), "runtime_world": str(runtime_world),
        "sky": "Gazebo procedural sky, not scanned geometry",
        "collision_geometry_changed": False, "gaussian_data_changed": False,
    }, indent=2))
    return output


def start(context):
    if os.environ.get("ROS_DOMAIN_ID") != "93" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1")
    bundle = Path(LaunchConfiguration("bundle").perform(context)).resolve()
    if not all((bundle/file).is_file() for file in ("scene.json", "saved_route.json", "preparation.json", "nav2_params.yaml")):
        raise ValueError("Prepare a complete imported-scene route validation bundle first")
    description = Path(get_package_share_directory("agribot_vehicle_description"))
    scene = bundle/"scene.json"
    scene = scene_with_sky(bundle, scene)
    runner_arguments = [str(bundle)]
    plan_manual_points = LaunchConfiguration("plan_manual_points").perform(context) == "true"
    planning_nodes = []
    if plan_manual_points:
        planner_parameters = route_planner_bindings(bundle)
        runner_arguments.append("--plan-manual-points")
        planning_nodes = [
            Node(package="nav2_planner", executable="planner_server", name="planner_server", namespace="fuxi_route",
                 output="screen", parameters=[str(planner_parameters)], remappings=[("map", "/map")]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager", namespace="fuxi_route",
                 output="screen", parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["planner_server"]}]),
        ]
    if LaunchConfiguration("keep_result").perform(context) == "true":
        runner_arguments.append("--keep-result")
    first = int(LaunchConfiguration("diagnostic_start_anchor").perform(context))
    last = int(LaunchConfiguration("diagnostic_end_anchor").perform(context))
    if not 1 <= first < last <= 104:
        raise ValueError("Diagnostic anchor range must be ordered within the original 104 points")
    if first != 1 or last != 104:
        helper = Path(get_package_prefix("agribot_differential_fastlivo_mppi"))/"lib/agribot_differential_fastlivo_mppi/prepare_scanned_route_validation.py"
        spec = importlib.util.spec_from_file_location("scanned_diagnostic_support", helper)
        preparation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(preparation)
        original = json.loads((bundle/"saved_route.json").read_text())
        begin, end = original["anchor_path_indices"][first-1], original["anchor_path_indices"][last-1]
        route = {**original, "path": original["path"][begin:end+1],
                 "anchor_path_indices": [i-begin for i in original["anchor_path_indices"][first-1:last]],
                 "original_anchor_numbers": list(range(first, last+1))}
        route_file = bundle/f"diagnostic_route_{first}_{last}.json"
        route_file.write_text(json.dumps(route, ensure_ascii=False))
        manifest = json.loads(scene.read_text())
        p = route["path"][0]
        heights = preparation.surface_heights(preparation.stl_triangles(bundle/"ground_support.stl"), [p["x"], p["y"]])
        metadata = json.loads((bundle/"preparation.json").read_text())
        source = Path(metadata["source_scene"])
        yaw = metadata["scene_yaw_rad"]
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        xy = (np.array([p["x"], p["y"]])-metadata["scene_translation_xy"]) @ rotation
        raw = np.concatenate([preparation.stl_triangles(part) for part in (source/"meshes").glob("collision_*.stl")])
        original_heights = preparation.surface_heights(raw, xy)
        z = float(ET.parse(source/"world.sdf").findtext("world/model[@name='k2_automation_sensor_scene']/pose").split()[2])
        ground = max(float(heights[-1]), float(original_heights[-1])+z if len(original_heights) else float(heights[-1]))
        manifest.update(spawn=[p["x"], p["y"], p["yaw"]], spawn_ground_z=ground)
        scene = bundle/f"diagnostic_scene_{first}_{last}.json"
        scene.write_text(json.dumps(manifest, indent=2))
        output = bundle/"diagnostics"/f"anchors_{first}_{last}_{time.time_ns()}"
        runner_arguments += ["--route-file", str(route_file), "--results-dir", str(output)]
    return planning_nodes + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description/"launch/configured_ackermann_sim.launch.py")),
                                 launch_arguments={"vehicle_id": "unity_car3", "scene_manifest": str(scene),
                                                   "nav2_params_override": str(bundle/"nav2_params.yaml"),
                                                   "localization_mode": "navsat", "run_waypoints": "false", "manipulator": "false",
                                                   "physical_truth_topic": "/validation/physics_truth", "sim_navsat_odom_topic": "/validation/physics_truth",
                                                   "track_drive_plugin": str(Path(get_package_prefix("agribot_differential_fastlivo_mppi"))/"lib/libagribot_scanned_track_drive.so"),
                                                   "arm_demo": "false", "controller_mode": "mppi", "gui": LaunchConfiguration("gui"),
                                                   "rviz": LaunchConfiguration("rviz"), "rviz_config": LaunchConfiguration("rviz_config")}.items()),
        Node(package="agribot_differential_fastlivo_mppi", executable="run_scanned_saved_route.py", output="screen",
             arguments=runner_arguments, parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("run_route"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bundle"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("run_route", default_value="true"),
        DeclareLaunchArgument("keep_result", default_value="false"),
        DeclareLaunchArgument("plan_manual_points", default_value="false"),
        DeclareLaunchArgument("diagnostic_start_anchor", default_value="1"),
        DeclareLaunchArgument("diagnostic_end_anchor", default_value="104"),
        DeclareLaunchArgument("rviz_config", default_value=str(Path(get_package_share_directory("agribot_differential_fastlivo_mppi"))/"config/scanned_saved_route.rviz")),
        OpaqueFunction(function=start),
    ])
