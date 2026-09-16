import os
import platform
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _write_jetson_world(source_file):
    target_file = os.path.join(
        tempfile.gettempdir(), "configured_ackermann_jetson.generated.world"
    )
    world_tree = ET.parse(source_file)
    world = world_tree.getroot().find("world")
    if world is None:
        raise RuntimeError(f"Gazebo world file has no <world>: {source_file}")

    scene = world.find("scene")
    if scene is not None:
        shadows = scene.find("shadows")
        if shadows is not None:
            shadows.text = "0"

    plugin_name = "agribot_ackermann_gazebo_scene_sync"
    existing = world.find(f"./plugin[@name='{plugin_name}']")
    if existing is not None:
        world.remove(existing)
    ET.SubElement(
        world,
        "plugin",
        {
            "name": plugin_name,
            "filename": "libagribot_ackermann_gazebo_scene_sync.so",
        },
    )
    ET.indent(world_tree, space="  ")
    world_tree.write(target_file, encoding="unicode", xml_declaration=False)
    return target_file


def _generated_mounts(generated):
    config_dir = os.path.join(generated, "config", "physical")
    with open(os.path.join(config_dir, "fastlivo_bridge.yaml"), encoding="utf-8") as stream:
        bridge = yaml.safe_load(stream)["fastlio_odom_bridge"]["ros__parameters"]
    with open(
        os.path.join(config_dir, "fastlivo_rtk_fusion.yaml"), encoding="utf-8"
    ) as stream:
        fusion = yaml.safe_load(stream)["fastlivo_rtk_fusion"]["ros__parameters"]
    return (
        bridge["base_to_body_xyz"],
        bridge["base_to_body_rpy"],
        fusion["base_to_antenna_xyz"],
    )


def generate_launch_description():
    description_share = get_package_share_directory("agribot_vehicle_description")
    simulation_share = get_package_share_directory("agribot_ackermann_mppi")
    navigation_share = get_package_share_directory("scout_navigation")
    gazebo_share = get_package_share_directory("scout_gazebo")
    generated = os.path.join(description_share, "generated", "ackermann_current")
    base_to_body_xyz, base_to_body_rpy, base_to_antenna_xyz = _generated_mounts(
        generated
    )
    is_jetson = platform.machine() in ("aarch64", "arm64")
    world_file = os.path.join(gazebo_share, "worlds", "orchard_barriers.world")
    if is_jetson:
        world_file = _write_jetson_world(world_file)

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("run_waypoints", default_value="false"),
            DeclareLaunchArgument("navigation_delay", default_value="22.0"),
            DeclareLaunchArgument("localization_mode", default_value="fastlivo_rtk"),
            DeclareLaunchArgument("use_static_map", default_value="true"),
            DeclareLaunchArgument("enable_slam_map", default_value="false"),
            DeclareLaunchArgument(
                "map_file_location",
                default_value=os.path.join(navigation_share, "maps"),
            ),
            DeclareLaunchArgument("map_file", default_value="orchard_v2_map6.yaml"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        simulation_share,
                        "launch",
                        "ackermann_waypoint_depth_collect_sim.launch.py",
                    )
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless": LaunchConfiguration("headless"),
                    "gazebo_render_workaround": "true" if is_jetson else "false",
                    "run_waypoints": LaunchConfiguration("run_waypoints"),
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "localization_mode": LaunchConfiguration("localization_mode"),
                    "use_static_map": LaunchConfiguration("use_static_map"),
                    "enable_slam_map": LaunchConfiguration("enable_slam_map"),
                    "map_file_location": LaunchConfiguration("map_file_location"),
                    "map_file": LaunchConfiguration("map_file"),
                    "world": world_file,
                    "gazebo_spawn_file": os.path.join(
                        generated, "models", "ackermann_current.sdf"
                    ),
                    "robot_description_file": os.path.join(
                        generated,
                        "urdf",
                        "ackermann_current.urdf.xacro",
                    ),
                    "navsat_static_nav2_params_file": os.path.join(
                        generated,
                        "config",
                        "simulation",
                        "nav2_params_ackermann_navsat_static.yaml",
                    ),
                    "fastlio_static_nav2_params_file": os.path.join(
                        generated,
                        "config",
                        "simulation",
                        "nav2_params_ackermann_fastlio_static.yaml",
                    ),
                    "fastlivo_lidar_config_file": os.path.join(
                        generated,
                        "config",
                        "physical",
                        "agribot_c16_astra.yaml",
                    ),
                    "fastlivo_bridge_config_file": os.path.join(
                        generated,
                        "config",
                        "physical",
                        "fastlivo_bridge.yaml",
                    ),
                    "fastlivo_rtk_fusion_config_file": os.path.join(
                        generated,
                        "config",
                        "physical",
                        "fastlivo_rtk_fusion.yaml",
                    ),
                    "navsat_ekf_params_file": os.path.join(
                        generated,
                        "config",
                        "physical",
                        "kf_gins_n300pro.yaml",
                    ),
                    "sim_base_to_antenna_x": str(base_to_antenna_xyz[0]),
                    "sim_base_to_antenna_y": str(base_to_antenna_xyz[1]),
                    "sim_base_to_antenna_z": str(base_to_antenna_xyz[2]),
                    "fastlio_base_to_body_x": str(base_to_body_xyz[0]),
                    "fastlio_base_to_body_y": str(base_to_body_xyz[1]),
                    "fastlio_base_to_body_z": str(base_to_body_xyz[2]),
                    "fastlio_base_to_body_roll": str(base_to_body_rpy[0]),
                    "fastlio_base_to_body_pitch": str(base_to_body_rpy[1]),
                    "fastlio_base_to_body_yaw": str(base_to_body_rpy[2]),
                }.items(),
            ),
        ]
    )
