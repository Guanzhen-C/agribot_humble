import copy
import os
import platform
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _write_gazebo_vehicle_model(source_file):
    """Prepare the generated vehicle for real-time Gazebo Classic sensing.

    The canonical Unity export keeps the physical C16 and camera resolution.
    Gazebo Classic's CPU ray sensor cannot sustain that load together with
    FAST-LIVO2 and both Nav2 costmaps.  The validated long-route simulation
    uses fewer azimuth samples and renders the same half-resolution image that
    FAST-LIVO2 would otherwise create internally.  Sensor poses, rates and all
    physical configuration files remain unchanged.
    """
    target_file = os.path.join(
        tempfile.gettempdir(), "configured_ackermann_vehicle.generated.sdf"
    )
    model_tree = ET.parse(source_file)
    for uri in model_tree.getroot().iter("uri"):
        value = (uri.text or "").strip()
        for prefix in ("package://", "model://"):
            if not value.startswith(prefix):
                continue
            package_path = value[len(prefix) :]
            package_name, separator, relative_path = package_path.partition("/")
            if not separator:
                break
            try:
                share = get_package_share_directory(package_name)
            except LookupError:
                break
            candidate = os.path.join(share, relative_path)
            if os.path.isfile(candidate):
                uri.text = Path(candidate).resolve().as_uri()
            break

    lidar = model_tree.getroot().find(".//sensor[@name='lslidar_c16_points']")
    if lidar is None:
        raise RuntimeError(f"Generated vehicle has no C16 sensor: {source_file}")
    lidar_values = {
        "./ray/scan/horizontal/samples": "720",
        "./ray/range/max": "25.0",
        "./plugin/max_range": "25.0",
    }
    for query, value in lidar_values.items():
        element = lidar.find(query)
        if element is None:
            raise RuntimeError(f"Generated C16 is missing {query}: {source_file}")
        element.text = value

    camera = model_tree.getroot().find(
        ".//sensor[@name='hikrobot_right_camera']"
    )
    if camera is None:
        raise RuntimeError(f"Generated vehicle has no Hikrobot camera: {source_file}")
    camera_values = {
        "./camera/image/width": "640",
        "./camera/image/height": "512",
        "./plugin/camera_info_url": (
            "package://agribot_ackermann_mppi/config/"
            "hikrobot_camera_640_sim.yaml"
        ),
    }
    for query, value in camera_values.items():
        element = camera.find(query)
        if element is None:
            raise RuntimeError(f"Generated camera is missing {query}: {source_file}")
        element.text = value

    ET.indent(model_tree, space="  ")
    model_tree.write(target_file, encoding="unicode", xml_declaration=False)
    return target_file


def _write_fastlivo_sim_config(source_file):
    """Apply the orchard profile without changing calibrated extrinsics."""
    target_file = os.path.join(
        tempfile.gettempdir(), "configured_ackermann_fastlivo.generated.yaml"
    )
    with open(source_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    parameters = config["/**"]["ros__parameters"]
    parameters["lio"]["voxel_size"] = 1.0
    parameters["vio"]["img_point_cov"] = 800
    parameters["time_offset"]["img_time_offset"] = 0.0
    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _write_rpp_nav2_config(source_file, controller_overlay_file, profile_name):
    """Replace only the controller plugin while retaining generated geometry."""
    target_file = os.path.join(
        tempfile.gettempdir(), f"configured_ackermann_{profile_name}_rpp.generated.yaml"
    )
    with open(source_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    with open(controller_overlay_file, encoding="utf-8") as stream:
        overlay = yaml.safe_load(stream)

    controller = config["controller_server"]["ros__parameters"]
    mppi = controller["FollowPath"]
    rpp = copy.deepcopy(overlay["FollowPath"])
    rpp["desired_linear_vel"] = float(mppi["vx_max"])
    rpp["rotate_to_heading_angular_vel"] = float(mppi["wz_max"])
    rpp["max_angular_accel"] = float(mppi["az_max"])
    rpp["regulated_linear_scaling_min_radius"] = float(
        mppi["AckermannConstraints"]["min_turning_r"]
    )
    inflation = (
        config.get("global_costmap", {})
        .get("global_costmap", {})
        .get("ros__parameters", {})
        .get("inflation_layer", {})
    )
    if "cost_scaling_factor" in inflation:
        rpp["inflation_cost_scaling_factor"] = float(
            inflation["cost_scaling_factor"]
        )

    controller["goal_checker"] = copy.deepcopy(overlay["goal_checker"])
    controller["FollowPath"] = rpp
    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _controller_config(mppi_file, rpp_file):
    return PythonExpression(
        [
            "'",
            rpp_file,
            "' if '",
            LaunchConfiguration("controller_mode"),
            "' == 'rpp' else '",
            mppi_file,
            "'",
        ]
    )


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
    vehicle_model = _write_gazebo_vehicle_model(
        os.path.join(generated, "models", "ackermann_current.sdf")
    )
    base_to_body_xyz, base_to_body_rpy, base_to_antenna_xyz = _generated_mounts(
        generated
    )
    fastlivo_sim_config = _write_fastlivo_sim_config(
        os.path.join(
            generated,
            "config",
            "physical",
            "agribot_c16_astra.yaml",
        )
    )
    navsat_mppi_config = os.path.join(
        generated,
        "config",
        "simulation",
        "nav2_params_ackermann_navsat_static.yaml",
    )
    fastlio_mppi_config = os.path.join(
        generated,
        "config",
        "simulation",
        "nav2_params_ackermann_fastlio_static.yaml",
    )
    controller_overlay = os.path.join(description_share, "config", "rpp_controller.yaml")
    navsat_rpp_config = _write_rpp_nav2_config(
        navsat_mppi_config, controller_overlay, "navsat"
    )
    fastlio_rpp_config = _write_rpp_nav2_config(
        fastlio_mppi_config, controller_overlay, "fastlio"
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
            DeclareLaunchArgument("run_waypoints", default_value="true"),
            DeclareLaunchArgument("waypoint_startup_delay", default_value="1.0"),
            # Start the readiness gate early. The gate still blocks Nav2 until
            # localization has produced a real map-frame pose.
            DeclareLaunchArgument("navigation_delay", default_value="8.0"),
            # Let localization and Nav2 complete their CPU-heavy startup before
            # RViz begins subscribing to point clouds, images and costmaps.
            DeclareLaunchArgument("rviz_start_delay", default_value="16.0"),
            DeclareLaunchArgument("localization_mode", default_value="fastlivo_rtk"),
            DeclareLaunchArgument(
                "controller_mode", default_value="mppi", choices=["mppi", "rpp"]
            ),
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
                    "waypoint_navigation_mode": "plan_then_follow_path",
                    "waypoint_startup_delay": LaunchConfiguration(
                        "waypoint_startup_delay"
                    ),
                    "waypoint_readiness_gate_enabled": "true",
                    "rviz_start_delay": LaunchConfiguration("rviz_start_delay"),
                    "waypoint_file": os.path.join(
                        simulation_share,
                        "config",
                        "orchard_waypoints_ackermann_smac.yaml",
                    ),
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "localization_mode": LaunchConfiguration("localization_mode"),
                    "use_static_map": LaunchConfiguration("use_static_map"),
                    "enable_slam_map": LaunchConfiguration("enable_slam_map"),
                    "map_file_location": LaunchConfiguration("map_file_location"),
                    "map_file": LaunchConfiguration("map_file"),
                    "world": world_file,
                    "gazebo_spawn_file": vehicle_model,
                    "robot_description_file": os.path.join(
                        generated,
                        "urdf",
                        "ackermann_current.urdf.xacro",
                    ),
                    "navsat_static_nav2_params_file": _controller_config(
                        navsat_mppi_config, navsat_rpp_config
                    ),
                    "fastlio_static_nav2_params_file": _controller_config(
                        fastlio_mppi_config, fastlio_rpp_config
                    ),
                    "fastlivo_lidar_config_file": fastlivo_sim_config,
                    "fastlivo_camera_config_file": os.path.join(
                        simulation_share,
                        "config",
                        "fastlivo_hikrobot_640_sim.yaml",
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
