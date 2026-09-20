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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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


def _write_nav2_profile_config(
    source_file,
    rpp_overlay_file,
    dwb_overlay_file,
    localization_name,
    perception_mode,
    controller_mode,
    planner_mode,
):
    """Build a Nav2 profile while retaining Unity-generated vehicle geometry."""
    target_file = os.path.join(
        tempfile.gettempdir(),
        (
            f"configured_ackermann_{localization_name}_{perception_mode}_{planner_mode}_"
            f"{controller_mode}.generated.yaml"
        ),
    )
    with open(source_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    _configure_perception(config, perception_mode)

    controller = config["controller_server"]["ros__parameters"]
    mppi = controller["FollowPath"]
    if controller_mode == "rpp":
        with open(rpp_overlay_file, encoding="utf-8") as stream:
            overlay = yaml.safe_load(stream)
        selected_controller = copy.deepcopy(overlay["FollowPath"])
        selected_controller["desired_linear_vel"] = float(mppi["vx_max"])
        selected_controller["rotate_to_heading_angular_vel"] = float(mppi["wz_max"])
        selected_controller["max_angular_accel"] = float(mppi["az_max"])
        selected_controller["regulated_linear_scaling_min_radius"] = float(
            mppi["AckermannConstraints"]["min_turning_r"]
        )
        inflation = (
            config.get("global_costmap", {})
            .get("global_costmap", {})
            .get("ros__parameters", {})
            .get("inflation_layer", {})
        )
        if "cost_scaling_factor" in inflation:
            selected_controller["inflation_cost_scaling_factor"] = float(
                inflation["cost_scaling_factor"]
            )
        controller["goal_checker"] = copy.deepcopy(overlay["goal_checker"])
        controller["FollowPath"] = selected_controller
    elif controller_mode == "dwb":
        with open(dwb_overlay_file, encoding="utf-8") as stream:
            overlay = yaml.safe_load(stream)
        selected_controller = copy.deepcopy(overlay["FollowPath"])
        selected_controller["max_vel_x"] = float(mppi["vx_max"])
        selected_controller["max_speed_xy"] = float(mppi["vx_max"])
        selected_controller["max_vel_theta"] = float(mppi["wz_max"])
        selected_controller["acc_lim_x"] = float(mppi["ax_max"])
        selected_controller["decel_lim_x"] = float(mppi["ax_min"])
        selected_controller["acc_lim_theta"] = float(mppi["az_max"])
        selected_controller["decel_lim_theta"] = -float(mppi["az_max"])
        controller["goal_checker"] = copy.deepcopy(overlay["goal_checker"])
        controller["FollowPath"] = selected_controller
    elif controller_mode != "mppi":
        raise ValueError(f"Unsupported controller mode: {controller_mode}")

    planner = config["planner_server"]["ros__parameters"]
    if planner_mode == "navfn":
        planner["GridBased"] = {
            "plugin": "nav2_navfn_planner/NavfnPlanner",
            "tolerance": 0.25,
            "use_astar": False,
            "allow_unknown": True,
            "use_final_approach_orientation": True,
        }
    elif planner_mode == "theta_star":
        planner["GridBased"] = {
            "plugin": "nav2_theta_star_planner/ThetaStarPlanner",
            "how_many_corners": 8,
            "w_euc_cost": 1.0,
            "w_traversal_cost": 2.0,
            "allow_unknown": True,
            "use_final_approach_orientation": True,
        }
    elif planner_mode == "smac_2d":
        planner["GridBased"] = {
            "plugin": "nav2_smac_planner/SmacPlanner2D",
            "tolerance": 0.25,
            "downsample_costmap": True,
            "downsampling_factor": 2,
            "allow_unknown": True,
            "max_iterations": 1000000,
            "max_on_approach_iterations": 1000,
            "max_planning_time": 5.0,
            "cost_travel_multiplier": 2.0,
            "use_final_approach_orientation": True,
            "smoother": {
                "max_iterations": 1000,
                "w_smooth": 0.3,
                "w_data": 0.2,
                "tolerance": 1.0e-10,
            },
        }
    elif planner_mode != "smac_hybrid":
        raise ValueError(f"Unsupported planner mode: {planner_mode}")

    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _configure_perception(config, perception_mode):
    """Select a costmap perception plugin behind one PointCloud2 contract."""
    if perception_mode == "stvl":
        return
    if perception_mode != "voxel":
        raise ValueError(f"Unsupported perception mode: {perception_mode}")

    for costmap_name in ("local_costmap", "global_costmap"):
        parameters = config[costmap_name][costmap_name]["ros__parameters"]
        plugins = list(parameters["plugins"])
        if "stvl_layer" not in plugins:
            raise ValueError(f"{costmap_name} has no stvl_layer to replace")
        plugin_index = plugins.index("stvl_layer")
        plugins[plugin_index] = "voxel_layer"
        parameters["plugins"] = plugins

        stvl = parameters.pop("stvl_layer")
        marking = stvl["lidar_mark"]
        obstacle_range = float(marking.get("obstacle_range", 8.0))
        min_height = float(marking.get("min_obstacle_height", 0.0))
        max_height = float(marking.get("max_obstacle_height", 2.0))
        parameters["voxel_layer"] = {
            "plugin": "nav2_costmap_2d::VoxelLayer",
            "enabled": True,
            "footprint_clearing_enabled": True,
            "publish_voxel_map": False,
            "origin_z": 0.0,
            "z_resolution": 0.1,
            "z_voxels": 16,
            "unknown_threshold": 15,
            "mark_threshold": 0,
            "combination_method": int(stvl.get("combination_method", 1)),
            "observation_sources": "lidar",
            "lidar": {
                "topic": marking["topic"],
                "data_type": "PointCloud2",
                "clearing": True,
                "marking": True,
                "obstacle_min_range": 0.0,
                "obstacle_max_range": obstacle_range,
                "raytrace_min_range": 0.0,
                "raytrace_max_range": obstacle_range + 1.0,
                "min_obstacle_height": min_height,
                "max_obstacle_height": max_height,
                "expected_update_rate": float(
                    marking.get("expected_update_rate", 0.0)
                ),
                "observation_persistence": float(
                    marking.get("observation_persistence", 0.0)
                ),
            },
        }


def _write_nav2_profiles(source_file, config_dir, localization_name):
    rpp_overlay = os.path.join(config_dir, "rpp_controller.yaml")
    dwb_overlay = os.path.join(config_dir, "dwb_controller.yaml")
    profiles = {}
    for perception_mode in ("stvl", "voxel"):
        for controller_mode in ("mppi", "rpp", "dwb"):
            for planner_mode in ("smac_hybrid", "navfn", "theta_star", "smac_2d"):
                profiles[(perception_mode, controller_mode, planner_mode)] = (
                    _write_nav2_profile_config(
                        source_file,
                        rpp_overlay,
                        dwb_overlay,
                        localization_name,
                        perception_mode,
                        controller_mode,
                        planner_mode,
                    )
                )
    return profiles


def _nav2_profile_config(profiles):
    mapping_items = ", ".join(
        f"{perception + ':' + controller + ':' + planner!r}: {path!r}"
        for (perception, controller, planner), path in profiles.items()
    )
    return PythonExpression(
        [
            "{",
            mapping_items,
            "}['",
            LaunchConfiguration("perception_mode"),
            ":' + '",
            LaunchConfiguration("controller_mode"),
            ":' + '",
            LaunchConfiguration("planner_mode"),
            "']",
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
    config_dir = os.path.join(description_share, "config")
    navsat_nav2_profiles = _write_nav2_profiles(
        navsat_mppi_config, config_dir, "navsat"
    )
    fastlio_nav2_profiles = _write_nav2_profiles(
        fastlio_mppi_config, config_dir, "fastlio"
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
            DeclareLaunchArgument(
                "perception_mode", default_value="stvl", choices=["stvl", "voxel"]
            ),
            DeclareLaunchArgument(
                "vision_mode",
                default_value="off",
                choices=["off", "canny", "orb", "optical_flow"],
                description=(
                    "Independent visual perception output; it is not connected "
                    "to Nav2 costmaps or motion control"
                ),
            ),
            DeclareLaunchArgument("localization_mode", default_value="fastlivo_rtk"),
            DeclareLaunchArgument(
                "controller_mode",
                default_value="mppi",
                choices=["mppi", "rpp", "dwb"],
            ),
            DeclareLaunchArgument(
                "planner_mode",
                default_value="smac_hybrid",
                choices=["smac_hybrid", "navfn", "theta_star", "smac_2d"],
            ),
            DeclareLaunchArgument(
                "route_mode", default_value="planned", choices=["planned", "direct"]
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
                    "waypoint_navigation_mode": PythonExpression(
                        [
                            "'follow_path' if '",
                            LaunchConfiguration("route_mode"),
                            "' == 'direct' else 'plan_then_follow_path'",
                        ]
                    ),
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
                    "navsat_static_nav2_params_file": _nav2_profile_config(
                        navsat_nav2_profiles
                    ),
                    "fastlio_static_nav2_params_file": _nav2_profile_config(
                        fastlio_nav2_profiles
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
            Node(
                package="agribot_visual_perception",
                executable="visual_perception_node",
                name="visual_perception",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("vision_mode"), "' != 'off'"]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "mode": LaunchConfiguration("vision_mode"),
                        "input_topic": "/camera/rgb/image_raw",
                        "output_topic": "/vision/annotated_image",
                        "status_topic": "/vision/status",
                        "max_rate_hz": 10.0,
                    }
                ],
            ),
        ]
    )
