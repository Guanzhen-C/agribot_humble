import copy
import json
import math
import os
import platform
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _gazebo_transport_compat_path():
    directory = os.path.join(get_package_prefix("agribot_vehicle_description"),
                             "lib", "agribot_gazebo_transport_compat")
    if not os.path.isfile(os.path.join(directory, "libgazebo_transport.so.11")):
        return ""
    # Never load a stale private library after the system Gazebo is upgraded.
    version = subprocess.check_output(
        ["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    return directory if version == "11.10.2" else ""


def _write_simulation_rviz_config(source_file, vision_mode="off", visual_belts=False):
    with open(source_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    panels = config.setdefault("Panels", [])
    if not any(panel.get("Class") == "nav2_rviz_plugins/Navigation 2" for panel in panels):
        panels.append({"Class": "nav2_rviz_plugins/Navigation 2", "Name": "Navigation 2"})
    tools = config["Visualization Manager"].setdefault("Tools", [])
    for tool in tools:
        if tool.get("Class") == "rviz_default_plugins/SetGoal":
            tool.clear()
            tool["Class"] = "nav2_rviz_plugins/GoalTool"
    if not any(tool.get("Class") == "nav2_rviz_plugins/GoalTool" for tool in tools):
        tools.append({"Class": "nav2_rviz_plugins/GoalTool"})
    config.setdefault("Window Geometry", {})["Navigation 2"] = {"collapsed": False}
    for display in config["Visualization Manager"]["Displays"]:
        if display["Class"] == "rviz_default_plugins/Image":
            display["Class"] = "scout_viz/Image"
        if (display["Class"] == "rviz_default_plugins/Path" and
                display.get("Topic", {}).get("Value") in ("/plan", "/local_plan", "/waypoint_path")):
            display["Class"] = "scout_viz/NavigationPath"
    rgb = next(display for display in config["Visualization Manager"]["Displays"]
               if display["Name"] == "RGB Image")
    rgb["Enabled"] = True
    rgb["Value"] = True
    rgb["Queue Size"] = 1
    image_topic = "/camera/rgb/image_raw" if vision_mode == "off" else "/vision/annotated_image"
    rgb["Topic"].update({"Value": image_topic, "Depth": 1,
                         "Reliability Policy": "Best Effort"})
    if visual_belts:
        config["Visualization Manager"]["Displays"].append({
            "Class": "rviz_default_plugins/MarkerArray", "Name": "Track Animation",
            "Enabled": True, "Value": True,
            "Topic": {"Value": "/simulation/track_visuals", "Depth": 2,
                      "Reliability Policy": "Reliable", "Durability Policy": "Volatile"},
        })
    target = os.path.join(tempfile.gettempdir(), "configured_vehicle_camera.generated.rviz")
    with open(target, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    return target


def _replace_manipulator(root, vehicle, sdf):
    from agribot_vehicle_config_tools.manipulator import append_sdf, append_urdf
    for child in list(root):
        name = child.get("name", "")
        if name.startswith("manipulator_") or name.startswith("vehicle_manipulator_"):
            root.remove(child)
    (append_sdf if sdf else append_urdf)(root, vehicle)


def _remove_implement_collisions(root, vehicle, sdf):
    """Simulation implements retain visuals and inertia, but have no contact geometry."""
    frames = {item["frameId"] for item in vehicle.get("implements", [])}
    if vehicle.get("manipulators"):
        from agribot_vehicle_config_tools.manipulator import PREFIX
        frames.add(PREFIX + "link_base")
    joints = []
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is not None and child is not None:
            joints.append((parent.text, child.text) if sdf else
                          (parent.get("link"), child.get("link")))
    while True:
        descendants = {child for parent, child in joints if parent in frames}
        if descendants.issubset(frames):
            break
        frames.update(descendants)
    for link in root.findall("link"):
        if link.get("name") in frames:
            for collision in list(link.findall("collision")):
                link.remove(collision)


def _body_navigation_footprint(vehicle):
    from agribot_vehicle_config_tools.model import footprint
    body = copy.copy(vehicle)
    body["implements"] = []
    return footprint(body)


def _final_position_tolerance(vehicle):
    """Use half the front-to-rear wheelbase for every configured vehicle."""
    wheelbase = float(vehicle["geometry"]["wheelbaseM"])
    if not math.isfinite(wheelbase) or wheelbase <= 0.0:
        raise ValueError("Final-position tolerance requires a positive finite wheelbaseM")
    return wheelbase * 0.5


def _write_manipulator_urdf(source_file, vehicle):
    tree = ET.parse(source_file)
    _replace_manipulator(tree.getroot(), vehicle, False)
    _remove_implement_collisions(tree.getroot(), vehicle, False)
    target = os.path.join(tempfile.gettempdir(), "configured_manipulator_vehicle.urdf")
    ET.indent(tree, space="  ")
    tree.write(target, encoding="unicode")
    return target


def _configure_tracked_contact(model, vehicle, config_file):
    with open(config_file, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream).get(vehicle["vehicleId"])
    if profile is None:
        return
    if vehicle["vehicleType"] != "tracked":
        raise ValueError("Tracked contact profile requires a tracked vehicle")
    for key in ("front_lateral_friction", "rear_lateral_friction", "longitudinal_friction"):
        value = profile[key]
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError("Tracked contact friction must be finite and positive")
    aligned = vehicle.get("metadata", {}).get("simulationTrackContactGeometry", {}).get("revision") == "car3_visible_idler_centers_v1"
    adaptive = profile.get("adaptive_slip")
    if aligned:
        value = profile["idler_aligned_lateral_friction"]
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError("Tracked contact friction/slip must be finite and positive")
        adaptive = {
            "straight_compliance": profile["idler_aligned_slip_compliance"],
            "turning_slip": profile["idler_aligned_turning_slip"],
            "turn_start_speed_difference": profile["turn_start_speed_difference"],
            "turn_full_speed_difference": profile["turn_full_speed_difference"],
        }
    if adaptive is not None:
        if not isinstance(adaptive, dict):
            raise ValueError("Tracked adaptive slip profile must be a mapping")
        for key in ("straight_compliance", "turning_slip",
                    "turn_start_speed_difference", "turn_full_speed_difference"):
            value = adaptive[key]
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError("Tracked contact friction/slip must be finite and positive")
        if adaptive["turn_start_speed_difference"] >= adaptive["turn_full_speed_difference"]:
            raise ValueError("Tracked turn speed transition must be increasing")
        masses = [float(node.text) for node in model.findall("link/inertial/mass")]
        if not masses or any(not math.isfinite(mass) or mass <= 0 for mass in masses):
            raise ValueError("Tracked wheel slip requires finite positive model masses")
        normal_force = sum(masses) * 9.81 / len(vehicle["geometry"]["wheelCenters"])
        previous = model.find("plugin[@name='configured_tracked_wheel_slip']")
        if previous is not None:
            model.remove(previous)
        slip_plugin = ET.SubElement(model, "plugin", name="configured_tracked_wheel_slip",
                                    filename="libagribot_simulation_tracked_wheel_slip.so")
        for key in ("straight_compliance", "turning_slip",
                    "turn_start_speed_difference", "turn_full_speed_difference"):
            ET.SubElement(slip_plugin, key).text = str(adaptive[key])
    for wheel in vehicle["geometry"]["wheelCenters"]:
        wheel_id = wheel["id"]
        lateral_key = "front_lateral_friction" if wheel_id.startswith("front_") else "rear_lateral_friction"
        link = model.find(f"link[@name='{wheel_id}_wheel_link']")
        ode = None if link is None else link.find("collision/surface/friction/ode")
        if ode is None:
            raise ValueError(f"Tracked wheel is missing ODE friction: {wheel_id}")
        # Collision-local Z is the cylinder axle and does not rotate with spin.
        lateral = profile["idler_aligned_lateral_friction"] if aligned else profile[lateral_key]
        for key, value in (("fdir1", "0 0 1"), ("mu", lateral),
                           ("mu2", profile["longitudinal_friction"])):
            element = ode.find(key)
            if element is None:
                element = ET.SubElement(ode, key)
            element.text = str(value)
        if adaptive is not None:
            # The official plugin scales ODE slip with wheel speed; constant
            # slip allowed gravity to move stopped tracks sideways on slopes.
            for key in ("slip1", "slip2"):
                element = ode.find(key)
                if element is None:
                    element = ET.SubElement(ode, key)
                element.text = "0"
            wheel_node = ET.SubElement(slip_plugin, "wheel", link_name=wheel_id + "_wheel_link")
            for key, value in (("wheel_normal_force", normal_force),
                               ("wheel_radius", wheel["radiusM"]),
                               ("slip_compliance_lateral", adaptive["straight_compliance"]),
                               ("slip_compliance_longitudinal", 0)):
                ET.SubElement(wheel_node, key).text = str(value)


def _write_gazebo_vehicle_model(source_file, lidar_input_topic=None, vehicle=None,
                                contact_config=None):
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
    if vehicle is not None:
        _replace_manipulator(model_tree.getroot().find("model"), vehicle, True)
        _remove_implement_collisions(model_tree.getroot().find("model"), vehicle, True)
        if contact_config is not None:
            _configure_tracked_contact(model_tree.getroot().find("model"), vehicle, contact_config)
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
    if lidar_input_topic:
        lidar.find("./plugin/ros/remapping").text = "~/out:=" + lidar_input_topic

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
    parameters["common"]["lid_topic"] = "/configured/localization_points"
    parameters["lio"]["voxel_size"] = 1.0
    parameters["vio"]["img_point_cov"] = 800
    parameters["time_offset"]["img_time_offset"] = 0.0
    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _write_fastlio_sim_config(source_file, extrinsics_file):
    with open(source_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    with open(extrinsics_file, encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream)["/**"]["ros__parameters"]["extrin_calib"]
    parameters = config["/**"]["ros__parameters"]
    parameters["common"]["lid_topic"] = "/configured/localization_points"
    for key in ("extrinsic_T", "extrinsic_R"):
        parameters["mapping"][key] = copy.deepcopy(extrinsics[key])
    target_file = os.path.join(tempfile.gettempdir(), "configured_ackermann_fastlio.generated.yaml")
    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _configure_nominal_footprint_inflation(config):
    """Keep a nonzero inflation cost at the CAD vehicle's circumscribed radius."""
    for name in ("global_costmap", "local_costmap"):
        parameters = config[name][name]["ros__parameters"]
        points = yaml.safe_load(parameters["footprint"])
        resolution = float(parameters["resolution"])
        circumscribed = max(math.hypot(x, y) for x, y in points)
        edge_distances = []
        for a, b in zip(points, points[1:] + points[:1]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length_squared = dx * dx + dy * dy
            t = max(0.0, min(1.0, -(a[0] * dx + a[1] * dy) / length_squared))
            edge_distances.append(math.hypot(a[0] + t * dx, a[1] + t * dy))
        inscribed = min(edge_distances)
        radius = math.ceil(circumscribed / resolution) * resolution + resolution
        inflation = parameters["inflation_layer"]
        inflation["inflation_radius"] = max(inflation["inflation_radius"], radius)
        # Nav2 truncates exponential inflation to uint8. Retain at least cost 5
        # at the outer circle, including a cell of discretization margin.
        max_scaling = math.log(252.0 / 5.0) / (radius - inscribed)
        inflation["cost_scaling_factor"] = min(inflation["cost_scaling_factor"], max_scaling)


def _uses_local_cad_profile(vehicle):
    metadata = vehicle.get("metadata", {})
    return (metadata.get("calibrationState") == "unity_nominal_simulation_only"
            and (metadata.get("authoringTool") == "inspected_local_CAD_profile"
                 or bool(metadata.get("nominalDimensions"))))


def _write_nav2_profile_config(
    source_file,
    rpp_overlay_file,
    dwb_overlay_file,
    localization_name,
    perception_mode,
    controller_mode,
    planner_mode,
    nominal_vehicle=False,
    navigation_footprint=None,
    final_position_tolerance=0.25,
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

    if navigation_footprint is not None:
        for name in ("global_costmap", "local_costmap"):
            config[name][name]["ros__parameters"]["footprint"] = json.dumps(
                navigation_footprint, separators=(",", ":"))
    _configure_obstacle_height_filter(config)
    _configure_perception(config, perception_mode)
    if nominal_vehicle:
        _configure_nominal_footprint_inflation(config)

    controller = config["controller_server"]["ros__parameters"]
    mppi = controller["FollowPath"]
    differential = mppi.get("motion_model") == "DiffDrive"
    reverse_speed = float(mppi["vx_min"])
    if controller_mode == "rpp":
        with open(rpp_overlay_file, encoding="utf-8") as stream:
            overlay = yaml.safe_load(stream)
        selected_controller = copy.deepcopy(overlay["FollowPath"])
        selected_controller["desired_linear_vel"] = float(mppi["vx_max"])
        selected_controller["rotate_to_heading_angular_vel"] = float(mppi["wz_max"])
        selected_controller["max_angular_accel"] = float(mppi["az_max"])
        if differential:
            selected_controller["use_rotate_to_heading"] = True
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
        selected_controller["min_turning_radius"] = float(
            mppi["AckermannConstraints"]["min_turning_r"]
        )
        if differential:
            selected_controller["trajectory_generator_name"] = "dwb_plugins::StandardTrajectoryGenerator"
            selected_controller.pop("min_turning_radius", None)
        controller["goal_checker"] = copy.deepcopy(overlay["goal_checker"])
        controller["FollowPath"] = selected_controller
    elif controller_mode != "mppi":
        raise ValueError(f"Unsupported controller mode: {controller_mode}")

    if controller_mode == "mppi" and not differential:
        tracking_files = ["ackermann_mppi_simulation.yaml"]
        if nominal_vehicle:
            tracking_files.append("nominal_ackermann_navigation.yaml")
        for filename in tracking_files:
            with open(Path(rpp_overlay_file).parent / filename, encoding="utf-8") as stream:
                tracking = yaml.safe_load(stream)["FollowPath"]
            for key, value in tracking.items():
                if isinstance(value, dict):
                    controller["FollowPath"].setdefault(key, {}).update(copy.deepcopy(value))
                else:
                    controller["FollowPath"][key] = copy.deepcopy(value)
        # Large-radius vehicles need samples on their attainable angular scale.
        selected = controller["FollowPath"]
        sampling_limit = selected["wz_max"] * (0.5 if nominal_vehicle else 1.0)
        if selected["wz_std"] > sampling_limit:
            original_std = selected["wz_std"]
            selected["wz_std"] = 0.5 * selected["wz_max"]
            # MPPI's control penalty divides gamma by sampling variance. Keep
            # that angular coefficient when narrowing samples for large vehicles.
            selected["gamma"] *= (selected["wz_std"] / original_std) ** 2
            # Begin heading correction within what this horizon can turn, not
            # the small-car 0.5 rad deadband when total reachable yaw is smaller.
            angle = selected["PathAngleCritic"]
            horizon_yaw = selected["wz_max"] * selected["time_steps"] * selected["model_dt"]
            angle["max_angle_to_furthest"] = min(angle["max_angle_to_furthest"], 0.5 * horizon_yaw)
            # Humble penalizes turning-radius deficits in metres. Normalize
            # that soft cost for large tractors; hard curvature limits remain.
            radius = selected["AckermannConstraints"]["min_turning_r"]
            selected["ConstraintCritic"]["cost_weight"] *= min(1.0, 1.3 / radius)
        if nominal_vehicle:
            # A large Ackermann vehicle needs forward arc length to settle its
            # final heading; the small-car 0.5 m window can begin too late.
            radius = selected["AckermannConstraints"]["min_turning_r"]
            goal_angle = selected["GoalAngleCritic"]
            goal_angle["threshold_to_consider"] = max(goal_angle["threshold_to_consider"], 0.5 * radius)

    if differential:
        with open(Path(rpp_overlay_file).parent / "differential_controllers.yaml", encoding="utf-8") as stream:
            differential_overlay = yaml.safe_load(stream)
        controller["goal_checker"] = copy.deepcopy(differential_overlay["goal_checker"])
        selected_controller = controller["FollowPath"]
        for key, value in differential_overlay[controller_mode].items():
            if isinstance(value, dict):
                selected_controller.setdefault(key, {}).update(copy.deepcopy(value))
            else:
                selected_controller[key] = copy.deepcopy(value)
        if controller_mode == "mppi":
            selected_controller.pop("AckermannConstraints", None)
        elif controller_mode == "rpp":
            # Humble RPP cannot enable rotation-to-heading and reversing together.
            reversing = reverse_speed < 0.0
            selected_controller["allow_reversing"] = reversing
            selected_controller["use_rotate_to_heading"] = not reversing
        else:
            selected_controller["min_vel_x"] = reverse_speed
            selected_controller.pop("min_turning_radius", None)

    if nominal_vehicle and controller_mode == "rpp":
        points = yaml.safe_load(config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
        length = max(point[0] for point in points) - min(point[0] for point in points)
        # Apply after both kinematic overlays: a small robot's 0.5 m carrot
        # over-corrects a multi-metre machine, whether steered or skid-steered.
        selected_controller = controller["FollowPath"]
        for key, ratio in (("min_lookahead_dist", .15), ("lookahead_dist", .2),
                           ("max_lookahead_dist", .3)):
            selected_controller[key] = max(float(selected_controller[key]), ratio * length)
        selected_controller["lookahead_time"] = max(
            float(selected_controller["lookahead_time"]),
            selected_controller["lookahead_dist"] / selected_controller["desired_linear_vel"])

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

    if nominal_vehicle and not differential and planner_mode == "smac_hybrid":
        with open(Path(rpp_overlay_file).parent / "nominal_ackermann_navigation.yaml", encoding="utf-8") as stream:
            planner["GridBased"].update(yaml.safe_load(stream)["GridBased"])
        radius = planner["GridBased"]["minimum_turning_radius"]
        planner["GridBased"]["analytic_expansion_max_length"] = max(
            planner["GridBased"]["analytic_expansion_max_length"], 5.0 * radius)

    if differential and planner_mode != "smac_hybrid":
        # A millimetre-scale final grid segment is not a requested heading.
        # Keep the goal quaternion so DiffDrive can settle at the target pose.
        planner["GridBased"]["use_final_approach_orientation"] = False

    if controller_mode == "mppi" and planner_mode != "smac_hybrid":
        # These XY planners do not enforce final yaw or Ackermann curvature.
        # Their intermediate orientations are not a heading constraint.
        if not differential:
            controller["goal_checker"] = {
                "plugin": "nav2_controller::PositionGoalChecker",
                "xy_goal_tolerance": 0.25,
                "stateful": False,
            }
        controller["FollowPath"]["PathAlignCritic"]["use_path_orientations"] = False
        controller["FollowPath"]["GoalAngleCritic"]["enabled"] = differential

    # Apply last so controller-specific overlays cannot restore a yaw-only
    # terminal manoeuvre after the final position has already been reached.
    with open(Path(rpp_overlay_file).parent / "terminal_stop_simulation.yaml", encoding="utf-8") as stream:
        controller["goal_checker"] = yaml.safe_load(stream)["goal_checker"]
    if not math.isfinite(final_position_tolerance) or final_position_tolerance <= 0.0:
        raise ValueError("Final-position tolerance must be positive and finite")
    controller["goal_checker"]["xy_goal_tolerance"] = final_position_tolerance
    controller["publish_zero_velocity"] = True
    if controller_mode == "mppi":
        controller["FollowPath"]["GoalAngleCritic"]["enabled"] = False
    elif controller_mode == "dwb":
        critics = controller["FollowPath"]["critics"]
        controller["FollowPath"]["critics"] = [name for name in critics if name != "RotateToGoal"]

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
            "max_obstacle_height": max_height,
            "combination_method": int(stvl.get("combination_method", 1)),
            "observation_sources": "lidar_mark lidar_clear",
            "lidar_mark": {
                "topic": marking["topic"],
                "data_type": "PointCloud2",
                "clearing": False,
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
            "lidar_clear": {
                "topic": stvl["lidar_clear"]["topic"],
                "data_type": "PointCloud2",
                "clearing": True,
                "marking": False,
                "obstacle_min_range": 0.0,
                "obstacle_max_range": obstacle_range,
                "raytrace_min_range": 0.0,
                "raytrace_max_range": obstacle_range + 1.0,
                "min_obstacle_height": -10.0,
                "max_obstacle_height": 10.0,
                "expected_update_rate": 0.0,
                "observation_persistence": 0.0,
            },
        }


def _configure_obstacle_height_filter(config):
    """Apply the authored height band in base_link, not a drifting map frame."""
    for costmap_name in ("local_costmap", "global_costmap"):
        layer = config[costmap_name][costmap_name]["ros__parameters"]["stvl_layer"]
        marking = layer["lidar_mark"]
        marking["topic"] = "/configured/obstacle_points"
        marking["min_obstacle_height"] = -10.0
        marking["max_obstacle_height"] = 10.0


def _write_nav2_profiles(source_file, config_dir, localization_name, nominal_vehicle=False,
                         guard_route_progress=False, navigation_footprint=None,
                         final_position_tolerance=0.25):
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
                        nominal_vehicle,
                        navigation_footprint,
                        final_position_tolerance,
                    )
                )
                if guard_route_progress:
                    key = (perception_mode, controller_mode, planner_mode)
                    with open(profiles[key], encoding="utf-8") as stream:
                        guarded = yaml.safe_load(stream)
                    controller = guarded["controller_server"]["ros__parameters"]
                    checker = copy.deepcopy(controller["goal_checker"])
                    checker["wrapped_plugin"] = checker["plugin"]
                    checker["plugin"] = "agribot_vehicle_description::RouteProgressGoalChecker"
                    checker["requested_goal_topic"] = "/current_goal"
                    checker["requested_goal_frame"] = guarded["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"]
                    controller["route_goal_checker"] = checker
                    controller["goal_checker_plugins"] = list(controller["goal_checker_plugins"]) + ["route_goal_checker"]
                    profiles[key] = profiles[key].replace(".generated.yaml", ".route.generated.yaml")
                    with open(profiles[key], "w", encoding="utf-8") as stream:
                        yaml.safe_dump(guarded, stream)
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


def _external_scene_bundle(manifest_file):
    path = Path(manifest_file).expanduser().resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("Unsupported external simulation scene schema")
    result = {}
    for key in ("world", "map", "waypoints"):
        relative = Path(data[key])
        target = (path.parent / relative).resolve()
        if relative.is_absolute() or ".." in relative.parts or not target.is_file():
            raise ValueError(f"Invalid external scene {key}")
        result[key] = str(target)
    spawn = data["spawn"]
    ground = data["spawn_ground_z"]
    if (not isinstance(spawn, list) or len(spawn) != 3 or
            any(type(value) not in (int, float) or not math.isfinite(value)
                for value in [*spawn, ground])):
        raise ValueError("External scene requires finite spawn x/y/yaw and ground height")
    result.update(spawn=spawn, ground=float(ground), directory=path.parent)
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_id", default_value="ackermann_current"),
        DeclareLaunchArgument("scene_manifest", default_value="",
                              description="Optional local scanned-scene bundle; default farm scenes unchanged"),
        DeclareLaunchArgument("scenario", default_value="orchard",
                              choices=["orchard", "arable_field", "greenhouse", "farmyard", "tomato_greenhouse", "strawberry_field", "vineyard", "vegetable_field", "wide_headland"]),
        DeclareLaunchArgument("manipulator", default_value="configured", choices=["configured", "true", "false"]),
        DeclareLaunchArgument("arm_demo", default_value="configured", choices=["configured", "true", "false"]),
        OpaqueFunction(function=_launch_vehicle),
    ])


def _launch_vehicle(context):
    description_share = get_package_share_directory("agribot_vehicle_description")
    simulation_share = get_package_share_directory("agribot_ackermann_mppi")
    navigation_share = get_package_share_directory("scout_navigation")
    gazebo_share = get_package_share_directory("scout_gazebo")
    vehicle_id = LaunchConfiguration("vehicle_id").perform(context)
    import re
    if not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_-]*", vehicle_id):
        raise ValueError("Invalid vehicle_id")
    generated = os.path.join(description_share, "generated", vehicle_id)
    with open(os.path.join(generated, "vehicle_config.json"), encoding="utf-8") as stream:
        import json
        vehicle = json.load(stream)
    had_manipulator = bool(vehicle.get("manipulators"))
    arm_option = LaunchConfiguration("manipulator").perform(context)
    from agribot_vehicle_config_tools.manipulator import default_manipulator
    if arm_option == "true" and not had_manipulator:
        vehicle["manipulators"] = [default_manipulator(vehicle)]
    elif arm_option == "false":
        vehicle["manipulators"] = []
    arm_installed = bool(vehicle.get("manipulators"))
    demo_option = LaunchConfiguration("arm_demo").perform(context)
    if arm_installed and demo_option != "configured":
        vehicle["manipulators"][0]["demoEnabled"] = demo_option == "true"
    from agribot_vehicle_config_tools.implements import collision_boxes as implement_collision_boxes
    implement_boxes = implement_collision_boxes(vehicle, padding=0.03)
    self_filter_enabled = vehicle_id != "ackermann_current" or bool(implement_boxes)
    vehicle_model = _write_gazebo_vehicle_model(
        os.path.join(generated, "models", vehicle_id + ".sdf"),
        "/configured/unfiltered_lidar_points" if self_filter_enabled else None,
        vehicle,
        os.path.join(description_share, "config", "tracked_contact_simulation.yaml"),
    )
    robot_description_file = os.path.join(generated, "urdf", vehicle_id + ".urdf.xacro")
    if had_manipulator or arm_installed or vehicle.get("implements"):
        robot_description_file = _write_manipulator_urdf(robot_description_file, vehicle)
    lidar_sensor = next(sensor for sensor in vehicle["sensors"] if sensor["id"] == "lidar")
    extents = vehicle["geometry"]["bodyExtents"]
    collision = vehicle["collision"]
    body_top = collision["center"][2] + collision["size"][2] * 0.5
    sensor_top = max(sensor["pose"]["xyz"][2] for sensor in vehicle["sensors"]) + 0.1
    self_min = [-extents["rearM"] - 0.03, -extents["rightM"] - 0.03,
                min(wheel["position"][2] - (wheel.get("radiusM") or vehicle["geometry"]["wheelRadiusM"])
                    for wheel in vehicle["geometry"]["wheelCenters"]) - 0.03]
    self_max = [extents["frontM"] + 0.03, extents["leftM"] + 0.03,
                max(body_top, sensor_top)]
    spawn_z = max((wheel.get("radiusM") or vehicle["geometry"]["wheelRadiusM"]) - wheel["position"][2]
                  for wheel in vehicle["geometry"]["wheelCenters"]) + 0.005
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
    fastlio_sim_config = _write_fastlio_sim_config(
        os.path.join(simulation_share, "config", "fast_lio_c16_physical_sim.yaml"),
        os.path.join(generated, "config", "fastlivo_extrinsics.yaml"),
    )
    fastlivo_bridge_config = os.path.join(generated, "config", "physical", "fastlivo_bridge.yaml")
    if vehicle.get("metadata", {}).get("simulationVelocityFromPose") is True:
        with open(fastlivo_bridge_config, encoding="utf-8") as stream:
            bridge_config = yaml.safe_load(stream)
        bridge_config["fastlio_odom_bridge"]["ros__parameters"]["twist_from_pose"] = True
        fastlivo_bridge_config = os.path.join(tempfile.gettempdir(),
            f"configured_{vehicle_id}_fastlivo_bridge.generated.yaml")
        with open(fastlivo_bridge_config, "w", encoding="utf-8") as stream:
            yaml.safe_dump(bridge_config, stream)
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
    with open(fastlio_mppi_config, encoding="utf-8") as stream:
        marking = yaml.safe_load(stream)["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]["lidar_mark"]
    obstacle_min_height = float(marking["min_obstacle_height"])
    obstacle_max_height = float(marking["max_obstacle_height"])
    config_dir = os.path.join(description_share, "config")
    nominal_vehicle = _uses_local_cad_profile(vehicle)
    guard_route_progress = (
        LaunchConfiguration("run_waypoints", default="false").perform(context).lower() == "true"
        and LaunchConfiguration("route_mode", default="planned").perform(context) == "planned"
    )
    navsat_nav2_profiles = _write_nav2_profiles(
        navsat_mppi_config, config_dir, "navsat", nominal_vehicle, guard_route_progress,
        _body_navigation_footprint(vehicle), _final_position_tolerance(vehicle)
    )
    fastlio_nav2_profiles = _write_nav2_profiles(
        fastlio_mppi_config, config_dir, "fastlio", nominal_vehicle, guard_route_progress,
        _body_navigation_footprint(vehicle), _final_position_tolerance(vehicle)
    )
    is_jetson = platform.machine() in ("aarch64", "arm64")
    world_file = os.path.join(gazebo_share, "worlds", "orchard_barriers.world")
    map_directory = os.path.join(navigation_share, "maps")
    map_name = "orchard_v2_map6.yaml"
    waypoint_file = os.path.join(simulation_share, "config", "orchard_waypoints_ackermann_smac.yaml")
    spawn_xy_yaw = [2.0, 36.0, 0.0]
    georeference_file = os.path.join(simulation_share, "config", "orchard_ackermann_sim_georeference.yaml")
    scenario = LaunchConfiguration("scenario").perform(context)
    external_manifest = LaunchConfiguration("scene_manifest").perform(context)
    if external_manifest:
        from agribot_farm_sim.scenarios import write_georeference
        external = _external_scene_bundle(external_manifest)
        world_file = external["world"]
        map_directory, map_name = os.path.split(external["map"])
        waypoint_file = external["waypoints"]
        spawn_xy_yaw = external["spawn"]
        spawn_z += external["ground"]
        georeference_file = write_georeference(
            georeference_file, external["directory"] / "georeference.yaml",
            [*spawn_xy_yaw[:2], spawn_z, spawn_xy_yaw[2]], base_to_antenna_xyz, "external_scene")
    elif scenario == "orchard":
        from agribot_farm_sim.ground import prepare_orchard_ground
        world_file = prepare_orchard_ground(
            world_file, os.path.expanduser("~/.cache/agribot/scenarios/orchard"))
    else:
        import json
        from agribot_farm_sim.scenarios import build, vehicle_spawn, write_georeference, write_vehicle_route
        scenario_share = get_package_share_directory("agribot_farm_sim")
        with open(os.path.join(scenario_share, "config", "scenarios.json"), encoding="utf-8") as stream:
            entry = next(item for item in json.load(stream)["scenarios"] if item["id"] == scenario)
        scene_directory = build(scenario, os.path.expanduser("~/.cache/agribot/scenarios"))
        world_file = str(scene_directory / "world.sdf")
        map_directory = str(scene_directory)
        map_name = "map.yaml"
        waypoint_file = str(write_vehicle_route(scenario, vehicle, scene_directory))
        spawn_xy_yaw = vehicle_spawn(vehicle, scenario, entry["spawn"])
        spawn_z += float(entry.get("spawnZOffset", 0.0))
        georeference_file = write_georeference(
            georeference_file, scene_directory / "georeference.yaml",
            [*spawn_xy_yaw[:2], spawn_z, spawn_xy_yaw[2]], base_to_antenna_xyz, scenario)
    if is_jetson:
        world_file = _write_jetson_world(world_file)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value=os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")),
            SetEnvironmentVariable(
                "CYCLONEDDS_URI",
                os.environ.get("CYCLONEDDS_URI", "file://" + os.path.join(
                    config_dir, "cyclonedds_simulation.xml")),
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("default_nav_to_pose_bt_xml", default_value=os.path.join(
                config_dir, "behavior_trees", "navigate_initial_plan_with_recovery.xml")),
            DeclareLaunchArgument("default_nav_through_poses_bt_xml", default_value=os.path.join(
                config_dir, "behavior_trees", "navigate_through_poses_initial_plan_with_recovery.xml")),
            DeclareLaunchArgument("rviz_config", default_value=_write_simulation_rviz_config(
                os.path.join(get_package_share_directory("agribot_autonomy"), "rviz",
                             "robot_map_global_plan_only.rviz"),
                LaunchConfiguration("vision_mode", default="off").perform(context),
                bool(vehicle.get("metadata", {}).get("simulationVisualBelts")))),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("run_waypoints", default_value="false",
                                  description="Opt in to preset validation routes; normal use selects poses in RViz"),
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
                choices=[
                    "off",
                    "object_detection",
                    "instance_segmentation",
                    "pose_estimation",
                ],
                description=(
                    "Independent visual perception output; it is not connected "
                    "to Nav2 costmaps or motion control"
                ),
            ),
            DeclareLaunchArgument(
                "vision_python",
                default_value=os.environ.get("AGRIBOT_VISION_PYTHON", ""),
            ),
            DeclareLaunchArgument(
                "vision_model_dir",
                default_value=os.environ.get(
                    "AGRIBOT_VISION_MODEL_DIR",
                    os.path.expanduser("~/.local/share/agribot/vision_models"),
                ),
            ),
            DeclareLaunchArgument(
                "vision_device",
                default_value=os.environ.get("AGRIBOT_VISION_DEVICE", ""),
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
                default_value=map_directory,
            ),
            DeclareLaunchArgument("map_file", default_value=map_name),
            Node(
                package="agribot_vehicle_description",
                executable="simulation_visual_belts.py",
                output="screen",
                parameters=[{"use_sim_time": True, "vehicle_config": os.path.join(generated, "vehicle_config.json")}],
                condition=IfCondition("true" if vehicle.get("metadata", {}).get("simulationVisualBelts") else "false"),
            ),
            Node(
                package="agribot_vehicle_description",
                executable="simulation_vehicle_cloud_filter.py",
                parameters=[{
                    "use_sim_time": True,
                    "lidar_frame": lidar_sensor["frameId"],
                    "lidar_xyz": lidar_sensor["pose"]["xyz"],
                    "lidar_rpy": lidar_sensor["pose"]["rpy"],
                    "self_min": [float(value) for value in self_min],
                    "self_max": [float(value) for value in self_max],
                    **({"extra_self_boxes": [float(value) for box in implement_boxes for value in box]}
                       if implement_boxes else {}),
                }],
                output="screen",
                condition=IfCondition("true" if self_filter_enabled else "false"),
            ),
            Node(
                package="agribot_manipulator_sim",
                executable="arm_demo",
                output="screen",
                remappings=[("/cmd_vel", "/nav2/cmd_vel")],
                parameters=[{"use_sim_time": True,
                             "demo_enabled": vehicle["manipulators"][0]["demoEnabled"] if arm_installed else False}],
                condition=IfCondition("true" if arm_installed else "false"),
            ),
            Node(
                package="agribot_vehicle_description",
                executable="simulation_initial_pose.py",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("localization_mode"), "' == 'fastlivo_rtk'",
                ])),
            ),
            Node(
                package="agribot_vehicle_description",
                executable="simulation_lidar_time_adapter.py",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("localization_mode"),
                    "' in ('fast_lio', 'fastlivo_rtk')",
                ])),
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="differential_obstacle_height_filter",
                name="configured_obstacle_height_filter",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "input_topic": "/lidar/points",
                    "output_topic": "/configured/obstacle_points",
                    "base_frame": "base_link",
                    "min_height": obstacle_min_height,
                    "max_height": obstacle_max_height,
                    "transform_timeout_sec": 0.0,
                }],
            ),
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
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "headless": LaunchConfiguration("headless"),
                    "initial_pose_z": str(spawn_z),
                    "initial_pose_x": str(spawn_xy_yaw[0]),
                    "initial_pose_y": str(spawn_xy_yaw[1]),
                    "initial_pose_yaw": str(spawn_xy_yaw[2]),
                    "gazebo_pause_until_spawn": "true",
                    # One-time map anchor after settling, never motion feedback.
                    "use_settled_spawn_pose": "true",
                    "gazebo_transport_library_path": _gazebo_transport_compat_path(),
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
                    "waypoint_goal_checker_id": "route_goal_checker" if guard_route_progress else "",
                    "rviz_start_delay": LaunchConfiguration("rviz_start_delay"),
                    "waypoint_file": waypoint_file,
                    "navigation_delay": LaunchConfiguration("navigation_delay"),
                    "localization_mode": LaunchConfiguration("localization_mode"),
                    "use_static_map": LaunchConfiguration("use_static_map"),
                    "enable_slam_map": LaunchConfiguration("enable_slam_map"),
                    "map_file_location": LaunchConfiguration("map_file_location"),
                    "map_file": LaunchConfiguration("map_file"),
                    "world": world_file,
                    "gazebo_spawn_file": vehicle_model,
                    "robot_description_file": robot_description_file,
                    "navsat_static_nav2_params_file": _nav2_profile_config(
                        navsat_nav2_profiles
                    ),
                    "fastlio_static_nav2_params_file": _nav2_profile_config(
                        fastlio_nav2_profiles
                    ),
                    "fastlivo_lidar_config_file": fastlivo_sim_config,
                    "fastlivo_georeference_file": georeference_file,
                    "fastlio_config_file": fastlio_sim_config,
                    "fastlivo_camera_config_file": os.path.join(
                        simulation_share,
                        "config",
                        "fastlivo_hikrobot_640_sim.yaml",
                    ),
                    "fastlivo_bridge_config_file": fastlivo_bridge_config,
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
                    "default_nav_to_pose_bt_xml": LaunchConfiguration("default_nav_to_pose_bt_xml"),
                    "default_nav_through_poses_bt_xml": LaunchConfiguration("default_nav_through_poses_bt_xml"),
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
                prefix=LaunchConfiguration("vision_python"),
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
                        "result_topic": "/vision/results",
                        "status_topic": "/vision/status",
                        "model_dir": LaunchConfiguration("vision_model_dir"),
                        "device": LaunchConfiguration("vision_device"),
                        "confidence": 0.35,
                        "image_size": 640,
                        "max_rate_hz": 10.0,
                    }
                ],
            ),
        ]
    ).entities
