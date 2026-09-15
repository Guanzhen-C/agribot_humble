import copy
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


def _write_scene_sync_world(source_file):
    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_fastlivo_orchard.generated.world"
    )
    world_tree = ET.parse(source_file)
    world = world_tree.getroot().find("world")
    if world is None:
        raise RuntimeError(f"Gazebo world file has no <world>: {source_file}")

    # RViz is the user-facing display for this launch; shadow rendering in the
    # hidden Gazebo scene only consumes GPU time and does not affect sensing.
    scene = world.find("scene")
    if scene is not None:
        shadows = scene.find("shadows")
        if shadows is not None:
            shadows.text = "0"

    # The fixed tractor is scenery. Its 1 kHz seat-contact sensor has no
    # consumer but still wakes on every physics step in Gazebo Classic.
    for model in world.findall("model"):
        if (model.findtext("static") or "").strip() not in ("1", "true"):
            continue
        for link in model.findall("link"):
            for sensor in list(link.findall("sensor")):
                link.remove(sensor)

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


def _write_realtime_sensor_robot(ackermann_share):
    """Build the long-run robot with the validated CPU simulation sampling."""
    robot_tree = ET.parse(
        os.path.join(ackermann_share, "models", "ackermann_scout.sdf")
    )
    robot_model = robot_tree.getroot().find("model")
    sensor_model = ET.parse(
        os.path.join(ackermann_share, "models", "ackermann_scout_sensor.sdf")
    ).getroot().find("model")
    if robot_model is None or sensor_model is None:
        raise RuntimeError("Robot and sensor SDF files must each contain one <model>")

    robot_base = robot_model.find("./link[@name='base_link']")
    sensor_base = sensor_model.find("./link[@name='base_link']")
    if robot_base is None or sensor_base is None:
        raise RuntimeError("Robot and sensor SDF files must each contain base_link")

    sensor_names = {sensor.get("name") for sensor in sensor_base.findall("sensor")}
    for sensor in list(robot_base.findall("sensor")):
        if sensor.get("name") in sensor_names:
            robot_base.remove(sensor)
    for sensor in sensor_base.findall("sensor"):
        robot_base.append(copy.deepcopy(sensor))

    lidar = robot_base.find("./sensor[@name='lslidar_c16_points']")
    if lidar is None:
        raise RuntimeError("Sensor SDF has no lslidar_c16_points sensor")
    samples = lidar.find("./ray/scan/horizontal/samples")
    ray_max_range = lidar.find("./ray/range/max")
    plugin_max_range = lidar.find("./plugin/max_range")
    if samples is None or ray_max_range is None or plugin_max_range is None:
        raise RuntimeError("C16 SDF is missing ray sampling parameters")

    # Gazebo Classic's CPU ray sensor is single-core bound in this orchard.
    # 720 azimuth samples retains all 16 rings at 10 Hz and matches the
    # previously validated long-run simulator while leaving the full physical
    # sensor definition untouched for other launches.
    samples.text = "720"
    ray_max_range.text = "25.0"
    plugin_max_range.text = "25.0"

    camera = robot_base.find("./sensor[@name='hikrobot_right_camera']")
    if camera is None:
        raise RuntimeError("Sensor SDF has no hikrobot_right_camera sensor")
    image_width = camera.find("./camera/image/width")
    image_height = camera.find("./camera/image/height")
    camera_info_url = camera.find("./plugin/camera_info_url")
    if image_width is None or image_height is None or camera_info_url is None:
        raise RuntimeError("Hikrobot simulation camera is missing image parameters")
    # FAST-LIVO2 previously resized 1280x1024 by 0.5 internally. Render that
    # exact working resolution directly and avoid producing discarded pixels.
    image_width.text = "640"
    image_height.text = "512"
    camera_info_url.text = (
        "package://agribot_ackermann_mppi/config/hikrobot_camera_640_sim.yaml"
    )

    publish_tf = robot_model.find(".//publish_tf")
    if publish_tf is not None:
        publish_tf.text = "false"
    sensor_model_name = robot_model.find(".//sensor_model_name")
    if sensor_model_name is not None:
        sensor_model_name.text = ""

    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_long_route_robot.generated.sdf"
    )
    ET.indent(robot_tree, space="  ")
    robot_tree.write(target_file, encoding="unicode", xml_declaration=False)
    return target_file


def _write_long_route_fastlivo_config(source_file):
    """Use the stable orchard-simulation estimator tuning with real extrinsics."""
    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_long_route_fastlivo.generated.yaml"
    )
    with open(source_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    params = config["/**"]["ros__parameters"]
    # Synthetic orchard geometry is much more repetitive than the recorded
    # physical scenes. These are the settings already validated by the
    # differential FAST-LIVO2 simulator; all calibrated Ackermann extrinsics
    # and all real-vehicle configuration files remain untouched.
    params["lio"]["voxel_size"] = 1.0
    params["vio"]["img_point_cov"] = 800
    params["time_offset"]["img_time_offset"] = 0.0

    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _write_long_route_nav2_config(source_file):
    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_long_route_nav2.generated.yaml"
    )
    with open(source_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    grid_based = config["planner_server"]["ros__parameters"]["GridBased"]
    # ComputePathThroughPoses plans all 23 mandatory legs before motion starts.
    # The regular 5 s limit is per single-goal operation and is too short for
    # this one-shot route preflight.
    grid_based["max_planning_time"] = 20.0

    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def _write_compatible_rviz_config(source_file):
    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_fastlivo_old_display.generated.rviz"
    )
    with open(source_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    displays = config.get("Visualization Manager", {}).get("Displays", [])
    for display in displays:
        topic = display.get("Topic")
        if not isinstance(topic, dict):
            continue
        if display.get("Name") == "3D Lidar PointCloud2":
            topic["Value"] = "/lidar/points"
            topic["Reliability Policy"] = "Best Effort"
        elif display.get("Name") == "RGB Image":
            topic["Value"] = "/camera/rgb/image_raw"
            topic["Reliability Policy"] = "Best Effort"

    with open(target_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return target_file


def generate_launch_description():
    ackermann_share = get_package_share_directory("agribot_ackermann_mppi")
    autonomy_share = get_package_share_directory("agribot_autonomy")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    fastlivo_share = get_package_share_directory("fast_livo")

    is_jetson = platform.machine() in ("aarch64", "arm64")
    world_file = os.path.join(
        scout_gazebo_share, "worlds", "orchard_barriers.world"
    )
    if is_jetson:
        world_file = _write_scene_sync_world(world_file)

    realtime_sensor_robot = _write_realtime_sensor_robot(ackermann_share)
    fastlivo_config = _write_long_route_fastlivo_config(
        os.path.join(fastlivo_share, "config", "agribot_c16_astra.yaml")
    )

    nav2_config = _write_long_route_nav2_config(
        os.path.join(
            ackermann_share,
            "config",
            "nav2_params_ackermann_fastlio_static.yaml",
        )
    )

    rviz_config = _write_compatible_rviz_config(
        os.path.join(
            autonomy_share, "rviz", "robot_map_global_plan_only.rviz"
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("run_waypoints", default_value="true"),
            DeclareLaunchArgument("waypoint_startup_delay", default_value="1.0"),
            DeclareLaunchArgument("initial_pose_x", default_value="2.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="36.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.1275"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        ackermann_share,
                        "launch",
                        "ackermann_waypoint_depth_collect_sim.launch.py",
                    )
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    # A rendering engine is required even without gzclient.
                    "headless": "false",
                    "gazebo_render_workaround": "true" if is_jetson else "false",
                    "gazebo_spawn_file": realtime_sensor_robot,
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": rviz_config,
                    "run_waypoints": LaunchConfiguration("run_waypoints"),
                    "waypoint_navigation_mode": "plan_then_follow_path",
                    "waypoint_startup_delay": LaunchConfiguration(
                        "waypoint_startup_delay"
                    ),
                    "waypoint_file": os.path.join(
                        ackermann_share,
                        "config",
                        "orchard_waypoints_ackermann_smac.yaml",
                    ),
                    "fastlivo_camera_config_file": os.path.join(
                        ackermann_share,
                        "config",
                        "fastlivo_hikrobot_640_sim.yaml",
                    ),
                    "fastlivo_lidar_config_file": fastlivo_config,
                    "use_static_map": "true",
                    "localization_mode": "fastlivo_rtk",
                    "enable_slam_map": "false",
                    "world": world_file,
                    "map_file_location": os.path.join(
                        scout_navigation_share, "maps"
                    ),
                    "map_file": "orchard_v2_map6.yaml",
                    "fastlio_static_nav2_params_file": nav2_config,
                    "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                    "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                    "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                    "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                }.items(),
            ),
        ]
    )
