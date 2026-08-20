import os
import platform
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _write_fastlivo_world(source_file):
    target_file = os.path.join(
        tempfile.gettempdir(), "differential_fastlivo.generated.world"
    )
    world_tree = ET.parse(source_file)
    world = world_tree.getroot().find("world")
    if world is None:
        raise RuntimeError(f"Gazebo world file has no <world>: {source_file}")

    plugin_name = "agribot_differential_gazebo_scene_sync"
    existing = world.find(f"./plugin[@name='{plugin_name}']")
    if existing is not None:
        world.remove(existing)
    ET.SubElement(
        world,
        "plugin",
        {
            "name": plugin_name,
            "filename": "libagribot_differential_gazebo_scene_sync.so",
        },
    )
    ET.indent(world_tree, space="  ")
    world_tree.write(target_file, encoding="unicode", xml_declaration=False)
    return target_file


def generate_launch_description():
    simulation_share = get_package_share_directory(
        "agribot_differential_fastlivo_mppi"
    )
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")
    config_dir = os.path.join(simulation_share, "config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_jetson = platform.machine() in ("aarch64", "arm64")
    world_file = os.path.join(
        scout_gazebo_share, "worlds", "orchard_barriers.world"
    )
    if is_jetson:
        world_file = _write_fastlivo_world(world_file)

    simulation_prefix = get_package_prefix(
        "agribot_differential_fastlivo_mppi"
    )
    plugin_path = os.pathsep.join(
        value
        for value in (
            os.path.join(simulation_prefix, "lib"),
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        )
        if value
    )
    render_workaround = os.path.join(
        simulation_prefix,
        "lib",
        "libagribot_differential_gazebo_render_workaround.so",
    )
    preload = os.pathsep.join(
        value
        for value in (render_workaround, os.environ.get("LD_PRELOAD", ""))
        if value
    )

    gazebo = GroupAction(
        scoped=True,
        actions=[
            # Make all three simulated sensor origins coincide at the VLP16
            # optical center. This keeps simulation extrinsics deterministic.
            SetEnvironmentVariable("SCOUT_IMU_XYZ", "0 0 0.5777"),
            SetEnvironmentVariable("SCOUT_IMU_RPY", "0 0 0"),
            SetEnvironmentVariable("SCOUT_REALSENSE_ENABLED", "0"),
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", plugin_path),
            *(
                [
                    SetEnvironmentVariable(
                        "AGRIBOT_GAZEBO_RENDER_PATH_WORKAROUND", "1"
                    ),
                    SetEnvironmentVariable("LD_PRELOAD", preload),
                ]
                if is_jetson
                else []
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        scout_gazebo_share,
                        "launch",
                        "scout_orchard_world.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world_name": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": "false",
                    "use_xvfb": LaunchConfiguration("use_xvfb"),
                    "rviz": "false",
                    "x": LaunchConfiguration("start_x"),
                    "y": LaunchConfiguration("start_y"),
                    "z": "0.146336",
                    "yaw": LaunchConfiguration("start_yaw"),
                    "laser_enabled": "false",
                    "laser_3d_enabled": "true",
                    "laser_3d_topic": "/lidar/points",
                    "laser_3d_update_rate": "10",
                    "laser_3d_horizontal_samples": "720",
                    "laser_3d_vertical_samples": "16",
                    "laser_3d_min_range": "0.3",
                    "laser_3d_max_range": "25.0",
                    "publish_odom_tf": "false",
                    "publish_ground_truth": "true",
                    "publish_joint_states": "true",
                    "urdf_extras": os.path.join(
                        simulation_share,
                        "urdf",
                        "differential_sim_rgb_camera.urdf.xacro",
                    ),
                }.items(),
            ),
        ],
    )

    navigation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hardware_share,
                        "launch",
                        "include",
                        "navigation_only.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "odom_topic": "/fastlivo_rtk/odometry",
                    "map_topic": "/map",
                    "lattice_filepath": LaunchConfiguration("lattice_filepath"),
                    "default_nav_to_pose_bt_xml": os.path.join(
                        nav2_bt_share,
                        "behavior_trees",
                        "navigate_to_pose_w_replanning_and_recovery.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        nav2_bt_share,
                        "behavior_trees",
                        "navigate_through_poses_w_replanning_and_recovery.xml",
                    ),
                }.items(),
            )
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_xvfb", default_value="false"),
            DeclareLaunchArgument("auto_run", default_value="false"),
            DeclareLaunchArgument("world", default_value=world_file),
            DeclareLaunchArgument("start_x", default_value="2.0"),
            DeclareLaunchArgument("start_y", default_value="35.0"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            DeclareLaunchArgument("test_goal_x", default_value="35.0"),
            DeclareLaunchArgument("test_goal_y", default_value="35.5"),
            DeclareLaunchArgument("test_goal_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    scout_navigation_share, "maps", "orchard_v2_map6.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    config_dir, "nav2_params_scout_orchard_sim.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "lattice_filepath",
                default_value=os.path.join(
                    config_dir,
                    "motion_primitives",
                    "scout_diff_5cm.json",
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(hardware_share, "rviz", "navigation.rviz"),
            ),
            gazebo,
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="fastlivo_mapping",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "fastlivo_sim.yaml"),
                    os.path.join(config_dir, "fastlivo_sim_camera.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlio_odom_bridge.py",
                name="fastlio_odom_bridge",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "fastlivo_sim_bridge.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_fastlivo_world",
                arguments=["--frame-id", "odom", "--child-frame-id", "camera_init"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="agribot_differential_fastlivo_mppi",
                executable="differential_sim_sensor_bridge.py",
                name="differential_sim_sensor_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "reference_latitude_deg": 30.5,
                        "reference_longitude_deg": 114.0,
                        "reference_altitude_m": 20.0,
                        "fix_rate_hz": 10.0,
                        "horizontal_sigma_m": 0.03,
                        "fix_quality": 4,
                    }
                ],
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="fastlivo_rtk_fusion",
                name="fastlivo_rtk_fusion",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "fastlivo_rtk_fusion_sim.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "georeference_file": os.path.join(
                            config_dir, "orchard_sim_georeference.yaml"
                        ),
                        "map_file": "",
                        "allow_missing_georeference": False,
                        "auto_initialize_from_fixed_rtk": True,
                    },
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": LaunchConfiguration("map"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_differential_sim_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            navigation,
            Node(
                package="agribot_differential_fastlivo_mppi",
                executable="differential_sim_cmd_bridge.py",
                name="differential_sim_cmd_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="agribot_differential_fastlivo_mppi",
                executable="differential_sim_validator.py",
                name="differential_sim_validator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "goal_x": ParameterValue(
                            LaunchConfiguration("test_goal_x"), value_type=float
                        ),
                        "goal_y": ParameterValue(
                            LaunchConfiguration("test_goal_y"), value_type=float
                        ),
                        "goal_yaw": ParameterValue(
                            LaunchConfiguration("test_goal_yaw"), value_type=float
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("auto_run")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="differential_fastlivo_rtk_sim_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
