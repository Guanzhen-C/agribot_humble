import copy
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _write_sensor_integrated_sdf(ackermann_share):
    robot_tree = ET.parse(
        os.path.join(ackermann_share, "models", "ackermann_scout.sdf")
    )
    sensor_tree = ET.parse(
        os.path.join(ackermann_share, "models", "ackermann_scout_sensor.sdf")
    )
    robot_model = robot_tree.getroot().find("model")
    sensor_model = sensor_tree.getroot().find("model")
    robot_base = robot_model.find("./link[@name='base_link']") if robot_model is not None else None
    sensor_base = sensor_model.find("./link[@name='base_link']") if sensor_model is not None else None
    if robot_base is None or sensor_base is None:
        raise RuntimeError("Robot and sensor SDF files must each contain base_link")
    for sensor in sensor_base.findall("sensor"):
        robot_base.append(copy.deepcopy(sensor))

    publish_tf = robot_model.find(".//publish_tf")
    if publish_tf is not None:
        publish_tf.text = "false"

    target = os.path.join(
        tempfile.gettempdir(), "ackermann_orchard_sensor_integrated.generated.sdf"
    )
    ET.indent(robot_tree, space="  ")
    robot_tree.write(target, encoding="unicode", xml_declaration=False)
    return target


def generate_launch_description():
    ackermann_share = get_package_share_directory("agribot_ackermann_mppi")
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    xacro_exec = os.path.join(get_package_prefix("xacro"), "bin", "xacro")
    description_file = os.path.join(ackermann_share, "urdf", "ackermann_scout.urdf.xacro")
    gazebo_spawn_file = _write_sensor_integrated_sdf(ackermann_share)

    system_model_paths = [
        model_path
        for model_path in (
            "/usr/share/gazebo-11/models",
            "/usr/share/gazebo/models",
            os.path.expanduser("~/.gazebo/models"),
            os.path.dirname(scout_gazebo_share),
            os.path.dirname(ackermann_share),
            os.path.dirname(hardware_share),
        )
        if os.path.isdir(model_path)
    ]
    gazebo_model_path = os.pathsep.join(
        system_model_paths
        + ([os.environ["GAZEBO_MODEL_PATH"]] if os.environ.get("GAZEBO_MODEL_PATH") else [])
    )
    gazebo_plugin_path = os.pathsep.join(
        path
        for path in [
            os.path.join(get_package_prefix("agribot_ackermann_mppi"), "lib"),
            os.path.join(get_package_prefix("velodyne_gazebo_plugins"), "lib"),
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        ]
        if path
    )

    robot_description = ParameterValue(
        Command([xacro_exec, " ", description_file]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="agribot_ackermann"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(scout_gazebo_share, "worlds", "orchard_barriers.world"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(scout_navigation_share, "maps", "orchard_v2_map6.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    ackermann_share, "config", "nav2_params_ackermann.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    ackermann_share,
                    "behavior_trees",
                    "navigate_w_replanning_and_ackermann_recovery.xml",
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_through_poses_bt_xml",
                default_value=os.path.join(
                    ackermann_share,
                    "behavior_trees",
                    "navigate_through_poses_w_replanning_ackermann.xml",
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument("x", default_value="2.0"),
            DeclareLaunchArgument("y", default_value="36.0"),
            DeclareLaunchArgument("z", default_value="0.1275"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "verbose": "false",
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            LaunchConfiguration("robot_name"),
                            "-file",
                            gazebo_spawn_file,
                            "-x",
                            LaunchConfiguration("x"),
                            "-y",
                            LaunchConfiguration("y"),
                            "-z",
                            LaunchConfiguration("z"),
                            "-Y",
                            LaunchConfiguration("yaw"),
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="pointcloud_ring_to_laserscan",
                name="c16_horizontal_scan",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_cloud_topic": "/lidar/points",
                        "output_scan_topic": "/scan",
                        "ring_index": 8,
                        "beam_count": 2000,
                        "range_min": 0.3,
                        "range_max": 100.0,
                        "scan_time": 0.1,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_navigation_share, "launch", "amcl_navigation.launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "default_nav_to_pose_bt_xml": LaunchConfiguration(
                        "default_nav_to_pose_bt_xml"
                    ),
                    "default_nav_through_poses_bt_xml": LaunchConfiguration(
                        "default_nav_through_poses_bt_xml"
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "navigation_delay": "8.0",
                    "odom_topic": "/odom",
                    "start_robot": "false",
                }.items(),
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                name="ackermann_initial_pose_sender",
                output="screen",
                parameters=[
                    {
                        "x": LaunchConfiguration("x"),
                        "y": LaunchConfiguration("y"),
                        "yaw": LaunchConfiguration("yaw"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "startup_delay": 8.0,
                        "publish_count": 1,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                        "stamp_offset_sec": -0.5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("publish_initial_pose")),
            ),
        ]
    )
