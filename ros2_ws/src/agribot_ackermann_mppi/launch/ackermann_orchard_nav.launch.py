import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ackermann_share = get_package_share_directory("agribot_ackermann_mppi")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    xacro_exec = os.path.join(get_package_prefix("xacro"), "bin", "xacro")
    description_file = os.path.join(ackermann_share, "urdf", "ackermann_scout.urdf.xacro")
    gazebo_spawn_file = os.path.join(ackermann_share, "models", "ackermann_scout.sdf")
    sensor_spawn_file = os.path.join(ackermann_share, "models", "ackermann_scout_sensor.sdf")

    system_model_paths = [
        model_path
        for model_path in (
            "/usr/share/gazebo-11/models",
            "/usr/share/gazebo/models",
            os.path.expanduser("~/.gazebo/models"),
            os.path.dirname(scout_gazebo_share),
            os.path.dirname(ackermann_share),
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
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        ]
        if path
    )

    robot_description = ParameterValue(
        Command(
            [
                xacro_exec,
                " ",
                description_file,
                " ",
                "robot_namespace:=",
                LaunchConfiguration("robot_namespace"),
                " ",
                "laser_enabled:=true",
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="ackermann_scout"),
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
            DeclareLaunchArgument("z", default_value="0.24"),
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
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "ackermann_scout_sensor",
                    "-file",
                    sensor_spawn_file,
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
