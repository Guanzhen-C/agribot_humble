import copy
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _write_orchard_geometry_sdf(gazebo_share):
    mesh_dir = os.path.join(gazebo_share, "meshes")
    orchard_model_file = os.path.join(
        tempfile.gettempdir(), "ackermann_orchard_geometry.generated.sdf"
    )
    orchard_model_xml = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="orchard_geometry">
    <static>true</static>
    <link name="orchard_world_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
      <visual name="ground">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_world.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="trunks">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_trunks.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="leaves">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_leaves.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="ground_collision">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_world.dae</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100.0</mu>
              <mu2>50.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="trunks_collision">
        <geometry>
          <mesh>
            <uri>file://{mesh_dir}/orchard_trunks.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    with open(orchard_model_file, "w", encoding="utf-8") as orchard_file:
        orchard_file.write(orchard_model_xml)
    return orchard_model_file


def _write_ackermann_localized_sdf(ackermann_share):
    source_file = os.path.join(ackermann_share, "models", "ackermann_scout.sdf")
    sensor_file = os.path.join(
        ackermann_share, "models", "ackermann_scout_sensor.sdf"
    )
    target_file = os.path.join(
        tempfile.gettempdir(), "ackermann_scout_localized.generated.sdf"
    )

    robot_tree = ET.parse(source_file)
    robot_model = robot_tree.getroot().find("model")
    sensor_model = ET.parse(sensor_file).getroot().find("model")
    if robot_model is None or sensor_model is None:
        raise RuntimeError("Robot and sensor SDF files must each contain one <model>")

    robot_base_link = robot_model.find("./link[@name='base_link']")
    sensor_base_link = sensor_model.find("./link[@name='base_link']")
    if robot_base_link is None or sensor_base_link is None:
        raise RuntimeError("Robot and sensor SDF files must each contain base_link")

    sensor_names = {
        sensor.get("name") for sensor in sensor_base_link.findall("sensor")
    }
    for existing_sensor in list(robot_base_link.findall("sensor")):
        if existing_sensor.get("name") in sensor_names:
            robot_base_link.remove(existing_sensor)
    for sensor in sensor_base_link.findall("sensor"):
        robot_base_link.append(copy.deepcopy(sensor))

    publish_tf = robot_model.find(".//publish_tf")
    if publish_tf is not None:
        publish_tf.text = "false"
    sensor_model_name = robot_model.find(".//sensor_model_name")
    if sensor_model_name is not None:
        sensor_model_name.text = ""

    ET.indent(robot_tree, space="  ")
    robot_tree.write(target_file, encoding="unicode", xml_declaration=False)
    return target_file


def generate_launch_description():
    ackermann_share = get_package_share_directory("agribot_ackermann_rpp")
    autonomy_share = get_package_share_directory("agribot_autonomy")
    rl_nav_share = get_package_share_directory("agribot_rl_nav")
    scout_gazebo_share = get_package_share_directory("scout_gazebo")
    scout_navigation_share = get_package_share_directory("scout_navigation")
    scout_viz_share = get_package_share_directory("scout_viz")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    xacro_exec = os.path.join(get_package_prefix("xacro"), "bin", "xacro")
    description_file = os.path.join(ackermann_share, "urdf", "ackermann_scout.urdf.xacro")
    localized_gazebo_spawn_file = _write_ackermann_localized_sdf(ackermann_share)
    orchard_model_file = _write_orchard_geometry_sdf(scout_gazebo_share)
    map_path = PathJoinSubstitution(
        [LaunchConfiguration("map_file_location"), LaunchConfiguration("map_file")]
    )
    localization_mode = LaunchConfiguration("localization_mode")
    use_static_map = LaunchConfiguration("use_static_map")
    static_navsat_condition = IfCondition(
        PythonExpression(
            ["'", use_static_map, "' == 'true' and '", localization_mode, "' == 'navsat'"]
        )
    )
    static_fastlio_condition = IfCondition(
        PythonExpression(
            ["'", use_static_map, "' == 'true' and '", localization_mode, "' == 'fast_lio'"]
        )
    )
    navsat_condition = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'navsat'"])
    )
    fastlio_condition = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'fast_lio'"])
    )

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
            os.path.join(get_package_prefix("agribot_ackermann_rpp"), "lib"),
            os.path.join(get_package_prefix("velodyne_gazebo_plugins"), "lib"),
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
                "laser_enabled:=false",
            ]
        ),
        value_type=str,
    )

    robot_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            LaunchConfiguration("robot_name"),
            "-file",
            localized_gazebo_spawn_file,
            "-x",
            LaunchConfiguration("initial_pose_x"),
            "-y",
            LaunchConfiguration("initial_pose_y"),
            "-z",
            LaunchConfiguration("initial_pose_z"),
            "-Y",
            LaunchConfiguration("initial_pose_yaw"),
        ],
        output="screen",
    )
    gzclient_after_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    def _map_server_groups(context):
        resolved_map_path = map_path.perform(context)
        resolved_navigation_delay = LaunchConfiguration("navigation_delay").perform(context)
        resolved_nav_odom_topic = LaunchConfiguration("nav_odom_topic").perform(context)
        resolved_navsat_static_nav2_params = LaunchConfiguration(
            "navsat_static_nav2_params_file"
        ).perform(context)
        resolved_fastlio_static_nav2_params = LaunchConfiguration(
            "fastlio_static_nav2_params_file"
        ).perform(context)
        resolved_default_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml").perform(context)
        resolved_default_through_poses_bt_xml = LaunchConfiguration(
            "default_nav_through_poses_bt_xml"
        ).perform(context)

        static_navsat_localization_gate = Node(
            package="agribot_autonomy",
            executable="topic_ready_gate.py",
            name="ackermann_static_navsat_localization_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "topic": "/amcl_pose",
                    "message_type": "pose",
                    "timeout_sec": float(resolved_navigation_delay) + 20.0,
                }
            ],
        )
        static_fastlio_localization_gate = Node(
            package="agribot_autonomy",
            executable="topic_ready_gate.py",
            name="ackermann_static_fastlio_localization_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "topic": "/amcl_pose",
                    "message_type": "pose",
                    "timeout_sec": float(resolved_navigation_delay) + 20.0,
                }
            ],
        )

        def static_map_nodes():
            return [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[
                        {
                            "yaml_filename": resolved_map_path,
                            "use_sim_time": True,
                        }
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package="nav2_lifecycle_manager",
                            executable="lifecycle_manager",
                            name="lifecycle_manager_map_server",
                            output="screen",
                            parameters=[
                                {"use_sim_time": True},
                                {"autostart": True},
                                {"node_names": ["map_server"]},
                            ],
                        ),
                    ],
                ),
            ]

        return [
            GroupAction(
                condition=static_navsat_condition,
                actions=static_map_nodes()
                + [
                    TimerAction(
                        period=float(resolved_navigation_delay),
                        actions=[static_navsat_localization_gate],
                    ),
                    RegisterEventHandler(
                        event_handler=OnProcessExit(
                            target_action=static_navsat_localization_gate,
                            on_exit=[
                                IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                        os.path.join(
                                            scout_navigation_share,
                                            "launch",
                                            "include",
                                            "navigation_only.launch.py",
                                        )
                                    ),
                                    launch_arguments={
                                        "use_sim_time": "true",
                                        "autostart": "true",
                                        "params_file": resolved_navsat_static_nav2_params,
                                        "odom_topic": resolved_nav_odom_topic,
                                        "default_nav_to_pose_bt_xml": resolved_default_bt_xml,
                                        "default_nav_through_poses_bt_xml": (
                                            resolved_default_through_poses_bt_xml
                                        ),
                                    }.items(),
                                )
                            ],
                        )
                    ),
                ],
            ),
            GroupAction(
                condition=static_fastlio_condition,
                actions=static_map_nodes()
                + [
                    TimerAction(
                        period=float(resolved_navigation_delay),
                        actions=[static_fastlio_localization_gate],
                    ),
                    RegisterEventHandler(
                        event_handler=OnProcessExit(
                            target_action=static_fastlio_localization_gate,
                            on_exit=[
                                IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                        os.path.join(
                                            scout_navigation_share,
                                            "launch",
                                            "include",
                                            "navigation_only.launch.py",
                                        )
                                    ),
                                    launch_arguments={
                                        "use_sim_time": "true",
                                        "autostart": "true",
                                        "params_file": resolved_fastlio_static_nav2_params,
                                        "odom_topic": resolved_nav_odom_topic,
                                        "default_nav_to_pose_bt_xml": resolved_default_bt_xml,
                                        "default_nav_through_poses_bt_xml": (
                                            resolved_default_through_poses_bt_xml
                                        ),
                                    }.items(),
                                )
                            ],
                        )
                    ),
                ],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="ackermann_scout"),
            DeclareLaunchArgument("robot_namespace", default_value="/"),
            DeclareLaunchArgument(
                "gazebo_ip",
                default_value=os.environ.get("GAZEBO_IP", "172.17.0.1"),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_start_delay", default_value="5.0"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_static_map", default_value="true"),
            DeclareLaunchArgument("localization_mode", default_value="navsat"),
            DeclareLaunchArgument("enable_slam_map", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "nav_odom_topic",
                default_value=PythonExpression(
                    [
                        "'/fastlio/odometry' if '",
                        localization_mode,
                        "' == 'fast_lio' else ('/odometry/filtered_navsat' if '",
                        localization_mode,
                        "' == 'navsat' else '/odom')",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "navigation_delay",
                default_value="22.0",
            ),
            DeclareLaunchArgument("map_file_location", default_value=os.path.join(scout_navigation_share, "maps")),
            DeclareLaunchArgument("map_file", default_value="orchard_v2_map6.yaml"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(scout_gazebo_share, "worlds", "orchard_barriers.world"),
            ),
            DeclareLaunchArgument(
                "navsat_static_nav2_params_file",
                default_value=os.path.join(
                    ackermann_share, "config", "nav2_params_ackermann_navsat_static.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_static_nav2_params_file",
                default_value=os.path.join(
                    ackermann_share, "config", "nav2_params_ackermann_fastlio_static.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "navsat_ekf_params_file",
                default_value=os.path.join(rl_nav_share, "config", "navsat_kf_gins_map.yaml"),
            ),
            DeclareLaunchArgument(
                "fastlio_config_file",
                default_value=os.path.join(autonomy_share, "config", "fast_lio_sim_tuned.yaml"),
            ),
            DeclareLaunchArgument("fastlio_start_delay", default_value="8.0"),
            DeclareLaunchArgument("fastlio_localization_start_delay", default_value="20.0"),
            DeclareLaunchArgument("fastlio_visualize", default_value="false"),
            DeclareLaunchArgument("navsat_pose_topic", default_value="/odometry/gps"),
            DeclareLaunchArgument("navsat_pose_message_type", default_value="odometry"),
            DeclareLaunchArgument("navsat_imu_topic", default_value="/imu/data_corrected"),
            DeclareLaunchArgument(
                "navsat_auto_reference_from_first_noah_gnss",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    ackermann_share,
                    "behavior_trees",
                    "navigate_w_replanning_ackermann_no_spin.xml",
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
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    autonomy_share, "rviz", "robot_map_global_plan_only.rviz"
                ),
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=os.path.join(
                    autonomy_share, "config", "orchard_waypoints_default_start.yaml"
                ),
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="2.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="36.0"),
            DeclareLaunchArgument("initial_pose_z", default_value="0.24"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("waypoint_transform_enabled", default_value="false"),
            DeclareLaunchArgument("navsat_reference_lat", default_value="30.5"),
            DeclareLaunchArgument("navsat_reference_lon", default_value="114.0"),
            DeclareLaunchArgument("navsat_reference_alt", default_value="20.0"),
            DeclareLaunchArgument("spawn_orchard_geometry", default_value="false"),
            SetEnvironmentVariable("GAZEBO_IP", LaunchConfiguration("gazebo_ip")),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="ackermann_collision_monitor",
                output="screen",
                parameters=[
                    os.path.join(
                        autonomy_share, "config", "collision_monitor_ackermann.yaml"
                    ),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "autostart": True,
                        "node_names": ["ackermann_collision_monitor"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "headless": LaunchConfiguration("headless"),
                    "verbose": "false",
                }.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "orchard_geometry",
                    "-file",
                    orchard_model_file,
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("spawn_orchard_geometry")),
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot_spawn,
                    on_exit=[
                        TimerAction(
                            period=1.0,
                            actions=[gzclient_after_robot_spawn],
                        )
                    ],
                )
            ),
            robot_spawn,
            Node(
                package="agribot_autonomy",
                executable="pointcloud_ring_to_laserscan",
                name="c16_horizontal_scan",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_cloud_topic": "/points",
                        "output_scan_topic": "/scan",
                        "ring_index": 8,
                        "beam_count": 720,
                        "range_min": 0.3,
                        "range_max": 25.0,
                        "scan_time": 0.1,
                    }
                ],
            ),
            Node(
                package="agribot_ackermann_rpp",
                executable="sim_navsat_bridge.py",
                name="sim_navsat_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": "/odom",
                        "ground_truth_topic": "/base_pose_ground_truth",
                        "fix_topic": "/navsat/fix",
                        "imu_topic": "/imu/data",
                        "imu_corrected_topic": "/imu/data_corrected",
                        "publish_imu": False,
                        "reference_lat": LaunchConfiguration("navsat_reference_lat"),
                        "reference_lon": LaunchConfiguration("navsat_reference_lon"),
                        "reference_alt": LaunchConfiguration("navsat_reference_alt"),
                        "origin_x": LaunchConfiguration("initial_pose_x"),
                        "origin_y": LaunchConfiguration("initial_pose_y"),
                        "origin_z": LaunchConfiguration("initial_pose_z"),
                        "origin_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
            ),
            Node(
                package="agribot_autonomy",
                executable="imu_frame_bridge.py",
                name="imu_frame_bridge",
                output="screen",
                condition=navsat_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="navsat_to_local_odom.py",
                name="ackermann_navsat_to_local_odom",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "fix_topic": "/navsat/fix",
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "yaw_source_topic": "/base_pose_ground_truth",
                        "yaw_source_message_type": "odometry",
                        "invert_gazebo_axes": True,
                        "zero_altitude": False,
                        "publish_pose_and_tf": False,
                        "yaw_variance": 0.02,
                        "origin_x": LaunchConfiguration("initial_pose_x"),
                        "origin_y": LaunchConfiguration("initial_pose_y"),
                        "origin_z": LaunchConfiguration("initial_pose_z"),
                        "origin_yaw": LaunchConfiguration("initial_pose_yaw"),
                    }
                ],
                condition=navsat_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="rtk_eskf_localization",
                name="ackermann_rtk_eskf_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("navsat_ekf_params_file"),
                    {
                        "use_sim_time": True,
                        "imu_topic": ParameterValue(LaunchConfiguration("navsat_imu_topic"), value_type=str),
                        "pose_topic": ParameterValue(LaunchConfiguration("navsat_pose_topic"), value_type=str),
                        "pose_message_type": ParameterValue(
                            LaunchConfiguration("navsat_pose_message_type"),
                            value_type=str,
                        ),
                        "reference_lat_deg": ParameterValue(
                            LaunchConfiguration("navsat_reference_lat"),
                            value_type=float,
                        ),
                        "reference_lon_deg": ParameterValue(
                            LaunchConfiguration("navsat_reference_lon"),
                            value_type=float,
                        ),
                        "reference_alt_m": ParameterValue(
                            LaunchConfiguration("navsat_reference_alt"),
                            value_type=float,
                        ),
                        "auto_reference_from_first_noah_gnss": ParameterValue(
                            LaunchConfiguration("navsat_auto_reference_from_first_noah_gnss"),
                            value_type=bool,
                        ),
                        "initial_pose_x": ParameterValue(LaunchConfiguration("initial_pose_x"), value_type=float),
                        "initial_pose_y": ParameterValue(LaunchConfiguration("initial_pose_y"), value_type=float),
                        "initial_pose_z": ParameterValue(LaunchConfiguration("initial_pose_z"), value_type=float),
                        "initial_pose_yaw": ParameterValue(LaunchConfiguration("initial_pose_yaw"), value_type=float),
                    },
                ],
                condition=navsat_condition,
            ),
            Node(
                package="agribot_rl_nav",
                executable="navsat_pose_bridge.py",
                name="ackermann_navsat_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": "/odometry/filtered_navsat",
                        "pose_topic": "/amcl_pose",
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "tf_mode": "odom_to_base",
                    }
                ],
                condition=navsat_condition,
            ),
            TimerAction(
                period=LaunchConfiguration("fastlio_start_delay"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(autonomy_share, "launch", "fast_lio_sim.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            "is_simulation": "true",
                            "fastlio_config_file": LaunchConfiguration("fastlio_config_file"),
                            "fastlio_visualize": LaunchConfiguration("fastlio_visualize"),
                            "fastlio_output_odom_topic": LaunchConfiguration("nav_odom_topic"),
                            "fastlio_output_odom_frame": "odom",
                            "fastlio_output_base_frame": "base_link",
                            "fastlio_stamp_with_current_time": "true",
                            "fastlio_publish_tf": "true",
                        }.items(),
                    )
                ],
                condition=fastlio_condition,
            ),
            TimerAction(
                period=LaunchConfiguration("fastlio_localization_start_delay"),
                actions=[
                    Node(
                        package="agribot_autonomy",
                        executable="kiss_localization.py",
                        name="ackermann_fastlio_localization",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "map_frame": "map",
                                "odom_frame": "odom",
                                "base_frame": "base_link",
                                "odom_topic": LaunchConfiguration("nav_odom_topic"),
                                "initial_pose_topic": "/initialpose",
                                "pose_topic": "/amcl_pose",
                                "planar_mode": False,
                                "allow_reinitialization": True,
                                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                                "initial_pose_z": LaunchConfiguration("initial_pose_z"),
                                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                                "stamp_with_current_time": True,
                            }
                        ],
                    ),
                    Node(
                        package="agribot_autonomy",
                        executable="initial_pose_sender.py",
                        name="ackermann_initial_pose_sender_fastlio",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "x": LaunchConfiguration("initial_pose_x"),
                                "y": LaunchConfiguration("initial_pose_y"),
                                "z": LaunchConfiguration("initial_pose_z"),
                                "yaw": LaunchConfiguration("initial_pose_yaw"),
                                "frame_id": "map",
                                "topic": "/initialpose",
                                "startup_delay": 0.5,
                                "publish_count": 10,
                                "publish_interval": 0.5,
                                "covariance_xy": 0.05,
                                "covariance_yaw": 0.02,
                            }
                        ],
                    ),
                ],
                condition=fastlio_condition,
            ),
            Node(
                package="agribot_autonomy",
                executable="initial_pose_sender.py",
                name="ackermann_initial_pose_sender_navsat",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "z": LaunchConfiguration("initial_pose_z"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "frame_id": "map",
                        "topic": "/initialpose",
                        "startup_delay": 6.0,
                        "publish_count": 10,
                        "publish_interval": 0.5,
                        "covariance_xy": 0.05,
                        "covariance_yaw": 0.02,
                    }
                ],
                condition=navsat_condition,
            ),
            OpaqueFunction(function=_map_server_groups),
            Node(
                package="agribot_autonomy",
                executable="snake_waypoint_runner.py",
                name="snake_waypoint_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "waypoint_file": LaunchConfiguration("waypoint_file"),
                        "navigation_mode": "follow_path",
                        "startup_delay": 24.0,
                        "action_name": "navigate_to_pose",
                        "path_action_name": "follow_path",
                        "controller_id": "FollowPath",
                        "path_step": 0.5,
                        "goal_topic": "/current_goal",
                        "frame_id": "map",
                        "stop_on_failure": False,
                        "retries_per_waypoint": 2,
                        "transition_delay": 2.0,
                        "waypoint_transform_enabled": LaunchConfiguration(
                            "waypoint_transform_enabled"
                        ),
                        "waypoint_source_origin_x": LaunchConfiguration("initial_pose_x"),
                        "waypoint_source_origin_y": LaunchConfiguration("initial_pose_y"),
                        "waypoint_source_origin_yaw": LaunchConfiguration("initial_pose_yaw"),
                        "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                        "require_pose_before_start": True,
                    }
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("rviz_start_delay"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(scout_viz_share, "launch", "view_robot.launch.py")
                        ),
                        launch_arguments={
                            "rviz_config": LaunchConfiguration("rviz_config"),
                        }.items(),
                        condition=IfCondition(LaunchConfiguration("rviz")),
                    ),
                ],
            ),
        ]
    )
