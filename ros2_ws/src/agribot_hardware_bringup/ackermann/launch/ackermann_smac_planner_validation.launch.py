from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def validate_inputs(context):
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser()
    if map_path.suffix not in (".yaml", ".yml"):
        raise RuntimeError("map must be a Nav2 .yaml file")
    if not map_path.is_file():
        raise RuntimeError(f"map file does not exist: {map_path}")

    if LaunchConfiguration("show_3d_map").perform(context).lower() in (
        "1", "true", "yes", "on"
    ):
        pcd_path = Path(LaunchConfiguration("pcd_map").perform(context)).expanduser()
        if pcd_path.suffix.lower() != ".pcd":
            raise RuntimeError("pcd_map must be a .pcd file when show_3d_map is true")
        if not pcd_path.is_file():
            raise RuntimeError(f"3D map file does not exist: {pcd_path}")

    route_plan = LaunchConfiguration("route_plan").perform(context).strip()
    if route_plan:
        route_path = Path(route_plan).expanduser()
        if route_path.suffix.lower() != ".json":
            raise RuntimeError("route_plan must be a semantic route .json file")
        if not route_path.is_file():
            raise RuntimeError(f"semantic route file does not exist: {route_path}")
    path_output = LaunchConfiguration("path_output").perform(context).strip()
    if path_output and Path(path_output).expanduser().suffix.lower() != ".json":
        raise RuntimeError("path_output must use the .json suffix")
    return []


def generate_launch_description():
    package_share = get_package_share_directory("agribot_hardware_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", description="Absolute path to the Nav2 map YAML"
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value="",
                description="Absolute path to the matching 3D PCD map",
            ),
            DeclareLaunchArgument("show_3d_map", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "route_plan",
                default_value="",
                description=(
                    "Optional preview-only semantic route JSON; its requested "
                    "stops are planned automatically with Smac"
                ),
            ),
            DeclareLaunchArgument(
                "route_waypoint_mode",
                default_value="all_dijkstra",
                description=(
                    "all_dijkstra for normal validation; requested_stops is "
                    "reserved for exporting a Smac topology reference path"
                ),
            ),
            DeclareLaunchArgument(
                "path_output",
                default_value="",
                description="Optional planner-certified path JSON output",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    Path(package_share)
                    / "ackermann"
                    / "config"
                    / "nav2_params_ackermann_planner_only.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    Path(package_share) / "rviz" / "planner_validation.rviz"
                ),
            ),
            OpaqueFunction(function=validate_inputs),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    configured_params,
                    {"yaml_filename": LaunchConfiguration("map")},
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="agribot_hardware_bringup",
                executable="planner_validation_bridge.py",
                name="planner_validation_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "route_plan": LaunchConfiguration("route_plan"),
                        "route_waypoint_mode": LaunchConfiguration(
                            "route_waypoint_mode"
                        ),
                        "path_output_file": LaunchConfiguration("path_output"),
                        "map_yaml": LaunchConfiguration("map"),
                        "planner_params_file": LaunchConfiguration("params_file"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="planner_validation_lifecycle_manager",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["map_server", "planner_server"]},
                ],
            ),
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                name="planner_validation_pcd_publisher",
                output="screen",
                parameters=[{
                    "file_name": LaunchConfiguration("pcd_map"),
                    "tf_frame": "map",
                    "publishing_period_ms": 10000,
                }],
                remappings=[("cloud_pcd", "/planning_test/map_3d")],
                condition=IfCondition(LaunchConfiguration("show_3d_map")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="planner_validation_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
