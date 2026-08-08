from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def validate_map(context):
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser()
    if map_path.suffix not in (".yaml", ".yml"):
        raise RuntimeError("map must be a Nav2 .yaml file")
    if not map_path.is_file():
        raise RuntimeError(f"map file does not exist: {map_path}")
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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
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
            OpaqueFunction(function=validate_map),
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
                parameters=[{"use_sim_time": use_sim_time}],
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
                package="rviz2",
                executable="rviz2",
                name="planner_validation_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
