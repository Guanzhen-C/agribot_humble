from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _validate_inputs(context):
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser()
    if map_path.suffix not in (".yaml", ".yml") or not map_path.is_file():
        raise RuntimeError(f"Nav2地图YAML不存在: {map_path}")

    lattice_path = Path(
        LaunchConfiguration("lattice_filepath").perform(context)
    ).expanduser()
    if lattice_path.suffix != ".json" or not lattice_path.is_file():
        raise RuntimeError(f"差速运动基元JSON不存在: {lattice_path}")

    if LaunchConfiguration("show_3d_map").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    ):
        pcd_path = Path(LaunchConfiguration("pcd_map").perform(context)).expanduser()
        if pcd_path.suffix.lower() != ".pcd" or not pcd_path.is_file():
            raise RuntimeError(f"三维PCD地图不存在: {pcd_path}")
    return []


def generate_launch_description():
    package_share = get_package_share_directory("agribot_hardware_bringup")
    differential_share = Path(package_share) / "differential"
    use_sim_time = LaunchConfiguration("use_sim_time")
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "lattice_filepath": LaunchConfiguration("lattice_filepath"),
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Nav2地图YAML绝对路径"),
            DeclareLaunchArgument("pcd_map", default_value=""),
            DeclareLaunchArgument("show_3d_map", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    differential_share
                    / "config"
                    / "nav2_params_differential_planner_only.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "lattice_filepath",
                default_value=str(
                    differential_share
                    / "config"
                    / "motion_primitives"
                    / "diff_5cm.json"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    Path(package_share) / "rviz" / "planner_validation.rviz"
                ),
            ),
            OpaqueFunction(function=_validate_inputs),
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
                name="differential_planner_validation_lifecycle_manager",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": LaunchConfiguration("autostart")},
                    {"node_names": ["map_server", "planner_server"]},
                ],
            ),
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                name="differential_planner_validation_pcd_publisher",
                output="screen",
                parameters=[
                    {
                        "file_name": LaunchConfiguration("pcd_map"),
                        "tf_frame": "map",
                        "publishing_period_ms": 10000,
                    }
                ],
                remappings=[("cloud_pcd", "/planning_test/map_3d")],
                condition=IfCondition(LaunchConfiguration("show_3d_map")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="differential_planner_validation_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
