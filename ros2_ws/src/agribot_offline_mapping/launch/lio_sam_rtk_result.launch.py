from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def fingerprint_file(path):
    value = 14695981039346656037
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            for byte in chunk:
                value ^= byte
                value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def launch_setup(context):
    map_base = Path(LaunchConfiguration("map_base").perform(context)).expanduser()
    if not map_base.is_absolute() or map_base.suffix:
        raise RuntimeError("map_base must be an absolute path without an extension")
    paths = {
        "pcd": map_base.with_suffix(".pcd"),
        "yaml": map_base.with_suffix(".yaml"),
        "georeference": map_base.parent / f"{map_base.name}_georeference.yaml",
        "manifest": map_base.parent / f"{map_base.name}_manifest.yaml",
        "result_bag": map_base.parent / f"{map_base.name}_result",
    }
    comparison_bag_value = LaunchConfiguration("comparison_bag").perform(context)
    comparison_bag = (
        Path(comparison_bag_value).expanduser()
        if comparison_bag_value
        else map_base.parent / f"{map_base.name}_comparison"
    )
    show_comparison_paths = LaunchConfiguration(
        "show_comparison_paths"
    ).perform(context).lower() in ("1", "true", "yes", "on")
    required = [
        paths["pcd"], paths["yaml"], paths["georeference"], paths["manifest"],
        paths["result_bag"] / "metadata.yaml",
    ]
    if show_comparison_paths:
        required.append(comparison_bag / "metadata.yaml")
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(
            "mapping result is incomplete:\n" + "\n".join(str(path) for path in missing)
        )
    manifest = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    if Path(manifest.get("map_base", "")) != map_base:
        raise RuntimeError("manifest map_base does not match the requested map")
    if Path(manifest.get("result_bag", "")) != paths["result_bag"]:
        raise RuntimeError("manifest result_bag does not match the requested map")
    artifacts = manifest.get("artifacts", {})
    for key in ("pcd", "yaml", "georeference"):
        if Path(artifacts.get(key, "")) != paths[key]:
            raise RuntimeError(f"manifest {key} does not match the requested map")
    georeference = yaml.safe_load(
        paths["georeference"].read_text(encoding="utf-8")
    )
    georeference_map = georeference.get("map", {})
    if Path(georeference_map.get("pcd", "")) != paths["pcd"]:
        raise RuntimeError("georeference PCD does not match the requested map")
    expected_fingerprint = georeference_map.get("fingerprint_fnv1a64", "")
    if fingerprint_file(paths["pcd"]) != expected_fingerprint:
        raise RuntimeError("PCD fingerprint does not match the georeference")

    rviz_config = LaunchConfiguration("rviz_config")
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": str(paths["yaml"]), "frame_id": "map"}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="mapping_result_lifecycle_manager",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["map_server"]}],
        ),
        Node(
            package="agribot_offline_mapping",
            executable="mapping_result_trajectory_publisher.py",
            name="mapping_result_trajectory_publisher",
            output="screen",
            parameters=[{
                "result_bag": str(paths["result_bag"]),
                "georeference_file": str(paths["georeference"]),
                "flatten_z": LaunchConfiguration("flatten_z"),
                "comparison_bag": (
                    str(comparison_bag) if show_comparison_paths else ""
                ),
            }],
        ),
        Node(
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="mapping_result_pcd_publisher",
            output="screen",
            parameters=[{
                "file_name": str(paths["pcd"]),
                "tf_frame": "map",
                "publishing_period_ms": 10000,
            }],
            remappings=[("cloud_pcd", "/pcd_map")],
            condition=IfCondition(LaunchConfiguration("show_3d_map")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mapping_result_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    package_share = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_base", description="Absolute map result path without an extension"
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("show_3d_map", default_value="true"),
        DeclareLaunchArgument("show_comparison_paths", default_value="false"),
        DeclareLaunchArgument(
            "comparison_bag",
            default_value="",
            description=(
                "Recomputed FAST-LIO2/KF-GINS/robot_localization bag; defaults to "
                "MAP_NAME_comparison when comparison paths are enabled"
            ),
        ),
        DeclareLaunchArgument("flatten_z", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(package_share / "rviz" / "lio_sam_rtk_result.rviz"),
        ),
        OpaqueFunction(function=launch_setup),
    ])
