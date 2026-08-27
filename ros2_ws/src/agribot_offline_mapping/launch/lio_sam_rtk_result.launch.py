from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
    fastlio_bag_value = LaunchConfiguration("fastlio_bag").perform(context)
    fastlivo_bag_value = LaunchConfiguration("fastlivo_bag").perform(context)
    kf_gins_bag_value = LaunchConfiguration("kf_gins_bag").perform(context)
    fastlivo_rtk_bag_value = LaunchConfiguration(
        "fastlivo_rtk_bag"
    ).perform(context)

    estimator_values = (
        fastlio_bag_value, fastlivo_bag_value, kf_gins_bag_value
    )
    needs_legacy_comparison = comparison_bag_value or not all(estimator_values)
    comparison_bag = (
        Path(comparison_bag_value).expanduser()
        if comparison_bag_value
        else (
            map_base.parent / f"{map_base.name}_comparison"
            if needs_legacy_comparison
            else None
        )
    )

    def estimator_bag(value):
        return Path(value).expanduser() if value else comparison_bag

    fastlio_bag = estimator_bag(fastlio_bag_value)
    fastlivo_bag = estimator_bag(fastlivo_bag_value)
    kf_gins_bag = estimator_bag(kf_gins_bag_value)
    fastlivo_rtk_bag = (
        Path(fastlivo_rtk_bag_value).expanduser()
        if fastlivo_rtk_bag_value
        else None
    )
    show_comparison_paths = LaunchConfiguration(
        "show_comparison_paths"
    ).perform(context).lower() in ("1", "true", "yes", "on")
    required = [
        paths["pcd"], paths["yaml"], paths["georeference"], paths["manifest"],
        paths["result_bag"] / "metadata.yaml",
    ]
    if show_comparison_paths:
        for estimator_name, estimator_path in (
            ("FAST-LIO2", fastlio_bag),
            ("FAST-LIVO2", fastlivo_bag),
            ("KF-GINS", kf_gins_bag),
        ):
            if estimator_path is None:
                raise RuntimeError(f"{estimator_name} result bag was not provided")
            required.append(estimator_path / "metadata.yaml")
        if fastlivo_rtk_bag is not None:
            required.append(fastlivo_rtk_bag / "metadata.yaml")
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
    source_bag = Path(manifest.get("source_bag", "")).expanduser()
    if show_comparison_paths and not (source_bag / "metadata.yaml").is_file():
        raise RuntimeError(f"manifest source_bag is unavailable: {source_bag}")
    artifacts = manifest.get("artifacts", {})
    for key in ("pcd", "yaml", "georeference"):
        if Path(artifacts.get(key, "")) != paths[key]:
            raise RuntimeError(f"manifest {key} does not match the requested map")
    mounts_override = LaunchConfiguration("sensor_mounts_file").perform(context)
    if mounts_override:
        sensor_mounts_file = Path(mounts_override).expanduser()
    else:
        hardware_share = Path(
            get_package_share_directory("agribot_hardware_bringup")
        )
        vehicle_profile = manifest.get("vehicle_profile", "ackermann")
        if vehicle_profile == "differential":
            sensor_mounts_file = (
                hardware_share
                / "differential"
                / "config"
                / "sensor_mounts.yaml"
            )
        elif vehicle_profile == "ackermann":
            sensor_mounts_file = hardware_share / "config" / "sensor_mounts.yaml"
        else:
            raise RuntimeError(
                f"unsupported vehicle profile in manifest: {vehicle_profile}"
            )
    if not sensor_mounts_file.is_file():
        raise RuntimeError(f"sensor mounts file not found: {sensor_mounts_file}")
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
                    str(comparison_bag)
                    if show_comparison_paths and comparison_bag is not None
                    else ""
                ),
                "source_bag": str(source_bag) if show_comparison_paths else "",
                "fastlio_bag": (
                    str(fastlio_bag)
                    if show_comparison_paths and fastlio_bag is not None
                    else ""
                ),
                "fastlivo_bag": (
                    str(fastlivo_bag)
                    if show_comparison_paths and fastlivo_bag is not None
                    else ""
                ),
                "fastlivo_topic": LaunchConfiguration("fastlivo_topic"),
                "fastlivo_rtk_bag": (
                    str(fastlivo_rtk_bag)
                    if show_comparison_paths and fastlivo_rtk_bag is not None
                    else ""
                ),
                "kf_gins_bag": (
                    str(kf_gins_bag)
                    if show_comparison_paths and kf_gins_bag is not None
                    else ""
                ),
                "fastlivo_rtk_topic": LaunchConfiguration(
                    "fastlivo_rtk_topic"
                ),
                "sensor_mounts_file": str(sensor_mounts_file),
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
                "Legacy combined FAST-LIO2/FAST-LIVO2/KF-GINS bag; used as "
                "fallback for estimator bags that are not specified"
            ),
        ),
        DeclareLaunchArgument(
            "fastlio_bag",
            default_value="",
            description="Optional independently recomputed FAST-LIO2 bag",
        ),
        DeclareLaunchArgument(
            "fastlivo_bag",
            default_value="",
            description=(
                "Optional independently recomputed FAST-LIVO2 bag; when set, "
                "it replaces only the FAST-LIVO2 path from comparison_bag"
            ),
        ),
        DeclareLaunchArgument(
            "fastlivo_topic",
            default_value="/comparison/fastlivo/odometry",
            description="Odometry topic stored in fastlivo_bag",
        ),
        DeclareLaunchArgument(
            "kf_gins_bag",
            default_value="",
            description="Optional independently recomputed KF-GINS bag",
        ),
        DeclareLaunchArgument(
            "fastlivo_rtk_bag",
            default_value="",
            description="Optional independently recomputed FAST-LIVO2+RTK bag",
        ),
        DeclareLaunchArgument(
            "fastlivo_rtk_topic",
            default_value="/fastlivo_rtk/odometry",
            description="Fused base_link odometry topic stored in fastlivo_rtk_bag",
        ),
        DeclareLaunchArgument("flatten_z", default_value="true"),
        DeclareLaunchArgument(
            "sensor_mounts_file",
            default_value="",
            description=(
                "Optional sensor-mount override; otherwise selected from the "
                "map manifest vehicle_profile"
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(package_share / "rviz" / "lio_sam_rtk_result.rviz"),
        ),
        OpaqueFunction(function=launch_setup),
    ])
