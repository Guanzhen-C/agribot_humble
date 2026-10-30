"""Driverless estimators. Only the external switch node publishes navigation TF."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    hardware = Path(get_package_share_directory("agribot_hardware_bringup"))
    diff = hardware / "differential/config"
    base = Path(LaunchConfiguration("map_base").perform(context)).expanduser().resolve()
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    kind = LaunchConfiguration("estimator").perform(context)
    if kind == "navsat":
        return [Node(package="agribot_hardware_bringup", executable="rtk_eskf_localization",
                     name="rtk_eskf_localization", output="screen", parameters=[
                         str(hardware/"config/kf_gins_n300pro.yaml"), str(diff/"kf_gins_n300pro.yaml"),
                         {"use_sim_time": sim, "required_fix_quality": 4,
                          "quality_timeout_sec": .3,
                          "georeference_file": str(base)+"_georeference.yaml",
                          "map_file": str(base)+".pcd",
                          "auto_reference_from_first_navsat_fix": False,
                          # GNSS measurement stamps arrive later than IMU samples.
                          "imu_processing_delay_sec": .25,
                          "output_odom_topic": "/localization/navsat/odometry",
                          "publish_raw_pose": False,
                          "initial_pose_topic": "/localization/navsat/manual_seed_unused"}])]
    if kind != "fastlivo":
        raise RuntimeError("estimator must be navsat or fastlivo")
    status = yaml.safe_load((diff/"hikrobot_camera_calibration_status.yaml").read_text())
    checks = ("lens_installed", "intrinsics_calibrated", "lidar_camera_extrinsics_calibrated",
              "image_time_offset_calibrated")
    if any(status.get(key) is not True for key in checks):
        raise RuntimeError("FAST-LIVO2 camera calibration is incomplete; keep NavSat active")
    return [
        Node(package="fast_livo", executable="fastlivo_mapping", name="fastlivo_mapping",
             output="screen", parameters=[str(diff/"fastlivo_c16_camera.yaml"),
                 str(hardware/"config/fastlivo_hikrobot_mv_cu013.yaml"),
                 {"use_sim_time": sim, "publish.dense_map_en": False, "local_map.map_sliding_en": True}]),
        Node(package="agribot_hardware_bringup", executable="fastlio_odom_bridge.py",
             name="fastlio_odom_bridge", output="screen", parameters=[str(diff/"fastlivo_bridge.yaml"),
                 {"use_sim_time": sim, "publish_tf": False, "twist_from_pose": True}]),
        Node(package="agribot_hardware_bringup", executable="fastlivo_rtk_fusion",
             name="fastlivo_rtk_fusion", output="screen", parameters=[str(diff/"fastlivo_rtk_fusion.yaml"),
                 {"use_sim_time": sim, "georeference_file": str(base)+"_georeference.yaml",
                  "map_file": str(base)+".pcd", "allow_missing_georeference": False,
                  "auto_initialize_from_fixed_rtk": False, "publish_map_to_odom_tf": False,
                  "seed_pose_topic": "/localization/handover_seed",
                  "fused_odom_topic": "/localization/fastlivo/odometry",
                  "ready_topic": "/localization/fastlivo/ready",
                  "fixed_active_topic": "/localization/fastlivo/fixed_active",
                  "fused_pose_topic": "/localization/fastlivo/pose",
                  "fused_path_topic": "/localization/fastlivo/path",
                  "local_path_topic": "/localization/fastlivo/local_path",
                  "rtk_path_topic": "/localization/fastlivo/rtk_path"}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("map_base"), DeclareLaunchArgument("estimator"),
                              DeclareLaunchArgument("use_sim_time", default_value="false"),
                              OpaqueFunction(function=nodes)])
