import hashlib
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


DEFAULT_MAP_BASE = "/home/sunrise/agribot_maps/test_site/map_lio_sam_0811"
REQUIRED_SENSOR_DEVICES = (
    (
        "RTK",
        Path(
            "/dev/serial/by-id/"
            "usb-AirM2M_AirM2M_Compo_000000000001-if06"
        ),
    ),
    (
        "IMU",
        Path(
            "/dev/serial/by-id/"
            "usb-1a86_USB_Single_Serial_5C2C082600-if00"
        ),
    ),
)


def _enabled(value):
    return value.lower() in ("true", "1", "yes", "on")


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path, description):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RuntimeError(f"无法读取{description}: {path}: {error}") from error
    if not isinstance(document, dict):
        raise RuntimeError(f"{description}不是YAML映射: {path}")
    return document


def _validate_experiment(context):
    locked_arguments = {
        "initialization_source": "rtk",
        "enable_fpfh": "false",
        "allow_missing_georeference": "false",
    }
    for name, expected in locked_arguments.items():
        actual = LaunchConfiguration(name).perform(context).lower()
        if actual != expected:
            raise RuntimeError(
                f"0811室外入口要求{name}:={expected}，实际为{actual}"
            )

    map_base = Path(
        LaunchConfiguration("map_base").perform(context)
    ).expanduser()
    profile_path = Path(
        LaunchConfiguration("map_profile").perform(context)
    ).expanduser()
    profile = _load_yaml(profile_path, "室外地图配置")

    if profile.get("schema_version") != 1:
        raise RuntimeError("室外地图配置schema_version必须为1")
    if map_base.name != profile.get("map_id"):
        raise RuntimeError(
            f"地图名称必须为{profile.get('map_id')}，实际为{map_base.name}"
        )

    verified_files = {}
    file_specs = profile.get("files")
    if not isinstance(file_specs, dict) or not file_specs:
        raise RuntimeError("室外地图配置缺少files")
    for label, file_spec in file_specs.items():
        if not isinstance(file_spec, dict):
            raise RuntimeError(f"室外地图文件配置无效: {label}")
        suffix = file_spec.get("suffix")
        expected_hash = file_spec.get("sha256")
        if not isinstance(suffix, str) or not isinstance(expected_hash, str):
            raise RuntimeError(f"室外地图文件配置缺少suffix或sha256: {label}")
        path = Path(f"{map_base}{suffix}")
        if not path.is_file():
            raise RuntimeError(f"室外实验缺少地图文件: {path}")
        actual_hash = _sha256(path)
        if actual_hash != expected_hash.lower():
            raise RuntimeError(
                f"室外地图文件版本不匹配: {path}; "
                f"期望SHA256={expected_hash}，实际={actual_hash}"
            )
        verified_files[label] = path

    nav2_yaml = _load_yaml(verified_files["nav2_yaml"], "Nav2地图配置")
    image_value = nav2_yaml.get("image")
    if not isinstance(image_value, str) or not image_value:
        raise RuntimeError("Nav2地图配置缺少image")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = verified_files["nav2_yaml"].parent / image_path
    if image_path.resolve() != verified_files["pgm"].resolve():
        raise RuntimeError(
            "Nav2 YAML引用的PGM与室外地图配置不一致: "
            f"{image_path}"
        )

    georeference = _load_yaml(
        verified_files["georeference"], "地图地理配准"
    )
    if georeference.get("map", {}).get("id") != profile["map_id"]:
        raise RuntimeError("地理配准map.id与室外地图名称不一致")
    calibration = georeference.get("calibration", {})
    horizontal_rmse = calibration.get("horizontal_rmse_m")
    maximum_rmse = profile.get("georeference_policy", {}).get(
        "maximum_horizontal_rmse_m"
    )
    if not isinstance(horizontal_rmse, (int, float)) or not isinstance(
        maximum_rmse, (int, float)
    ):
        raise RuntimeError("地图地理配准缺少水平RMSE验收参数")
    if horizontal_rmse > maximum_rmse:
        raise RuntimeError(
            f"地图地理配准水平RMSE {horizontal_rmse:.3f} m超过"
            f"{maximum_rmse:.3f} m"
        )
    if (
        not calibration.get("yaw_validation_passed", False)
        and not profile.get("georeference_policy", {}).get(
            "allow_unvalidated_yaw_as_coarse_prior", False
        )
    ):
        raise RuntimeError("地图地理配准航向未通过验收")

    for sensor_name, device in REQUIRED_SENSOR_DEVICES:
        if not device.exists():
            raise RuntimeError(f"{sensor_name}串口设备不存在: {device}")
    camera = Path(
        LaunchConfiguration("right_camera_device").perform(context)
    )
    if not camera.exists():
        raise RuntimeError(f"FAST-LIVO2右目相机设备不存在: {camera}")

    enable_chassis = _enabled(
        LaunchConfiguration("enable_chassis_output").perform(context)
    )
    can_transport = LaunchConfiguration("can_transport").perform(context)
    if enable_chassis and can_transport == "zqwl_cdc":
        zqwl_port = Path(LaunchConfiguration("zqwl_port").perform(context))
        if not zqwl_port.exists():
            raise RuntimeError(f"USB-CAN设备不存在: {zqwl_port}")
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    full_navigation_launch = os.path.join(
        hardware_share,
        "launch",
        "ackermann_mppi_fastlivo_rtk_mapped.launch.py",
    )
    profile = os.path.join(
        hardware_share, "config", "outdoor_0811_map_profile.yaml"
    )
    rviz_config = os.path.join(hardware_share, "rviz", "navigation.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_base", default_value=DEFAULT_MAP_BASE),
            DeclareLaunchArgument("map_profile", default_value=profile),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("navigation_delay", default_value="8.0"),
            # Keep delayed actions in nested launch files bound to the safe
            # outdoor values after their scoped include contexts have exited.
            DeclareLaunchArgument("initialization_source", default_value="rtk"),
            DeclareLaunchArgument("enable_fpfh", default_value="false"),
            DeclareLaunchArgument(
                "allow_missing_georeference", default_value="false"
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("enable_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "use_detailed_vehicle_model", default_value="false"
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            DeclareLaunchArgument(
                "enable_chassis_output",
                default_value="false",
                description="阶段A保持false；全部检查通过后阶段B显式改为true",
            ),
            DeclareLaunchArgument("chassis_driver", default_value="ackermann_can"),
            DeclareLaunchArgument("can_transport", default_value="zqwl_cdc"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "zqwl_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00"
                ),
            ),
            DeclareLaunchArgument("zqwl_channel", default_value="0"),
            DeclareLaunchArgument("zqwl_bitrate", default_value="1000000"),
            OpaqueFunction(function=_validate_experiment),
            LogInfo(
                msg=[
                    "室外0811全流程：地图校验通过；底盘输出=",
                    LaunchConfiguration("enable_chassis_output"),
                    "。车辆保持静止，等待RTK粗定位及NDT/GICP精配准完成。",
                ]
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(full_navigation_launch),
                        launch_arguments={
                            "map_base": LaunchConfiguration("map_base"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "autostart": LaunchConfiguration("autostart"),
                            "start_sensors": "true",
                            "start_rtk": "true",
                            "start_camera": "true",
                            "start_fastlivo": "true",
                            "start_navigation": "true",
                            "navigation_delay": LaunchConfiguration(
                                "navigation_delay"
                            ),
                            "rviz": LaunchConfiguration("rviz"),
                            "rviz_config": LaunchConfiguration("rviz_config"),
                            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                            "use_detailed_vehicle_model": LaunchConfiguration(
                                "use_detailed_vehicle_model"
                            ),
                            "initialization_source": LaunchConfiguration(
                                "initialization_source"
                            ),
                            "enable_fpfh": LaunchConfiguration("enable_fpfh"),
                            "allow_missing_georeference": LaunchConfiguration(
                                "allow_missing_georeference"
                            ),
                            "right_camera_device": LaunchConfiguration(
                                "right_camera_device"
                            ),
                            "enable_chassis_output": LaunchConfiguration(
                                "enable_chassis_output"
                            ),
                            "chassis_driver": LaunchConfiguration(
                                "chassis_driver"
                            ),
                            "can_transport": LaunchConfiguration(
                                "can_transport"
                            ),
                            "can_interface": LaunchConfiguration("can_interface"),
                            "zqwl_port": LaunchConfiguration("zqwl_port"),
                            "zqwl_channel": LaunchConfiguration("zqwl_channel"),
                            "zqwl_bitrate": LaunchConfiguration("zqwl_bitrate"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
