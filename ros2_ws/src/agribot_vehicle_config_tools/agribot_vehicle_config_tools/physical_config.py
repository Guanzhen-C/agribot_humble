"""Portable handoff of the verified Ackermann stack, never simulation tuning."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from pathlib import Path

FORMAT = "agribot.physical-runtime"
VERSION = 1
PRESET = "ackermann_fastlivo_rtk_smac_mppi_v1"
MAX_BYTES = 2 * 1024 * 1024
ALGORITHMS = {
    "perception": "stvl",
    "localization": "fastlivo_rtk",
    "initial_localization": "ndt_gicp",
    "planner": "smac_hybrid_dubins",
    "controller": "mppi_ackermann",
}
# Keys are launch arguments, not paths supplied by the importing client.
TEMPLATES = {
    "nav2_params": ("agribot_hardware_bringup", "ackermann/config/nav2_params_ackermann_fastlio_mapped.yaml"),
    "mount_config": ("agribot_hardware_bringup", "config/sensor_mounts.yaml"),
    "fastlivo_lidar_config": ("fast_livo", "config/agribot_c16_astra.yaml"),
    "fastlivo_camera_config": ("agribot_hardware_bringup", "config/fastlivo_hikrobot_mv_cu013.yaml"),
    "fastlivo_bridge_config": ("agribot_hardware_bringup", "config/fastlivo_bridge.yaml"),
    "pcd_initial_localization_config": ("agribot_hardware_bringup", "ackermann/config/pcd_initial_localization.yaml"),
    "rtk_map_initializer_config": ("agribot_hardware_bringup", "config/rtk_map_initializer.yaml"),
    "fastlivo_rtk_fusion_config": ("agribot_hardware_bringup", "config/fastlivo_rtk_fusion.yaml"),
    "chassis_can_config": ("agribot_hardware_bringup", "ackermann/config/chassis_can.yaml"),
    "chassis_serial_config": ("agribot_hardware_bringup", "ackermann/config/chassis_serial.yaml"),
}


class PhysicalConfigError(ValueError):
    pass


def template_paths(workspace_src: Path | None = None) -> dict[str, Path]:
    if workspace_src is not None:
        roots = {
            "agribot_hardware_bringup": workspace_src / "agribot_hardware_bringup",
            "fast_livo": workspace_src / "FAST-LIVO2",
        }
    else:
        from ament_index_python.packages import get_package_share_directory

        roots = {
            package: Path(get_package_share_directory(package))
            for package, _ in TEMPLATES.values()
        }
    return {key: roots[package] / relative for key, (package, relative) in TEMPLATES.items()}


def digest(document: dict) -> str:
    return hashlib.sha256(
        json.dumps(document, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def export_document(paths: dict[str, Path]) -> dict:
    try:
        files = {key: paths[key].read_text(encoding="utf-8") for key in TEMPLATES}
    except (KeyError, OSError, UnicodeError) as error:
        raise PhysicalConfigError(f"无法读取已验证真机参数: {error}") from error
    payload = {
        "format": FORMAT,
        "version": VERSION,
        "preset": PRESET,
        "vehicle_type": "ackermann",
        "algorithms": dict(ALGORITHMS),
        "parameters": files,
    }
    return {**payload, "sha256": digest(payload)}


def validate_document(document: object, baseline: dict) -> dict:
    if not isinstance(document, dict) or set(document) != set(baseline):
        raise PhysicalConfigError("配置格式不正确，请选择导出的真机配置JSON文件")
    if len(json.dumps(document, ensure_ascii=False).encode("utf-8")) > MAX_BYTES:
        raise PhysicalConfigError("真机配置文件过大")
    # This release deliberately accepts only the installed verified preset.
    # A checksum alone would allow arbitrary launch plugins and unsafe tuning.
    for key in ("format", "version", "preset", "vehicle_type", "algorithms", "parameters"):
        if document.get(key) != baseline.get(key):
            raise PhysicalConfigError(f"{key}与本机已验证方案不一致，请先同步软件版本；原配置未改变")
    payload = {key: value for key, value in document.items() if key != "sha256"}
    if document.get("sha256") != digest(payload):
        raise PhysicalConfigError("配置校验失败，文件可能损坏；原配置未改变")
    return document


def write_document(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".physical-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
