"""Validated, declarative ROS launch profiles for the mobile gateway."""

from __future__ import annotations

from pathlib import Path

import yaml

from .catalog import CatalogError, validated_identifier


class ProfileError(RuntimeError):
    """Raised when runtime profile configuration is unsafe or malformed."""


class RuntimeProfiles:
    def __init__(self, path: Path):
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise ProfileError(f"无法读取运行配置 {path}: {error}") from error
        source = document.get("profiles") if isinstance(document, dict) else None
        if not isinstance(source, dict) or not source:
            raise ProfileError("运行配置必须包含非空profiles映射")
        self._profiles = {}
        for profile_id, profile in source.items():
            try:
                validated_identifier(profile_id, "运行配置名称")
            except CatalogError as error:
                raise ProfileError(str(error)) from error
            self._profiles[profile_id] = self._validated(profile_id, profile)

    @staticmethod
    def _validated(profile_id: str, profile: object) -> dict:
        if not isinstance(profile, dict):
            raise ProfileError(f"运行配置必须是映射: {profile_id}")
        required = ("label", "launch_package", "launch_file", "map_argument")
        if any(not isinstance(profile.get(key), str) or not profile[key] for key in required):
            raise ProfileError(f"运行配置缺少必要字段: {profile_id}")
        fixed_args = profile.get("fixed_args", {})
        motion_args = profile.get("motion_args", {})
        required_suffixes = profile.get("required_suffixes", [".yaml", ".pcd"])
        if not isinstance(fixed_args, dict) or not isinstance(motion_args, dict):
            raise ProfileError(f"运行配置参数必须是映射: {profile_id}")
        if not isinstance(required_suffixes, list) or not all(
            isinstance(value, str)
            and value
            and not Path(value).is_absolute()
            and "/" not in value
            for value in required_suffixes
        ):
            raise ProfileError(f"required_suffixes无效: {profile_id}")
        return {
            "id": profile_id,
            "label": profile["label"],
            "description": str(profile.get("description", "")),
            "launch_package": profile["launch_package"],
            "launch_file": profile["launch_file"],
            "map_argument": profile["map_argument"],
            "fixed_args": {str(key): str(value) for key, value in fixed_args.items()},
            "motion_args": {str(key): str(value) for key, value in motion_args.items()},
            "required_suffixes": required_suffixes,
            "requires_georeference": bool(profile.get("requires_georeference", False)),
        }

    def public(self) -> list[dict]:
        return [
            {
                "id": profile["id"],
                "label": profile["label"],
                "description": profile["description"],
                "requires_georeference": profile["requires_georeference"],
            }
            for profile in self._profiles.values()
        ]

    def get(self, profile_id: str) -> dict:
        try:
            return self._profiles[profile_id]
        except KeyError as error:
            raise ProfileError(f"未知运行配置: {profile_id}") from error

    def command(self, profile_id: str, map_base: Path, motion: bool) -> list[str]:
        profile = self.get(profile_id)
        for suffix in profile["required_suffixes"]:
            path = Path(f"{map_base}{suffix}")
            if not path.is_file():
                raise ProfileError(f"运行地图缺少文件: {path}")
        if profile["requires_georeference"]:
            georeference = Path(f"{map_base}_georeference.yaml")
            if not georeference.is_file():
                raise ProfileError(f"运行地图缺少地理配准: {georeference}")
        arguments = dict(profile["fixed_args"])
        arguments[profile["map_argument"]] = str(map_base)
        arguments["enable_chassis_output"] = "true" if motion else "false"
        if motion:
            arguments.update(profile["motion_args"])
        return [
            "ros2",
            "launch",
            profile["launch_package"],
            profile["launch_file"],
            *(f"{key}:={value}" for key, value in arguments.items()),
        ]
