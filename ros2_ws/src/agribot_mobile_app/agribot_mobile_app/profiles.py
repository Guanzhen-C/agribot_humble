"""Validated, declarative ROS launch profiles for the mobile gateway."""

from __future__ import annotations

from pathlib import Path

import yaml

from .catalog import CatalogError, validated_identifier


class ProfileError(RuntimeError):
    """Raised when runtime profile configuration is unsafe or malformed."""


def launch_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class RuntimeProfiles:
    def __init__(self, path: Path):
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise ProfileError(f"无法读取运行配置 {path}: {error}") from error
        vehicle_source = document.get("vehicles") if isinstance(document, dict) else None
        if not isinstance(vehicle_source, dict) or not vehicle_source:
            raise ProfileError("运行配置必须包含非空vehicles映射")
        self._vehicles = {}
        for vehicle_id, vehicle in vehicle_source.items():
            try:
                validated_identifier(vehicle_id, "车型名称")
            except CatalogError as error:
                raise ProfileError(str(error)) from error
            self._vehicles[vehicle_id] = self._validated_vehicle(vehicle_id, vehicle)

        source = document.get("profiles") if isinstance(document, dict) else None
        if not isinstance(source, dict) or not source:
            raise ProfileError("运行配置必须包含非空profiles映射")
        self._profiles = {}
        for profile_id, profile in source.items():
            try:
                validated_identifier(profile_id, "运行配置名称")
            except CatalogError as error:
                raise ProfileError(str(error)) from error
            validated = self._validated(profile_id, profile)
            if validated["vehicle_type"] not in self._vehicles:
                raise ProfileError(
                    f"运行配置引用未知车型: {profile_id}: "
                    f"{validated['vehicle_type']}"
                )
            self._profiles[profile_id] = validated

    @staticmethod
    def _validated_vehicle(vehicle_id: str, vehicle: object) -> dict:
        if not isinstance(vehicle, dict):
            raise ProfileError(f"车型配置必须是映射: {vehicle_id}")
        collection = vehicle.get("collection")
        footprint = vehicle.get("footprint")
        if not isinstance(vehicle.get("label"), str) or not vehicle["label"]:
            raise ProfileError(f"车型配置缺少label: {vehicle_id}")
        if not isinstance(collection, dict):
            raise ProfileError(f"车型配置缺少collection: {vehicle_id}")
        required_collection = (
            "launch_package",
            "launch_file",
            "output_argument",
        )
        if any(
            not isinstance(collection.get(key), str) or not collection[key]
            for key in required_collection
        ):
            raise ProfileError(f"车型采集配置缺少必要字段: {vehicle_id}")
        fixed_args = collection.get("fixed_args", {})
        if not isinstance(fixed_args, dict):
            raise ProfileError(f"车型采集参数必须是映射: {vehicle_id}")
        if (
            not isinstance(footprint, list)
            or len(footprint) < 3
            or not all(
                isinstance(point, list)
                and len(point) == 2
                and all(isinstance(value, (int, float)) for value in point)
                for point in footprint
            )
        ):
            raise ProfileError(f"车型footprint无效: {vehicle_id}")
        return {
            "id": vehicle_id,
            "label": vehicle["label"],
            "description": str(vehicle.get("description", "")),
            "footprint": [
                [float(point[0]), float(point[1])] for point in footprint
            ],
            "collection": {
                "launch_package": collection["launch_package"],
                "launch_file": collection["launch_file"],
                "output_argument": collection["output_argument"],
                "fixed_args": {
                    str(key): launch_value(value) for key, value in fixed_args.items()
                },
            },
        }

    @staticmethod
    def _validated(profile_id: str, profile: object) -> dict:
        if not isinstance(profile, dict):
            raise ProfileError(f"运行配置必须是映射: {profile_id}")
        required = (
            "label",
            "vehicle_type",
            "launch_package",
            "launch_file",
            "map_argument",
        )
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
            "vehicle_type": profile["vehicle_type"],
            "description": str(profile.get("description", "")),
            "launch_package": profile["launch_package"],
            "launch_file": profile["launch_file"],
            "map_argument": profile["map_argument"],
            "fixed_args": {
                str(key): launch_value(value) for key, value in fixed_args.items()
            },
            "motion_args": {
                str(key): launch_value(value) for key, value in motion_args.items()
            },
            "required_suffixes": required_suffixes,
            "requires_georeference": bool(profile.get("requires_georeference", False)),
        }

    def public(self) -> list[dict]:
        return [
            {
                "id": profile["id"],
                "label": profile["label"],
                "vehicle_type": profile["vehicle_type"],
                "description": profile["description"],
                "requires_georeference": profile["requires_georeference"],
            }
            for profile in self._profiles.values()
        ]

    def public_vehicles(self) -> list[dict]:
        return [
            {
                "id": vehicle["id"],
                "label": vehicle["label"],
                "description": vehicle["description"],
                "footprint": vehicle["footprint"],
            }
            for vehicle in self._vehicles.values()
        ]

    def vehicle(self, vehicle_id: str) -> dict:
        try:
            return self._vehicles[vehicle_id]
        except KeyError as error:
            raise ProfileError(f"未知车型: {vehicle_id}") from error

    def get(self, profile_id: str) -> dict:
        try:
            return self._profiles[profile_id]
        except KeyError as error:
            raise ProfileError(f"未知运行配置: {profile_id}") from error

    def command(
        self,
        profile_id: str,
        map_base: Path,
        motion: bool,
        vehicle_type: str | None = None,
    ) -> list[str]:
        profile = self.get(profile_id)
        if vehicle_type is not None and profile["vehicle_type"] != vehicle_type:
            raise ProfileError(
                f"运行配置{profile_id}不属于所选车型{vehicle_type}"
            )
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

    def collection_command(self, vehicle_id: str, bag_path: Path) -> list[str]:
        vehicle = self.vehicle(vehicle_id)
        collection = vehicle["collection"]
        arguments = dict(collection["fixed_args"])
        arguments[collection["output_argument"]] = str(bag_path)
        return [
            "ros2",
            "launch",
            collection["launch_package"],
            collection["launch_file"],
            *(f"{key}:={value}" for key, value in arguments.items()),
        ]
