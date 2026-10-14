"""Offline, uncalibrated physical-parameter drafts for Unity scene vehicles."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

import yaml

from .generator import (
    generate_fastlivo_config,
    generate_kinematics,
    generate_nav2_geometry,
    generate_sensor_mounts,
)
from .model import ConfigError, load_and_validate


def export_physical_draft(source: Path, output: Path) -> Path:
    config, _ = load_and_validate(source)
    vehicle_id = config["vehicleId"]
    if not re.fullmatch(r"unity_car[1-5]", vehicle_id):
        raise ConfigError("未验证配置草案仅支持 unity_car1 至 unity_car5")

    output = output.absolute()
    if output.name != f"{vehicle_id}_physical_draft":
        raise ConfigError("草案目录名称必须与车辆ID一致，且不能覆盖已验证配置")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix=".physical-draft-", dir=output.parent) as temporary:
        staged = Path(temporary) / output.name
        staged.mkdir()
        (staged / "vehicle_config.json").write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        parameters = {
            "sensor_mounts.yaml": generate_sensor_mounts(config),
            "nav2_geometry.yaml": generate_nav2_geometry(config),
            "vehicle_kinematics.yaml": generate_kinematics(config),
            "fastlivo_extrinsics.yaml": generate_fastlivo_config(config),
        }
        for filename, value in parameters.items():
            (staged / filename).write_text(
                yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8"
            )
        (staged / "README.md").write_text(
            f"# {vehicle_id} 离线真机配置草案\n\n"
            "由当前 Unity 车辆配置生成，仅供离线检查和后续标定。\n"
            "传感器外参、车体几何、速度限制及底盘协议均未经该车型真机验证。\n"
            "不包含底盘协议、固件或运动授权；不能直接导入当前农机控制台或用于控车。\n",
            encoding="utf-8",
        )
        files = {
            item.name: hashlib.sha256(item.read_bytes()).hexdigest()
            for item in sorted(staged.iterdir()) if item.is_file()
        }
        (staged / "manifest.json").write_text(
            json.dumps({
                "format": "agribot.physical-config-draft",
                "version": 1,
                "vehicleId": vehicle_id,
                "vehicleType": config["vehicleType"],
                "coordinateConvention": "ROS_FLU",
                "calibrationVerified": False,
                "deployable": False,
                "sourceSha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "files": files,
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if output.exists():
            if not output.is_dir() or output.is_symlink():
                raise ConfigError("草案输出路径不是普通目录，拒绝覆盖")
            previous = Path(temporary) / "previous"
            os.replace(output, previous)
        else:
            previous = None
        try:
            os.replace(staged, output)
        except OSError:
            if previous is not None:
                os.replace(previous, output)
            raise
    return output
