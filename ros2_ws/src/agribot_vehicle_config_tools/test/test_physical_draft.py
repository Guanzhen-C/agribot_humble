"""Offline exports must be vehicle-specific and never claim physical validation."""

import hashlib
import json
import os
from pathlib import Path

import pytest
import yaml

from agribot_vehicle_config_tools.model import ConfigError
from agribot_vehicle_config_tools.physical_draft import export_physical_draft


SOURCE = Path(__file__).parents[2] / "agribot_vehicle_description" / "config" / "vehicles"


@pytest.mark.parametrize("number", range(1, 6))
def test_each_new_vehicle_exports_unverified_draft(tmp_path, number):
    vehicle_id = f"unity_car{number}"
    source = SOURCE / vehicle_id / "vehicle_config.json"
    output = tmp_path / f"{vehicle_id}_physical_draft"
    export_physical_draft(source, output)
    manifest = json.loads((output / "manifest.json").read_text())
    exported = json.loads((output / "vehicle_config.json").read_text())
    assert manifest["format"] == "agribot.physical-config-draft"
    assert manifest["vehicleId"] == exported["vehicleId"] == vehicle_id
    assert manifest["calibrationVerified"] is False
    assert manifest["deployable"] is False
    assert not (output / "firmware").exists()
    for name, digest in manifest["files"].items():
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == digest
    geometry = yaml.safe_load((output / "nav2_geometry.yaml").read_text())
    assert geometry["/**"]["ros__parameters"]["vehicle_id"] == vehicle_id
    assert geometry["/**"]["ros__parameters"]["motion_model"] == (
        "DiffDrive" if number == 3 else "Ackermann"
    )


def test_current_source_values_refresh_without_touching_other_vehicle(tmp_path):
    one = SOURCE / "unity_car1" / "vehicle_config.json"
    two = SOURCE / "unity_car2" / "vehicle_config.json"
    output_one = tmp_path / "unity_car1_physical_draft"
    output_two = tmp_path / "unity_car2_physical_draft"
    export_physical_draft(one, output_one)
    export_physical_draft(two, output_two)
    unchanged = (output_two / "manifest.json").read_bytes()
    modified = json.loads(one.read_text())
    modified["navigation"]["maxLinearSpeedMps"] = 0.19
    current = tmp_path / "current.json"
    current.write_text(json.dumps(modified))
    export_physical_draft(current, output_one)
    assert json.loads((output_one / "vehicle_config.json").read_text())["navigation"]["maxLinearSpeedMps"] == 0.19
    assert (output_two / "manifest.json").read_bytes() == unchanged


def test_wrong_id_cannot_overwrite_another_draft_or_verified_preset(tmp_path):
    source = SOURCE / "unity_car1" / "vehicle_config.json"
    with pytest.raises(ConfigError, match="目录名称"):
        export_physical_draft(source, tmp_path / "unity_car2_physical_draft")
    with pytest.raises(ConfigError, match="目录名称"):
        export_physical_draft(source, tmp_path / "ackermann_physical_runtime")


def test_read_only_output_parent_does_not_report_success(tmp_path):
    if os.geteuid() == 0:
        pytest.skip("Permission denial requires an unprivileged user")
    source = SOURCE / "unity_car1" / "vehicle_config.json"
    parent = tmp_path / "readonly"
    parent.mkdir()
    parent.chmod(0o555)
    output = parent / "unity_car1_physical_draft"
    try:
        with pytest.raises(PermissionError):
            export_physical_draft(source, output)
        assert not output.exists()
    finally:
        parent.chmod(0o755)


def test_failed_directory_commit_restores_previous_complete_draft(tmp_path, monkeypatch):
    from agribot_vehicle_config_tools import physical_draft
    source = SOURCE / "unity_car1" / "vehicle_config.json"
    output = tmp_path / "unity_car1_physical_draft"
    export_physical_draft(source, output)
    previous = {p.name: p.read_bytes() for p in output.iterdir()}
    replace = os.replace

    def fail_staged_commit(src, dst):
        if Path(src).name == output.name and Path(dst) == output:
            raise OSError("Injected commit failure")
        return replace(src, dst)

    monkeypatch.setattr(physical_draft.os, "replace", fail_staged_commit)
    with pytest.raises(OSError, match="Injected commit failure"):
        export_physical_draft(source, output)
    assert {p.name: p.read_bytes() for p in output.iterdir()} == previous
