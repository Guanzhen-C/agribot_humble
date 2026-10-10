import copy
import json
import threading
from types import SimpleNamespace
from pathlib import Path

import pytest

from agribot_mobile_app.physical_config import PhysicalConfigStore
from agribot_mobile_app.profiles import RuntimeProfiles
from agribot_vehicle_config_tools.physical_config import (
    PhysicalConfigError, digest, export_document, template_paths,
)

SOURCE = Path(__file__).parents[2]


@pytest.fixture
def store(tmp_path):
    return PhysicalConfigStore(tmp_path / "config", template_paths(SOURCE))


def test_no_import_does_not_change_either_vehicle_command(store):
    assert store.public()["source"] == "default"
    assert store.launch_arguments("ackermann") == {}
    assert store.launch_arguments("differential") == {}
    assert not store.root.exists()


def test_round_trip_persists_and_materializes_all_parameters(store):
    document = export_document(template_paths(SOURCE))
    assert len(json.dumps(document).encode()) < 256 * 1024
    store.import_document(document)
    reopened = PhysicalConfigStore(store.root, template_paths(SOURCE))
    assert reopened.public()["source"] == "imported"
    args = reopened.launch_arguments("ackermann")
    for key, path in args.items():
        assert Path(path).read_text() == document["parameters"][key]
        assert Path(path).is_relative_to(store.root)
    assert reopened.launch_arguments("differential") == {}
    reopened.reset()
    assert reopened.launch_arguments("ackermann") == {}
    assert PhysicalConfigStore(store.root, template_paths(SOURCE)).document is None


@pytest.mark.parametrize("field,value", [
    ("version", 99), ("vehicle_type", "differential"),
    ("algorithms", {"controller": "unverified"}),
    ("parameters", {"../../malicious": "unsafe"}),
    ("sha256", "wrong"),
])
def test_invalid_import_preserves_previous_configuration(store, field, value):
    document = store.baseline()
    store.import_document(document)
    original = (store.root / "imported.json").read_bytes()
    wrong = copy.deepcopy(document)
    wrong[field] = value
    with pytest.raises(PhysicalConfigError):
        store.import_document(wrong)
    assert (store.root / "imported.json").read_bytes() == original
    assert store.document == document


def test_recomputed_digest_cannot_enable_unverified_parameters(store):
    wrong = store.baseline()
    wrong["parameters"]["nav2_params"] += "\nmalicious: true\n"
    wrong["sha256"] = digest({key: value for key, value in wrong.items() if key != "sha256"})
    with pytest.raises(PhysicalConfigError, match="不一致"):
        store.import_document(wrong)


def test_corrupt_persisted_import_falls_back_without_breaking_startup(store):
    store.root.mkdir()
    (store.root / "imported.json").write_text("broken")
    reopened = PhysicalConfigStore(store.root, store.paths)
    assert reopened.public()["source"] == "default"
    assert reopened.public()["error"]
    assert reopened.launch_arguments("ackermann") == {}


def test_imported_command_keeps_map_motion_and_authorization_policy(store, tmp_path):
    store.import_document(store.baseline())
    profiles = RuntimeProfiles(SOURCE / "agribot_mobile_app/config/runtime_profiles.yaml")
    map_base = tmp_path / "map"
    for suffix in (".yaml", ".pgm", ".pcd", "_visual_index.npz", "_georeference.yaml"):
        Path(f"{map_base}{suffix}").write_text("test")
    for profile in ("ackermann_indoor", "ackermann_outdoor_0811"):
        default = profiles.command(profile, map_base, False, "ackermann")
        imported = profiles.command(profile, map_base, False, "ackermann", store.launch_arguments("ackermann"))
        assert all(value in imported for value in default)
        assert "enable_chassis_output:=false" in imported
        assert not any("use_sim_time:=true" == value for value in imported)
        assert len(imported) == len(default) + len(store.document["parameters"])


def test_camera_parameters_are_forwarded_through_all_ackermann_wrappers():
    root = SOURCE / "agribot_hardware_bringup/ackermann/launch"
    for name in (
        "ackermann_mppi_fastlivo_rtk_mapped.launch.py",
        "ackermann_fastlivo_rtk_localization.launch.py",
        "ackermann_outdoor_0811_experiment.launch.py",
    ):
        source = (root / name).read_text()
        assert '"fastlivo_camera_config": LaunchConfiguration(' in source


def test_gateway_rejects_configuration_changes_while_a_job_is_running(store):
    from agribot_mobile_app.gateway_node import ApiError, MobileGateway

    gateway = object.__new__(MobileGateway)
    gateway.physical_config = store
    gateway._task_transition_lock = threading.RLock()
    gateway.processes = SimpleNamespace(snapshot=lambda: {"runtime": {"running": True}})
    gateway._touch = lambda: None
    with pytest.raises(ApiError, match="请先停止"):
        gateway.import_physical_config({"vehicle_type": "ackermann", "configuration": store.baseline()})
    assert store.document is None
    gateway.processes = SimpleNamespace(snapshot=lambda: {"runtime": {"running": False}})
    result = gateway.import_physical_config({"vehicle_type": "ackermann", "configuration": store.baseline()})
    assert result["physical_config"]["source"] == "imported"
    assert gateway.reset_physical_config({})["physical_config"]["source"] == "default"


def test_gateway_does_not_import_ackermann_parameters_into_a_differential_vehicle(store):
    from agribot_mobile_app.gateway_node import MobileGateway

    gateway = object.__new__(MobileGateway)
    gateway.physical_config = store
    with pytest.raises(PhysicalConfigError, match="差速车保持原配置"):
        gateway.import_physical_config({"vehicle_type": "differential", "configuration": store.baseline()})
