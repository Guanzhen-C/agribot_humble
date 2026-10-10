import hashlib
import json
from pathlib import Path
import pytest
from agribot_vehicle_config_tools.deployment_bundle import export_deployment
from agribot_vehicle_config_tools.physical_config import PhysicalConfigError, export_document, template_paths

@pytest.fixture
def firmware(tmp_path):
    root = tmp_path / "fixture_firmware"
    root.mkdir()
    elf = bytearray(32); elf[:6] = b"\x7fELF\x01\x01"; elf[18:20] = b"\xf3\x00"
    # Synthetic test fixture only; never exported as production firmware.
    files = {"hpm6750_ackermann.bin": b"fixture", "hpm6750_ackermann.elf": bytes(elf)}
    for name, data in files.items(): (root / name).write_bytes(data)
    manifest = {"format": "agribot.hpm6750-firmware", "version": 1, "firmware_version": "1.0.0",
        "board": "hpm6750evk2", "profile": "ackermann", "machine_id": 18, "protocol_id": 514,
        "transport": "classic_can", "bitrate": 1000000, "build_type": "flash_xip", "flashing": "manual",
        "files": {name: {"sha256": hashlib.sha256(data).hexdigest(), "size": len(data)} for name, data in files.items()}}
    (root / "manifest.json").write_text(json.dumps(manifest))
    return root

def test_bundle_and_optional_console_json_are_consistent(tmp_path, firmware):
    document = export_document(template_paths(Path(__file__).parents[2]))
    output = tmp_path / "ackermann_physical_runtime.json"
    for _ in range(2):
        bundle = export_deployment(output, document, firmware)
        assert json.loads(output.read_text()) == document
        assert json.loads((bundle / output.name).read_text()) == document
        manifest = json.loads((bundle / "manifest.json").read_text())
        assert manifest["configuration_sha256"] == document["sha256"]
        for name, checksum in manifest["files"].items():
            assert hashlib.sha256((bundle / name).read_bytes()).hexdigest() == checksum

def test_bad_firmware_preserves_existing_exports(tmp_path, firmware):
    output = tmp_path / "current.json"; output.write_text("original")
    (firmware / "hpm6750_ackermann.bin").write_bytes(b"corrupted")
    with pytest.raises(PhysicalConfigError, match="校验失败"):
        export_deployment(output, {}, firmware)
    assert output.read_text() == "original"
    assert not output.with_name("current_deployment").exists()

def test_mismatched_profile_fails(tmp_path, firmware):
    path = firmware / "manifest.json"
    manifest = json.loads(path.read_text()); manifest["profile"] = "tracked"
    path.write_text(json.dumps(manifest))
    with pytest.raises(PhysicalConfigError, match="不匹配"):
        export_deployment(tmp_path / "out.json", {}, firmware)
