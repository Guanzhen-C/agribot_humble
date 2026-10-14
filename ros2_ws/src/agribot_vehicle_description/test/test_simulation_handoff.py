"""Exercise the public stop/export script without launching a real vehicle."""
import json
import os
from pathlib import Path
import subprocess
import time

import pytest

SCRIPT = Path(__file__).parents[1] / "scripts/unity_sync_and_simulate.sh"


def call(tmp_path, *args):
    return subprocess.run(
        ["bash", str(SCRIPT), *args], text=True, capture_output=True,
        env={**os.environ, "XDG_STATE_HOME": str(tmp_path), "AGRIBOT_ROS_WORKSPACE": str(SCRIPT.parents[3])},
        timeout=30,
    )


def test_stop_is_idempotent_and_missing_firmware_blocks_export(tmp_path, monkeypatch):
    monkeypatch.setenv("AGRIBOT_HPM_FIRMWARE_ROOT", str(tmp_path / "missing-firmware"))
    assert call(tmp_path, "--stop").returncode == 0
    assert call(tmp_path, "--stop").returncode == 0
    assert call(tmp_path, "--status").stdout.strip() == "stopped"
    output = tmp_path / "physical.json"
    result = call(tmp_path, "--export-physical", str(output))
    assert result.returncode == 2
    assert "6750" in result.stderr
    assert not output.exists()


def test_draft_export_requires_stopped_session_and_current_car(tmp_path):
    workspace = SCRIPT.parents[3]
    source = workspace / "src/agribot_vehicle_description/config/vehicles/unity_car1"
    output = tmp_path / "unity_car1_physical_draft"
    result = call(tmp_path, "--export-dir", str(source), "--export-physical-draft", str(output))
    assert result.returncode == 0, result.stderr
    assert json.loads((output / "manifest.json").read_text())["deployable"] is False
    wrong = call(tmp_path, "--export-dir", str(source), "--export-physical-draft",
                 str(tmp_path / "unity_car2_physical_draft"))
    assert wrong.returncode != 0


def test_stop_exits_owned_session_but_preserves_unrelated_process(tmp_path):
    state = tmp_path / "agribot-unity-sim"
    state.mkdir()
    owned = subprocess.Popen(["sleep", "60"], start_new_session=True)
    unrelated = subprocess.Popen(["sleep", "60"], start_new_session=True)
    try:
        (state / "simulation.sid").write_text(str(owned.pid))
        (state / "simulation.boot_id").write_text(Path("/proc/sys/kernel/random/boot_id").read_text())
        assert call(tmp_path, "--status").stdout.strip() == "running"
        blocked = call(tmp_path, "--export-physical", str(tmp_path / "blocked.json"))
        assert blocked.returncode != 0
        assert not (tmp_path / "blocked.json").exists()
        source = SCRIPT.parents[3] / "src/agribot_vehicle_description/config/vehicles/unity_car1"
        draft = tmp_path / "unity_car1_physical_draft"
        blocked_draft = call(tmp_path, "--export-dir", str(source),
                             "--export-physical-draft", str(draft))
        assert blocked_draft.returncode != 0
        assert not draft.exists()
        assert call(tmp_path, "--stop").returncode == 0
        owned.wait(timeout=5)
        assert unrelated.poll() is None
        assert not (state / "simulation.sid").exists()
    finally:
        for process in (owned, unrelated):
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)
