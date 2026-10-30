import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


SCRIPT = Path(__file__).parents[1] / "tools/apply_planning_update.sh"
CONFIGS = ["nav2_params_differential_planner_only.yaml", "nav2_params_differential_fastlivo_mapped.yaml"]


def deployment_fixture(tmp_path, state=None):
    release, home, workspace, commands = [tmp_path / name for name in ("release", "home", "workspace", "bin")]
    for path in (release, home, workspace, commands):
        path.mkdir()
    map_root = home / "agribot_maps/test_site"
    map_root.mkdir(parents=True)
    (map_root / "field.pcd").write_bytes(b"unchanged localization PCD")
    pcd_sha = hashlib.sha256((map_root / "field.pcd").read_bytes()).hexdigest()
    (release / "manifest.json").write_text(json.dumps({"map_id": "field", "required_existing_pcd_sha256": pcd_sha}))
    (release / "SHA256SUMS").write_text(f"{hashlib.sha256((release / 'manifest.json').read_bytes()).hexdigest()}  manifest.json\n")
    shutil.copy2(SCRIPT, release / SCRIPT.name)
    source = workspace / "src/agribot_mobile_app"
    (source / "config").mkdir(parents=True)
    (source / "package.xml").write_text("old")
    (source / "config/mobile_gateway.yaml").write_text("preserved_device_settings")
    payload = release / "code/agribot_mobile_app"
    payload.mkdir(parents=True)
    (payload / "package.xml").write_text("new")
    (workspace / "install").mkdir()
    (workspace / "install/setup.bash").write_text("")
    (release / "maps").mkdir()
    (release / "maps/field.pgm").write_bytes(b"new map")
    (map_root / "field.pgm").write_bytes(b"old map")
    for directory in (workspace / "src/agribot_hardware_bringup/differential/config", workspace / "install/agribot_hardware_bringup/share/agribot_hardware_bringup/differential/config", release / "code/agribot_hardware_bringup/differential/config"):
        directory.mkdir(parents=True)
        for name in CONFIGS:
            (directory / name).write_text("new" if str(directory).startswith(str(release)) else "old")
    state_path = tmp_path / "state.json"
    state_path.write_text(json.dumps(state or {"processes": {}, "command": {}, "navigation": {"status": "idle"}}))
    scripts = {
        "uname": '#!/bin/sh\necho aarch64\n',
        "sudo": '#!/bin/sh\nexec "$@"\n',
        "systemctl": '#!/bin/sh\necho "systemctl $*" >> "$TEST_COMMAND_LOG"\nexit 0\n',
        "colcon": '#!/bin/sh\necho "colcon $*" >> "$TEST_COMMAND_LOG"\n',
        "curl": '''#!/usr/bin/python3
import json, os, pathlib, sys
args = sys.argv[1:]
if any('/routes' in arg for arg in args):
    body = json.dumps({'routes': [{'poses': [0]*53}]*2})
elif any('/state' in arg for arg in args):
    body = pathlib.Path(os.environ['TEST_STATE']).read_text()
else:
    body = '{}'
    with open(os.environ['TEST_COMMAND_LOG'], 'a') as stream: stream.write('runtime_stop\\n')
if '-o' in args: pathlib.Path(args[args.index('-o')+1]).write_text(body)
else: print(body)
''',
    }
    for name, content in scripts.items():
        path = commands / name
        path.write_text(content)
        path.chmod(0o755)
    environment = {**os.environ, "HOME": str(home), "AGRIBOT_WORKSPACE": str(workspace), "AGRIBOT_MAP_ROOT": str(map_root), "PATH": f"{commands}:{os.environ['PATH']}", "TEST_COMMAND_LOG": str(tmp_path / "commands.log"), "TEST_STATE": str(state_path)}
    return release, home, workspace, map_root, environment


def test_planning_deployment_preserves_device_config_and_never_starts_vehicle(tmp_path):
    release, home, workspace, maps, env = deployment_fixture(tmp_path)
    checked = subprocess.run(["bash", str(release / SCRIPT.name), "--check"], env=env, capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr
    result = subprocess.run(["bash", str(release / SCRIPT.name), "--apply"], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert (workspace / "src/agribot_mobile_app/package.xml").read_text() == "new"
    assert (workspace / "src/agribot_mobile_app/config/mobile_gateway.yaml").read_text() == "preserved_device_settings"
    assert (maps / "field.pcd").read_bytes() == b"unchanged localization PCD"
    assert (maps / "field.pgm").read_bytes() == b"new map"
    assert list((home / "agribot_backups").glob("*/mobile_source.tgz"))
    commands = Path(env["TEST_COMMAND_LOG"]).read_text()
    assert "runtime_stop" in commands
    assert "--packages-select agribot_mobile_app" in commands
    assert "/runtime/start" not in commands


@pytest.mark.parametrize("state", [{"processes": {"collection": {"running": True}}}, {"navigation": {"status": "executing"}}, {"command": {"linear": .2}}])
def test_planning_deployment_refuses_recording_navigation_or_nonzero_command(tmp_path, state):
    release, _, workspace, maps, env = deployment_fixture(tmp_path, state)
    result = subprocess.run(["bash", str(release / SCRIPT.name), "--apply"], env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert (workspace / "src/agribot_mobile_app/package.xml").read_text() == "old"
    assert (maps / "field.pgm").read_bytes() == b"old map"


def test_planning_deployment_rejects_wrong_pcd_before_stopping_service(tmp_path):
    release, _, workspace, maps, env = deployment_fixture(tmp_path)
    (maps / "field.pcd").write_bytes(b"wrong localization map")
    result = subprocess.run(["bash", str(release / SCRIPT.name), "--apply"], env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert (workspace / "src/agribot_mobile_app/package.xml").read_text() == "old"
    assert not Path(env["TEST_COMMAND_LOG"]).exists()
