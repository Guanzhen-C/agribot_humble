from pathlib import Path

import pytest
import yaml

from agribot_mobile_app.profiles import ProfileError, RuntimeProfiles


def profile_file(tmp_path: Path):
    path = tmp_path / "profiles.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "profiles": {
                    "indoor": {
                        "label": "室内",
                        "launch_package": "agribot_hardware_bringup",
                        "launch_file": "ackermann.launch.py",
                        "map_argument": "map_base",
                        "required_suffixes": [".yaml", ".pcd"],
                        "fixed_args": {"rviz": False},
                        "motion_args": {"authorization": "ENABLE"},
                    }
                }
            },
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return path


def test_builds_whitelisted_observe_and_motion_commands(tmp_path):
    profiles = RuntimeProfiles(profile_file(tmp_path))
    map_base = tmp_path / "map"
    map_base.with_suffix(".yaml").write_text("image: map.pgm\n")
    map_base.with_suffix(".pcd").write_bytes(b"pcd")

    observe = profiles.command("indoor", map_base, False)
    assert observe[:4] == [
        "ros2",
        "launch",
        "agribot_hardware_bringup",
        "ackermann.launch.py",
    ]
    assert f"map_base:={map_base}" in observe
    assert "enable_chassis_output:=false" in observe
    assert "authorization:=ENABLE" not in observe

    motion = profiles.command("indoor", map_base, True)
    assert "enable_chassis_output:=true" in motion
    assert "authorization:=ENABLE" in motion


def test_rejects_missing_map_artifact(tmp_path):
    profiles = RuntimeProfiles(profile_file(tmp_path))
    (tmp_path / "map.yaml").write_text("image: map.pgm\n")
    with pytest.raises(ProfileError, match="缺少文件"):
        profiles.command("indoor", tmp_path / "map", False)


def test_accepts_map_specific_visual_index_suffix(tmp_path):
    path = profile_file(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["profiles"]["indoor"]["required_suffixes"].append(
        "_visual_index.npz"
    )
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    map_base = tmp_path / "map"
    map_base.with_suffix(".yaml").write_text("image: map.pgm\n")
    map_base.with_suffix(".pcd").write_bytes(b"pcd")
    Path(f"{map_base}_visual_index.npz").write_bytes(b"index")
    command = RuntimeProfiles(path).command("indoor", map_base, False)
    assert f"map_base:={map_base}" in command
