from pathlib import Path

import pytest
import yaml

from agribot_mobile_app.profiles import ProfileError, RuntimeProfiles


def profile_file(tmp_path: Path):
    path = tmp_path / "profiles.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "vehicles": {
                    "ackermann": {
                        "label": "阿克曼车",
                        "footprint": [
                            [0.7, 0.4],
                            [0.7, -0.4],
                            [-0.2, -0.4],
                            [-0.2, 0.4],
                        ],
                        "collection": {
                            "launch_package": "agribot_hardware_bringup",
                            "launch_file": "ackermann_collection.launch.py",
                            "output_argument": "bag_output",
                            "fixed_args": {"record_bag": True},
                        },
                    }
                },
                "profiles": {
                    "indoor": {
                        "label": "室内",
                        "vehicle_type": "ackermann",
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

    observe = profiles.command("indoor", map_base, False, "ackermann")
    assert observe[:4] == [
        "ros2",
        "launch",
        "agribot_hardware_bringup",
        "ackermann.launch.py",
    ]
    assert f"map_base:={map_base}" in observe
    assert "enable_chassis_output:=false" in observe
    assert "authorization:=ENABLE" not in observe

    motion = profiles.command("indoor", map_base, True, "ackermann")
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


def test_builds_vehicle_specific_collection_command(tmp_path):
    profiles = RuntimeProfiles(profile_file(tmp_path))
    command = profiles.collection_command(
        "ackermann", tmp_path / "bags" / "test_bag"
    )
    assert command[:4] == [
        "ros2",
        "launch",
        "agribot_hardware_bringup",
        "ackermann_collection.launch.py",
    ]
    assert "record_bag:=true" in command
    assert f"bag_output:={tmp_path / 'bags' / 'test_bag'}" in command


def test_rejects_profile_from_another_vehicle(tmp_path):
    profiles = RuntimeProfiles(profile_file(tmp_path))
    map_base = tmp_path / "map"
    map_base.with_suffix(".yaml").write_text("image: map.pgm\n")
    map_base.with_suffix(".pcd").write_bytes(b"pcd")
    with pytest.raises(ProfileError, match="不属于所选车型"):
        profiles.command("indoor", map_base, False, "differential")


def test_production_profiles_dispatch_to_each_chassis_stack(tmp_path):
    profiles = RuntimeProfiles(
        Path(__file__).parents[1] / "config" / "runtime_profiles.yaml"
    )
    map_base = tmp_path / "map"
    for suffix in (
        ".yaml",
        ".pgm",
        ".pcd",
        "_visual_index.npz",
        "_georeference.yaml",
    ):
        Path(f"{map_base}{suffix}").write_bytes(b"test")

    ackermann = profiles.command(
        "ackermann_indoor", map_base, True, "ackermann"
    )
    ackermann_observe = profiles.command(
        "ackermann_indoor", map_base, False, "ackermann"
    )
    differential = profiles.command(
        "differential_indoor", map_base, True, "differential"
    )
    differential_observe = profiles.command(
        "differential_indoor", map_base, False, "differential"
    )

    assert "ackermann_mppi_fastlivo_rtk_mapped.launch.py" in ackermann
    assert "allow_uncalibrated_camera:=true" in ackermann_observe
    assert "allow_uncalibrated_camera:=false" in ackermann
    assert "motion_authorization:=ENABLE_DIFFERENTIAL_MOTION" not in ackermann
    assert "differential_mppi_fastlivo_rtk_mapped.launch.py" in differential
    assert "allow_uncalibrated_camera:=true" in differential_observe
    assert "allow_uncalibrated_camera:=false" in differential
    assert "motion_authorization:=ENABLE_DIFFERENTIAL_MOTION" in differential


def test_differential_outdoor_uses_rtk_map_without_visual_index(tmp_path):
    profiles = RuntimeProfiles(
        Path(__file__).parents[1] / "config" / "runtime_profiles.yaml"
    )
    map_base = tmp_path / "outdoor"
    for suffix in (".yaml", ".pgm", ".pcd", "_georeference.yaml"):
        Path(f"{map_base}{suffix}").write_bytes(b"test")

    command = profiles.command(
        "differential_outdoor", map_base, False, "differential"
    )

    assert "differential_outdoor_experiment.launch.py" in command
    assert "initialization_source:=rtk" in command
    assert "enable_chassis_output:=false" in command
