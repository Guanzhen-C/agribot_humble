import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
COMMANDS_ROOT = PACKAGE_ROOT / "docs/commands"
LAUNCH_PATH = (
    PACKAGE_ROOT
    / "ackermann/launch/ackermann_outdoor_0811_experiment.launch.py"
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "ackermann_outdoor_0811_experiment", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_map_set(tmp_path):
    map_base = tmp_path / "map_lio_sam_0811"
    pcd = map_base.with_suffix(".pcd")
    pgm = map_base.with_suffix(".pgm")
    nav2_yaml = map_base.with_suffix(".yaml")
    georeference = Path(f"{map_base}_georeference.yaml")
    camera = tmp_path / "camera"

    pcd.write_bytes(b"pcd-test")
    pgm.write_bytes(b"P5\n1 1\n255\n\xff")
    nav2_yaml.write_text("image: map_lio_sam_0811.pgm\n")
    georeference.write_text(
        yaml.safe_dump(
            {
                "map": {"id": map_base.name},
                "calibration": {
                    "horizontal_rmse_m": 0.15,
                    "yaw_validation_passed": False,
                },
            }
        )
    )
    camera.touch()

    profile = tmp_path / "profile.yaml"
    profile.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": map_base.name,
                "files": {
                    "pcd": {"suffix": ".pcd", "sha256": sha256(pcd)},
                    "pgm": {"suffix": ".pgm", "sha256": sha256(pgm)},
                    "nav2_yaml": {
                        "suffix": ".yaml",
                        "sha256": sha256(nav2_yaml),
                    },
                    "georeference": {
                        "suffix": "_georeference.yaml",
                        "sha256": sha256(georeference),
                    },
                },
                "georeference_policy": {
                    "maximum_horizontal_rmse_m": 0.20,
                    "allow_unvalidated_yaw_as_coarse_prior": True,
                },
            }
        )
    )
    return map_base, profile, pgm, camera


def context_for(map_base, profile, camera):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "map_base": str(map_base),
            "map_profile": str(profile),
            "right_camera_device": str(camera),
            "enable_chassis_output": "false",
            "can_transport": "zqwl_cdc",
            "zqwl_port": "/missing/can",
            "initialization_source": "rtk",
            "enable_fpfh": "false",
            "allow_missing_georeference": "false",
        }
    )
    return context


def test_complete_matching_map_set_passes_preflight(tmp_path, monkeypatch):
    module = load_launch_module()
    map_base, profile, _pgm, camera = make_map_set(tmp_path)
    monkeypatch.setattr(module, "REQUIRED_SENSOR_DEVICES", (("test", camera),))
    assert module._validate_experiment(
        context_for(map_base, profile, camera)
    ) == []


def test_modified_processed_pgm_is_rejected(tmp_path, monkeypatch):
    module = load_launch_module()
    map_base, profile, pgm, camera = make_map_set(tmp_path)
    monkeypatch.setattr(module, "REQUIRED_SENSOR_DEVICES", (("test", camera),))
    pgm.write_bytes(pgm.read_bytes() + b"changed")
    with pytest.raises(RuntimeError, match="PGM|地图文件版本不匹配"):
        module._validate_experiment(context_for(map_base, profile, camera))


def test_chassis_stage_requires_usb_can_device(tmp_path, monkeypatch):
    module = load_launch_module()
    map_base, profile, _pgm, camera = make_map_set(tmp_path)
    monkeypatch.setattr(module, "REQUIRED_SENSOR_DEVICES", (("test", camera),))
    context = context_for(map_base, profile, camera)
    context.launch_configurations["enable_chassis_output"] = "true"
    with pytest.raises(RuntimeError, match="USB-CAN设备不存在"):
        module._validate_experiment(context)


def test_missing_required_sensor_device_is_rejected(tmp_path, monkeypatch):
    module = load_launch_module()
    map_base, profile, _pgm, camera = make_map_set(tmp_path)
    missing = tmp_path / "missing-rtk"
    monkeypatch.setattr(module, "REQUIRED_SENSOR_DEVICES", (("RTK", missing),))
    with pytest.raises(RuntimeError, match="RTK串口设备不存在"):
        module._validate_experiment(context_for(map_base, profile, camera))


def test_outdoor_localization_policy_cannot_be_overridden(tmp_path, monkeypatch):
    module = load_launch_module()
    map_base, profile, _pgm, camera = make_map_set(tmp_path)
    monkeypatch.setattr(module, "REQUIRED_SENSOR_DEVICES", (("test", camera),))
    context = context_for(map_base, profile, camera)
    context.launch_configurations["initialization_source"] = "manual"
    with pytest.raises(RuntimeError, match="initialization_source:=rtk"):
        module._validate_experiment(context)


def test_launch_is_rtk_only_and_safe_by_default():
    source = LAUNCH_PATH.read_text()
    assert module_default_map() in source
    assert 'DeclareLaunchArgument("initialization_source", default_value="rtk")' in source
    assert '"allow_missing_georeference", default_value="false"' in source
    assert 'DeclareLaunchArgument("enable_fpfh", default_value="false")' in source
    assert '"enable_chassis_output",\n                default_value="false"' in source


def test_commands_are_split_by_function_and_outdoor_guide_is_two_stage():
    expected_guides = {
        "01_build_and_cleanup.txt",
        "02_simulation.txt",
        "03_rtk_eskf_validation.txt",
        "04_sensor_data_collection.txt",
        "05_offline_mapping_and_comparison.txt",
        "06_map_and_planner_validation.txt",
        "07_georeference_validation.txt",
        "08_physical_navigation.txt",
        "09_outdoor_0811_full_experiment.txt",
        "10_can_and_desktop_gui.txt",
    }
    assert {path.name for path in COMMANDS_ROOT.glob("*.txt")} == expected_guides

    old_command_file = PACKAGE_ROOT.parent / "2.txt"
    index = old_command_file.read_text()
    assert "命令索引" in index
    assert "ros2 launch" not in index

    outdoor = (COMMANDS_ROOT / "09_outdoor_0811_full_experiment.txt").read_text()
    assert "/media/cgz/data/agribot_data/maps/map_lio_sam_0811" in outdoor
    assert "ackermann_outdoor_0811_experiment.launch.py" in outdoor
    stage_a = outdoor.index("rviz:=true enable_chassis_output:=false")
    stage_b = outdoor.index("rviz:=true enable_chassis_output:=true")
    assert stage_a < stage_b
    assert "--checksum" in outdoor
    assert "不创建底盘节点" in outdoor


def module_default_map():
    return "/home/sunrise/agribot_maps/test_site/map_lio_sam_0811"
