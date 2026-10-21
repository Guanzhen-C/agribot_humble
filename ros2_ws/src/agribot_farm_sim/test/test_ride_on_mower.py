import json
import math
import os
from pathlib import Path

import pytest

from agribot_farm_sim.mounts import mounted_configuration
from agribot_vehicle_config_tools.implements import collision_boxes
from agribot_vehicle_config_tools.model import ConfigError, footprint


ROOT = Path(__file__).resolve().parents[1]


def test_mower_cad_mapping_keeps_forks_separate_from_wheels():
    profile = json.loads((ROOT / "config/ride_on_mower_cad.json").read_text())
    assert profile["vehicleType"] == "ackermann"
    assert "minTurningRadiusM" not in profile["simulation"]
    assert set(profile["casters"]) == {"FLwheel", "FRwheel"}
    wheel_solids = [solid for group in profile["wheelGroups"].values() for solid in group]
    fork_solids = [solid for group in profile["casters"].values() for solid in group["forkSolids"]]
    assert len(set(wheel_solids + fork_solids)) == len(wheel_solids + fork_solids)
    assert profile["recalculateNormals"] is True
    assert profile["simulation"]["frontWheelDriven"] is False


def test_local_mower_caster_geometry_and_sensor_assets():
    assets = Path(os.environ.get("AGRIBOT_FARM_ASSET_ROOT", Path.home() / ".local/share/agribot/farm_assets"))
    source = assets / "ride_on_mower/ros_v5"
    if not source.is_dir():
        pytest.skip("Local mower CAD assets not installed")
    config = json.loads((source / "vehicle_config.json").read_text())
    assert config["metadata"]["calibrationState"] == "unity_nominal_simulation_only"
    wheels = {wheel["id"]: wheel for wheel in config["geometry"]["wheelCenters"]}
    ground = -config["geometry"]["wheelRadiusM"]
    for wheel in wheels.values():
        assert wheel["position"][2] - wheel["radiusM"] == pytest.approx(ground)
        assert wheel["driven"] == wheel["id"].startswith("rear")
        if not wheel["driven"]:
            # A positive mechanical trail lets the passive fork follow motion.
            assert wheel["caster"]["pivotPosition"][0] - wheel["position"][0] > .03
            assert (source / (wheel["id"] + "_caster.obj")).is_file()
    for sensor in ("imu", "lidar", "right_camera", "rtk_master"):
        assert (source / f"sensor_{sensor}_visual.obj").stat().st_size > 0
    report = json.loads((source / "sensor_mount_report.json").read_text())
    assert report["positions"]["right_camera"][0] > report["positions"]["lidar"][0]
    assert report["positions"]["right_camera"][2] > report["positions"]["lidar"][2]


def test_front_cutter_clears_full_caster_sweep_and_extends_navigation_footprint():
    assets = Path(os.environ.get("AGRIBOT_FARM_ASSET_ROOT", Path.home() / ".local/share/agribot/farm_assets"))
    source = assets / "ride_on_mower/ros_v5/vehicle_config.json"
    tool = assets / "front_sickle_bar/ros_v2/implement_asset.json"
    if not source.is_file() or not tool.is_file():
        pytest.skip("Local mower/front cutter assets not installed")
    vehicle, asset = json.loads(source.read_text()), json.loads(tool.read_text())
    before = json.dumps(vehicle, sort_keys=True)
    config = mounted_configuration(vehicle, asset, "mower_front_cutter")
    assert json.dumps(vehicle, sort_keys=True) == before
    assert config["sensors"] == vehicle["sensors"]
    assert config["implements"][0]["pose"]["xyz"][0] > vehicle["geometry"]["bodyExtents"]["frontM"]
    assert config["metadata"]["implementGroundClearanceM"] == .12
    assert 0 < config["metadata"]["implementTransportBalance"]["frontAxleLoadFraction"] < 1
    boxes = collision_boxes(config)
    assert max(p[0] for p in footprint(config)) > max(p[0] for p in footprint(vehicle)) + .4
    for wheel in config["geometry"]["wheelCenters"]:
        if "caster" not in wheel:
            continue
        assert "/mower_front_cutter/" in wheel["caster"]["meshUri"]
        pivot = wheel["caster"]["pivotPosition"]
        # Conservative planar swept disk enclosing the entire rotating tire.
        trail = math.dist(pivot[:2], wheel["position"][:2])
        radius = math.hypot(trail + wheel["radiusM"], wheel["widthM"] / 2)
        for box in boxes:
            distance = math.hypot(max(box[0] - pivot[0], 0, pivot[0] - box[3]),
                                  max(box[1] - pivot[1], 0, pivot[1] - box[4]))
            assert distance - radius > .02, (wheel["id"], box)
    lidar = next(s for s in config["sensors"] if s["id"] == "lidar")["pose"]["xyz"]
    camera = next(s for s in config["sensors"] if s["id"] == "right_camera")["pose"]["xyz"]
    for sensor in (lidar, camera):
        for box in boxes:
            if box[3] > sensor[0]:
                assert box[5] < sensor[2] - .02
    vehicle["vehicleId"] = "compact_tractor"
    with pytest.raises(ConfigError, match="ride_on_mower"):
        mounted_configuration(vehicle, asset, "wrong_vehicle")
