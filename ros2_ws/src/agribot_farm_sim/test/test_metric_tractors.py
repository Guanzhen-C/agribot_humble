import json
import math
from pathlib import Path

import pytest
import xml.etree.ElementTree as ET

from agribot_farm_sim.compatibility import pose_occupancy
from agribot_farm_sim.mounts import mounted_configuration
from agribot_farm_sim.scenarios import build, route
from agribot_vehicle_config_tools.model import ConfigError, footprint


GENERATED = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/generated"


@pytest.mark.parametrize("vehicle,length,width", [("unity_car1", 4.8, 2.56),
    ("unity_car2", 5.2, 2.93), ("unity_car4", 3.6, 2.11), ("unity_car5", 3.4, 1.54)])
def test_metric_vehicle_has_consistent_contact_dynamics_and_height_band(vehicle, length, width):
    data = json.loads((GENERATED / vehicle / "vehicle_config.json").read_text())
    geometry = data["geometry"]
    extents = geometry["bodyExtents"]
    assert extents["frontM"] + extents["rearM"] == pytest.approx(length)
    assert extents["leftM"] + extents["rightM"] == pytest.approx(width, abs=.006)
    assert data["metadata"]["calibrationState"] == "unity_nominal_simulation_only"
    assert "estimate" in data["metadata"]["nominalDimensions"]["basis"]
    assert data["dynamics"]["massKg"] >= 1400
    for wheel in geometry["wheelCenters"]:
        assert wheel["position"][2] - wheel["radiusM"] == pytest.approx(-geometry["wheelRadiusM"])
    nav = data["navigation"]
    assert -geometry["wheelRadiusM"] < nav["obstacleMinHeightM"] < nav["obstacleMaxHeightM"]
    assert nav["maxAngularSpeedRadps"] == pytest.approx(nav["maxLinearSpeedMps"] / geometry["minTurningRadiusM"])
    assert geometry["minTurningRadiusM"] == pytest.approx(geometry["wheelbaseM"] / math.tan(geometry["maxSteeringAngleRad"]))
    sensors = {s["id"]: s for s in data["sensors"]}
    assert sensors["rtk_master"]["pose"]["xyz"][1] - sensors["rtk_secondary"]["pose"]["xyz"][1] == pytest.approx(.65)
    contacts = data["metadata"]["sensorMountContacts"]
    assert len(contacts) == 6
    assert all("Steering" not in foot["sourcePart"] for foot in contacts)
    assert sensors["lidar"]["pose"]["xyz"][0] > 1.5
    assert sensors["right_camera"]["pose"]["xyz"][0] > sensors["lidar"]["pose"]["xyz"][0]
    assert sensors["right_camera"]["pose"]["xyz"][2] > sensors["lidar"]["pose"]["xyz"][2]


@pytest.mark.parametrize("vehicle", ["unity_car1", "unity_car2", "unity_car4", "unity_car5"])
def test_metric_tractor_inner_joint_admits_configured_virtual_bicycle_angle(vehicle):
    from agribot_vehicle_config_tools.generator import _steering_joint_limit
    config = json.loads((GENERATED / vehicle / "vehicle_config.json").read_text())
    geometry = config["geometry"]
    expected = math.atan2(geometry["wheelbaseM"],
                         geometry["minTurningRadiusM"] - geometry["frontTrackM"] / 2)
    assert _steering_joint_limit(config) == pytest.approx(expected)
    assert expected > geometry["maxSteeringAngleRad"]
    sdf = ET.parse(GENERATED / vehicle / "models" / (vehicle + ".sdf"))
    for side in ("left", "right"):
        upper = sdf.findtext(f"./model/joint[@name='front_{side}_steering_joint']/axis/limit/upper")
        assert float(upper) == pytest.approx(expected)


def test_physical_ackermann_steering_limit_is_not_expanded():
    from agribot_vehicle_config_tools.generator import _steering_joint_limit
    config = json.loads((GENERATED / "ackermann_current/vehicle_config.json").read_text())
    assert _steering_joint_limit(config) == config["geometry"]["maxSteeringAngleRad"]


def test_old_miniature_profile_cannot_receive_large_implement():
    data = json.loads((GENERATED / "unity_car1/vehicle_config.json").read_text())
    data["metadata"].pop("nominalDimensions")
    with pytest.raises(ConfigError, match="metric"):
        mounted_configuration(data, {"id": "mounted_cultivator"}, "bad")


def test_wide_route_keeps_full_header_footprint_clear_through_the_turn(tmp_path):
    import yaml
    from PIL import Image
    directory = build("wide_headland", tmp_path)
    metadata = yaml.safe_load((directory / "map.yaml").read_text())
    image = Image.open(metadata["image"])
    config = json.loads((GENERATED / "harvester_header/vehicle_config.json").read_text())
    polygon = footprint(config)
    assert max(p[1] for p in polygon) - min(p[1] for p in polygon) > 12.7
    points = [(-25, 0, 0), *route("wide_headland")]
    for x, y, yaw in points:
        assert pose_occupancy(image, metadata, polygon, [x, y, yaw]) == "free"
    # Check the entire designed 12 m radius headland arc, not just its targets.
    for step in range(91):
        angle = step * math.pi / 180
        assert pose_occupancy(image, metadata, polygon,
            [10 + 12*math.sin(angle), 12-12*math.cos(angle), angle]) == "free"
