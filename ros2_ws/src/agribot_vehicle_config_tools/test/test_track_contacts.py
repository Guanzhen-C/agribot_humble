import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.model import validate
from agribot_vehicle_config_tools.track_contacts import align_car3_contacts


DESCRIPTION = Path(__file__).resolve().parents[2] / "agribot_vehicle_description"
BUNDLE = DESCRIPTION / "generated/unity_car3"


def current():
    return json.loads((BUNDLE / "vehicle_config.json").read_text())


def legacy():
    config = current()
    config["metadata"].pop("simulationTrackContactGeometry")
    before = {w["id"]: w["radiusM"] for w in config["geometry"]["wheelCenters"]}
    for rotor in config["metadata"]["simulationVisualRotors"]:
        rotor["ratio"] *= .2249184399843216 / before[rotor["sourceWheel"]]
    for belt in config["metadata"]["simulationVisualBelts"]:
        belt["sourceRadius"] = .2249184399843216
    for wheel in config["geometry"]["wheelCenters"]:
        wheel.update(position=[.201 if wheel["id"].startswith("front") else -.201,
                               .2945 if wheel["id"].endswith("left") else -.2945, 0],
                     radiusM=.2249184399843216)
    return config


def test_contacts_follow_visible_idlers_without_moving_vehicle_or_sensors():
    config = legacy()
    before = copy.deepcopy(config)
    aligned = align_car3_contacts(config, BUNDLE / "models")
    validate(aligned)
    assert config == before
    for field in ("baseFrame", "navigation", "sensors", "visuals", "collision", "dynamics", "manipulators"):
        assert aligned.get(field) == before.get(field)
    assert aligned["geometry"]["bodyExtents"] == before["geometry"]["bodyExtents"]
    assert aligned["geometry"]["wheelbaseM"] == pytest.approx(.8370031098092811)
    assert aligned["geometry"]["frontTrackM"] == pytest.approx(.6011831611394882)
    profile = aligned["metadata"]["simulationTrackContactGeometry"]
    assert not profile["physicalMeasurement"]
    rotors = {r["id"]: r for r in aligned["metadata"]["simulationVisualRotors"]}
    wheels = {w["id"]: w for w in aligned["geometry"]["wheelCenters"]}
    for item in profile["contacts"]:
        wheel = wheels[item["wheel"]]
        assert wheel["position"] == rotors[item["rotor"]]["center"]
        assert wheel["position"][2] - wheel["radiusM"] == pytest.approx(item["treadBottomM"])
        assert .02 < item["treadAllowanceM"] < .04
        assert .11 < wheel["radiusM"] < .13


def test_animation_keeps_visible_rotation_and_belt_travel_per_meter():
    before = legacy()
    after = align_car3_contacts(before, BUNDLE / "models")
    old_wheels = {w["id"]: w for w in before["geometry"]["wheelCenters"]}
    new_wheels = {w["id"]: w for w in after["geometry"]["wheelCenters"]}
    for old, new in zip(before["metadata"]["simulationVisualRotors"], after["metadata"]["simulationVisualRotors"]):
        assert old["center"] == new["center"]
        assert old_wheels[old["sourceWheel"]]["radiusM"] / old["ratio"] == pytest.approx(
            new_wheels[new["sourceWheel"]]["radiusM"] / new["ratio"])
    for belt in after["metadata"]["simulationVisualBelts"]:
        assert belt["sourceRadius"] == new_wheels[belt["sourceWheel"]]["radiusM"]


def test_canonical_sdf_urdf_and_drive_use_same_actual_centers():
    config = current()
    assert config == json.loads((DESCRIPTION / "config/vehicles/unity_car3/vehicle_config.json").read_text())
    sdf = ET.parse(BUNDLE / "models/unity_car3.sdf")
    urdf = ET.parse(BUNDLE / "urdf/unity_car3.urdf.xacro")
    for wheel in config["geometry"]["wheelCenters"]:
        link = sdf.find(f".//link[@name='{wheel['id']}_wheel_link']")
        assert [float(v) for v in link.findtext("pose").split()[:3]] == pytest.approx(wheel["position"])
        assert float(link.findtext("collision/geometry/cylinder/radius")) == pytest.approx(wheel["radiusM"])
        joint = urdf.find(f"joint[@name='{wheel['id']}_wheel_joint']")
        assert [float(v) for v in joint.find("origin").get("xyz").split()] == pytest.approx(wheel["position"])
    drive = sdf.find(".//plugin[@name='configured_diff_drive']")
    assert [float(v.text) for v in drive.findall("wheel_diameter")] == pytest.approx(
        [config["geometry"]["wheelRadiusM"] * 2] * 2)
    for scene in ("arable", "greenhouse"):
        label = DESCRIPTION.parent / f"agribot_farm_sim/config/manual_confirmations_car3_{scene}.json"
        historical = json.loads(label.read_text())["confirmations"][0]["vehicleConfigSha256"]
        assert historical != hashlib.sha256((BUNDLE / "vehicle_config.json").read_bytes()).hexdigest()


def test_wrong_vehicle_and_already_applied_profile_are_rejected():
    config = legacy()
    config["vehicleId"] = "tracked_harvester"
    with pytest.raises(ValueError, match="only"):
        align_car3_contacts(config, BUNDLE / "models")
    with pytest.raises(ValueError, match="already"):
        align_car3_contacts(current(), BUNDLE / "models")
