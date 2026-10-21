import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.casters import copy_meshes
from agribot_vehicle_config_tools.generator import generate_sdf, generate_simulation_sdf, generate_urdf
from agribot_vehicle_config_tools.model import ConfigError, validate


BASELINE = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/config/vehicles/ackermann_current/vehicle_config.json"


def configuration():
    config = json.loads(BASELINE.read_text())
    config["vehicleType"] = "differential"
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
        wheel["driven"] = wheel["id"].startswith("rear")
        if not wheel["driven"]:
            wheel["caster"] = dict(pivotPosition=[wheel["position"][0] + .05, wheel["position"][1], .1],
                massKg=1.5, damping=.02,
                meshUri=f"package://agribot_vehicle_description/generated/{config['vehicleId']}/models/{wheel['id']}_caster.obj")
    validate(config)
    return config


def test_caster_forks_and_single_driven_axle_in_both_sdf_exports(tmp_path):
    config = configuration()
    for sdf in (generate_sdf(config), generate_simulation_sdf(config, BASELINE.parents[4], tmp_path)):
        model = ET.fromstring(sdf).find("model")
        drive = model.find("plugin[@name='configured_diff_drive']")
        assert drive.findtext("num_wheel_pairs") == "1"
        assert [item.text for item in drive.findall("left_joint")] == ["rear_left_wheel_joint"]
        for side in ("left", "right"):
            key = f"front_{side}_caster"
            assert model.findtext(f"joint[@name='{key}_joint']/axis/xyz") == "0 0 1"
            assert model.findtext(f"joint[@name='front_{side}_wheel_joint']/parent") == key + "_link"
            assert model.find(f"link[@name='{key}_link']/visual/geometry/mesh") is not None
            published = model.findall("plugin[@name='configured_joint_states']/joint_name")
            assert key + "_joint" in [item.text for item in published]


def test_urdf_wheel_origin_is_relative_to_caster_pivot():
    config = configuration()
    robot = ET.fromstring(generate_urdf(config))
    for wheel in config["geometry"]["wheelCenters"][:2]:
        spin = robot.find(f"joint[@name='{wheel['id']}_wheel_joint']")
        assert spin.find("parent").get("link") == wheel["id"] + "_caster_link"
        xyz = [float(value) for value in spin.find("origin").get("xyz").split()]
        assert xyz == pytest.approx([a-b for a, b in zip(wheel["position"], wheel["caster"]["pivotPosition"])])


@pytest.mark.parametrize("invalid", ["driven", "steering", "ackermann", "unpaired_drive", "mesh"])
def test_invalid_casters_rejected(invalid):
    config = configuration()
    front = config["geometry"]["wheelCenters"][0]
    if invalid in ("driven", "steering"):
        front[invalid] = True
    elif invalid == "ackermann":
        config["vehicleType"] = "ackermann"
    elif invalid == "mesh":
        front["caster"]["meshUri"] = "file:///tmp/unexpected.obj"
    else:
        config["geometry"]["wheelCenters"][2]["driven"] = False
    with pytest.raises(ConfigError):
        validate(config)


def test_caster_assets_are_required_and_copied(tmp_path):
    config = configuration()
    source, output = tmp_path / "source", tmp_path / "output"
    source.mkdir()
    output.mkdir()
    with pytest.raises(ValueError, match="Missing caster"):
        copy_meshes(config, source, output)
    for side in ("left", "right"):
        for extension in ("obj", "mtl"):
            (source / f"front_{side}_caster.{extension}").write_text("test asset\n")
    copy_meshes(config, source, output)
    assert len(list(output.iterdir())) == 4


def test_existing_four_wheel_differential_drive_unchanged():
    config = json.loads(BASELINE.read_text())
    config["vehicleType"] = "differential"
    for wheel in config["geometry"]["wheelCenters"]:
        wheel["steering"] = False
    model = ET.fromstring(generate_sdf(config)).find("model")
    drive = model.find("plugin[@name='configured_diff_drive']")
    assert drive.findtext("num_wheel_pairs") == "2"
    assert model.find("joint[@name='front_left_caster_joint']") is None
