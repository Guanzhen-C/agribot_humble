import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import jsonschema
import pytest

from agribot_vehicle_config_tools.generator import generate_bundle, generate_sdf, generate_urdf
from agribot_vehicle_config_tools.implements import collision_boxes, copy_meshes
from agribot_vehicle_config_tools.model import ConfigError, footprint, load_and_validate, validate
from agribot_vehicle_config_tools.sprayer import default_sprayer


ROOT = Path(__file__).parents[1]
BASELINE = ROOT.parent / "agribot_vehicle_description/config/vehicles/ackermann_current/vehicle_config.json"


def configuration(kind="compact_chisel_plow"):
    config, _ = load_and_validate(BASELINE)
    config["implements"] = [{
        "id": kind, "implementType": kind, "parentFrameId": "base_link",
        "frameId": kind + "_link", "workFrameId": kind + "_work_frame",
        "pose": {"xyz": [-1., 0., .15], "rpy": [0., 0., 0.], "quaternion": [0., 0., 0., 1.]},
        "meshFile": "implement_" + kind + "_visual.obj", "mountMode": "fixed_transport",
        "simulationOnly": True, "workingWidthM": 2.2, "massKg": 60.,
        "workOffset": [-1., 0., 0.],
        "collisionBoxes": [
            {"name": "envelope", "center": [-1., 0., .5], "size": [2., 2.4, 1.]},
            {"name": "mount_bridge", "center": [.15, 0., .6], "size": [.4, .1, .1], "visualize": True},
        ],
    }]
    return config


@pytest.mark.parametrize("kind", ["compact_chisel_plow", "mounted_disc_harrow", "groove_maker", "mounted_cultivator", "forest_chain_crusher", "inter_row_milling_cutter", "front_sickle_bar", "garlic_planter", "mulch_planter"])
def test_implement_types_generate_matching_geometry(kind):
    config = configuration(kind)
    validate(config)
    jsonschema.validate(config, json.loads((ROOT / "schema/vehicle_config.schema.json").read_text()))
    urdf = ET.fromstring(generate_urdf(config))
    sdf = ET.fromstring(generate_sdf(config))
    name = kind + "_link"
    urdf_link = urdf.find(f"./link[@name='{name}']")
    sdf_link = sdf.find(f"./model/link[@name='{name}']")
    assert len(urdf_link.findall("collision")) == len(sdf_link.findall("collision")) == 2
    assert urdf_link.find("./visual/geometry/mesh").get("filename") == sdf_link.findtext("./visual/geometry/mesh/uri")
    assert urdf.find(f"./joint[@name='{kind}_mount_joint']").get("type") == "fixed"
    assert urdf.find(f"./link[@name='{kind}_work_frame']") is not None
    for expected, actual in zip(collision_boxes(config)[0], [-3., -1.2, .15, -1., 1.2, 1.15]):
        assert expected == pytest.approx(actual)
    fp = footprint(config)
    assert fp[2][0] == pytest.approx(-3. - config["navigation"]["clearance"]["rearM"])
    assert fp[0][1] == pytest.approx(1.2 + config["navigation"]["clearance"]["leftM"])


def test_collision_envelopes_rotate_with_mount():
    config = configuration()
    pose = config["implements"][0]["pose"]
    pose["rpy"][2] = math.pi / 2
    pose["quaternion"] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
    validate(config)
    assert collision_boxes(config)[0] == pytest.approx([-2.2, -2., .15, .2, 0., 1.15])


@pytest.mark.parametrize("fault", ["negative_box", "wrong_type", "unsafe_path", "duplicate_frame", "two_implements", "not_transport"])
def test_invalid_mounts_fail_validation(fault):
    config = configuration()
    item = config["implements"][0]
    if fault == "negative_box":
        item["collisionBoxes"][0]["size"][0] = -1
    elif fault == "wrong_type":
        item["implementType"] = "unknown"
    elif fault == "unsafe_path":
        item["meshFile"] = "../../unsafe.obj"
    elif fault == "duplicate_frame":
        config["sensors"][0]["frameId"] = item["frameId"]
    elif fault == "two_implements":
        config["implements"].append(copy.deepcopy(item))
    else:
        item["mountMode"] = "articulated_trailer"
    with pytest.raises(ConfigError):
        validate(config)


def test_mesh_and_material_are_packaged_and_hashed(tmp_path):
    config = configuration()
    filename = config["implements"][0]["meshFile"]
    material = Path(filename).with_suffix(".mtl").name
    (tmp_path / filename).write_text(f"mtllib {material}\nv -1 0 .5\nv -1 .5 .5\nv -1 0 .7\nf 1 2 3\n")
    (tmp_path / material).write_text("newmtl steel\nKd .2 .2 .2\n")
    output = tmp_path / "generated"
    manifest = generate_bundle(config, tmp_path / "vehicle_config.json", output)
    assert (output / "models" / filename).read_bytes() == (tmp_path / filename).read_bytes()
    files = {entry["path"] for entry in manifest["files"]}
    assert "models/" + filename in files
    assert "models/" + material in files
    (tmp_path / filename).write_text(f"mtllib {material}\nv 500 0 0\n")
    with pytest.raises(ConfigError, match="exceeds"):
        copy_meshes(config, tmp_path, output / "models")
    (tmp_path / filename).write_text("mtllib ../../outside.mtl\nv -1 0 .5\n")
    with pytest.raises(ConfigError, match="external material"):
        copy_meshes(config, tmp_path, output / "models")


def test_existing_sprayer_geometry_unchanged():
    config, _ = load_and_validate(BASELINE)
    config["implements"] = [default_sprayer(config)]
    validate(config)
    jsonschema.validate(config, json.loads((ROOT / "schema/vehicle_config.schema.json").read_text()))
    from agribot_vehicle_config_tools.sprayer import collision_boxes as old_boxes
    assert collision_boxes(config) == old_boxes(config)
    assert ET.fromstring(generate_sdf(config)).find("./model/link[@name='sprayer_link']") is not None
