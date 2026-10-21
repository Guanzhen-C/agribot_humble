import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.generator import (
    _simplify_obj_vertex_clusters, generate_sdf, generate_simulation_sdf, generate_urdf,
)
from agribot_vehicle_config_tools.visual_rotors import copy_assets, rotors


BASELINE = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/config/vehicles/harvester_header/vehicle_config.json"


def configuration():
    config = json.loads(BASELINE.read_text())
    config["metadata"]["simulationVisualRotors"] = [dict(
        id="rear_left", sourceWheel="rear_left", center=[-1.729, 1.645, .005],
        ratio=1.0, meshFile="rotor_rear_left.obj")]
    return config


def test_animation_does_not_change_physics_or_existing_joints():
    config = configuration()
    before = copy.deepcopy(config)
    before["metadata"].pop("simulationVisualRotors")
    old = ET.fromstring(generate_sdf(before))
    new = ET.fromstring(generate_sdf(config))
    base = new.find("model/link[@name='base_link']")
    base.remove(base.find("visual[@name='display_rear_left']"))
    ET.indent(old, space="  ")
    ET.indent(new, space="  ")
    assert ET.tostring(old) == ET.tostring(new)


def test_urdf_has_mimic_without_contact_or_mass():
    robot = ET.fromstring(generate_urdf(configuration()))
    link = robot.find("link[@name='display_rear_left_link']")
    assert link.find("collision") is None and link.find("inertial") is None
    joint = robot.find("joint[@name='display_rear_left_joint']")
    assert joint.find("mimic").get("joint") == "rear_left_wheel_joint"
    assert joint.find("origin").get("xyz") == "-1.729 1.645 0.005"


def test_runtime_template_also_gets_the_rotors(tmp_path):
    model = ET.fromstring(generate_simulation_sdf(configuration(), BASELINE.parents[4], tmp_path)).find("model")
    visual = model.find("link[@name='base_link']/visual[@name='display_rear_left']")
    assert visual is not None
    assert visual.findtext("plugin/source_link") == "rear_left_wheel_link"
    assert model.find("link[@name='display_rear_left_link']") is None


@pytest.mark.parametrize("field,value", [("meshFile", "../escape.obj"), ("sourceWheel", "missing"),
                                          ("ratio", float("nan")), ("center", [0, 1])])
def test_bad_rotors_are_rejected(field, value):
    config = configuration()
    config["metadata"]["simulationVisualRotors"][0][field] = value
    with pytest.raises(ValueError):
        rotors(config)


def test_texture_and_rotor_are_copied_and_external_paths_rejected(tmp_path):
    source, output = tmp_path / "source", tmp_path / "output"
    source.mkdir()
    output.mkdir()
    (source / "rotor_rear_left.obj").write_text("mesh")
    (source / "paint.png").write_bytes(b"texture")
    (source / "vehicle_visual.mtl").write_text("map_Kd paint.png\n")
    copy_assets(configuration(), source, output)
    assert (output / "paint.png").read_bytes() == b"texture"
    (source / "vehicle_visual.mtl").write_text("map_Kd ../paint.png\n")
    with pytest.raises(ValueError, match="export-local"):
        copy_assets(configuration(), source, output)


def test_lod_preserves_texture_coordinates_and_uv_seams(tmp_path):
    path = tmp_path / "visual.obj"
    path.write_text("mtllib paint.mtl\nusemtl paint\n"
                    "v 0 0 0\nv 1 0 0\nv 0 1 0\n"
                    "vn 0 0 1\nvt 0 0\nvt 1 0\nvt 0 1\nvt 0.5 0.5\n"
                    "f 1/1/1 2/2/1 3/3/1\nf 1/4/1 2/2/1 3/3/1\n")
    report = _simplify_obj_vertex_clusters(path, .02)
    assert report["outputFaces"] == 2
    assert report["outputVertices"] == 4
    lines = path.read_text().splitlines()
    assert sum(line.startswith("vt ") for line in lines) == 4
    assert all("//" not in line for line in lines if line.startswith("f "))
