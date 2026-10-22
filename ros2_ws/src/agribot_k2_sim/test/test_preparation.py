import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
import yaml

from agribot_k2_sim.prepare import append_gaussian_visual, append_vehicle_light, project_map


def test_original_gaussian_plugin_has_no_mesh_visual_proxy(tmp_path):
    link = ET.Element("link")
    visual = append_gaussian_visual(link, tmp_path)
    assert len(link.findall("visual")) == 1
    assert visual.find("geometry/mesh") is None
    assert visual.findtext("geometry/box/size") == ".000001 .000001 .000001"
    assert visual.find("plugin").get("filename") == str(tmp_path / "libK2GaussianVisual.so")
    assert visual.findtext("plugin/directory") == str(tmp_path)


def test_ordinary_vehicle_has_directional_illumination_without_shadows():
    world = ET.Element("world")
    light = append_vehicle_light(world)
    assert light.get("type") == "directional"
    assert light.findtext("cast_shadows") == "false"
    assert float(light.findtext("direction").split()[2]) < 0


def test_map_uses_scanned_ground_and_keeps_unobserved_unknown(tmp_path):
    vertices = np.array([[0., 0, -2], [2, 0, -2], [0, 2, -2],
                         [1, 0, -2], [1, 1, -2], [1, 1, -1]])
    faces = np.array([[0, 1, 2], [3, 4, 5]])
    report = project_map([(vertices, faces)], -2, tmp_path)
    grid = np.flipud(np.array(Image.open(tmp_path / "map.pgm")))
    config = yaml.safe_load((tmp_path / "map.yaml").read_text())
    assert config["origin"] == [-1., -1., 0.]
    assert grid[12, 12] == 254
    assert grid[15, 20] == 0
    assert grid[0, 0] == 205
    assert report["unknown_outside_observed_ground"]


@pytest.fixture
def external_loader():
    module_file = Path(__file__).parents[2] / "agribot_vehicle_description/launch/configured_ackermann_sim.launch.py"
    spec = importlib.util.spec_from_file_location("k2_configured_test", module_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._external_scene_bundle


def scene_manifest(tmp_path):
    for name in ("world.sdf", "map.yaml", "waypoints.yaml"):
        (tmp_path / name).touch()
    return {"schema_version": 1, "world": "world.sdf", "map": "map.yaml",
            "waypoints": "waypoints.yaml", "spawn": [.8, -17.5, math.pi / 2],
            "spawn_ground_z": 0.0}


def test_external_scene_loader_preserves_explicit_spawn(tmp_path, external_loader):
    manifest = scene_manifest(tmp_path)
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(manifest))
    result = external_loader(path)
    assert result["spawn"] == manifest["spawn"]
    assert result["world"] == str(tmp_path / "world.sdf")
    assert result["ground"] == 0.


@pytest.mark.parametrize("change", [{"spawn": [0, 0, float("nan")]},
                                   {"spawn_ground_z": True}, {"world": "../world.sdf"},
                                   {"schema_version": 2}])
def test_external_scene_rejects_bad_manifest(tmp_path, external_loader, change):
    manifest = scene_manifest(tmp_path)
    manifest.update(change)
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        external_loader(path)
