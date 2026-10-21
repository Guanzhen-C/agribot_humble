import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_farm_sim.ground import (
    _split_collision_mesh, is_navigation_surface, prepare_orchard_ground,
)
from agribot_farm_sim.scenarios import PREPARED_SCENES, build


def test_old_greenhouse_floor_tags_do_not_include_crops_or_structure():
    for name in ("greenhouse_base_link", "greenhouse_greenhouse_link",
                 "greenhouse_grass_link.003", "greenhouse_pathway_link.012"):
        assert is_navigation_surface({"name": name}, "greenhouse")
    for name in ("greenhouse_stems_link", "greenhouse_front_link",
                 "greenhouse_roof_link", "greenhouse_orange_stem_link"):
        assert not is_navigation_surface({"name": name}, "greenhouse")
    assert not is_navigation_surface({"name": "greenhouse_grass_link"}, "farmyard")


def test_split_preserves_coordinates_indices_and_unknown_obstacles(tmp_path):
    mesh = tmp_path / "combined.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\no Ground_collision_Plane\n"
                    "f 1 2 3\no UnlistedObstacle\nf -3 -2 -1\n")
    outputs = {key: tmp_path / (key + ".obj") for key in ("terrain", "obstacles")}
    assert _split_collision_mesh(mesh, outputs, {"Ground": "terrain"}) == {
        "terrain": 1, "obstacles": 1}
    assert "f 1 2 3" in outputs["terrain"].read_text()
    assert "f -3 -2 -1" in outputs["obstacles"].read_text()
    assert "v 0 0 0\nv 1 0 0\nv 0 1 0\n" in outputs["terrain"].read_text()
    with pytest.raises(ValueError, match="missing"):
        _split_collision_mesh(mesh, outputs, {"MissingGround": "slab"})


@pytest.mark.parametrize("scene", PREPARED_SCENES)
def test_every_prepared_scene_separates_ground_and_retains_obstacles(tmp_path, scene):
    from ament_index_python.packages import get_package_share_directory
    asset = Path(get_package_share_directory("agribot_farm_sim")) / "assets" / PREPARED_SCENES[scene]
    original = ET.parse(asset / "world.sdf")
    original_visuals = [ET.tostring(item) for item in original.findall(".//visual")]
    directory = build(scene, tmp_path)
    world = ET.parse(directory / "world.sdf")
    policy = json.loads((directory / "geometry.json").read_text())["groundPolicy"]
    assert policy["obstacleFaces"] > 0
    assert policy["materialIndependent"]
    for collision in world.findall(".//collision"):
        name = collision.get("name")
        if name == "obstacles_collision":
            assert collision.find("surface") is None
            continue
        assert float(collision.findtext("surface/friction/ode/mu")) == 100
        assert float(collision.findtext("surface/contact/ode/kp")) == 1e9
        if name.startswith("drivable_ground_"):
            assert float(collision.findtext("geometry/box/size").split()[2]) == .5
    if scene == "farmyard":
        assert policy["solidFlatPatches"] == 0
        assert policy["unevenTerrainFaces"] > 0
    else:
        assert policy["solidFlatPatches"] > 0
    # Restoring the original visual URI must make the entire visual identical.
    for visual in world.findall(".//visual"):
        visual.find("geometry/mesh/uri").text = "scene.obj"
    assert [ET.tostring(item) for item in world.findall(".//visual")] == original_visuals
    assert (directory / "obstacles_collision.obj").is_file()


def test_orchard_retains_terrain_and_trunk_geometry(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    world = Path(get_package_share_directory("scout_gazebo")) / "worlds/orchard_barriers.world"
    before = ET.parse(world)
    path = prepare_orchard_ground(world, tmp_path)
    after = ET.parse(path)
    for name in ("ground_collision", "trunks_collision"):
        query = f".//collision[@name='{name}']/geometry"
        assert ET.tostring(after.find(query)) == ET.tostring(before.find(query))
    assert after.findtext(".//collision[@name='ground_collision']/surface/friction/ode/mu2") == "100"


def test_greenhouse_grass_and_path_top_remain_at_original_height(tmp_path):
    directory = build("greenhouse", tmp_path)
    source = json.loads((directory / "geometry.json").read_text())
    flat = [item for item in source["objects"] if is_navigation_surface(item, "greenhouse")]
    world = ET.parse(directory / "world.sdf")
    for index, item in enumerate(flat):
        node = world.find(f".//collision[@name='drivable_ground_{index}']")
        pose = [float(v) for v in node.findtext("pose").split()]
        size = [float(v) for v in node.findtext("geometry/box/size").split()]
        assert pose[2] + size[2] / 2 == pytest.approx(item["maximum"][2])
        assert size[:2] == pytest.approx([item["maximum"][i] - item["minimum"][i] for i in (0, 1)])
