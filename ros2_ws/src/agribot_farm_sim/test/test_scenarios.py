import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image
import pytest
import yaml

from agribot_farm_sim.scenarios import build, route
from agribot_farm_sim.scenarios import vehicle_spawn, write_georeference


def test_vehicle_specific_spawn_preserves_other_vehicles_and_scenarios():
    config = {"metadata": {"scenarioSpawnOverrides": {"arable_field": [13., 38., 0.]}}}
    original = [4, 34, 0]
    assert vehicle_spawn(config, "arable_field", original) == [13., 38., 0.]
    assert vehicle_spawn({}, "arable_field", original) == original
    assert vehicle_spawn(config, "farmyard", original) == original
    assert original == [4, 34, 0]
    config["metadata"]["scenarioSpawnOverrides"]["arable_field"] = [13., float("nan"), 0.]
    with pytest.raises(ValueError, match="finite"):
        vehicle_spawn(config, "arable_field", original)


def test_georeference_uses_rotated_antenna_at_scene_spawn(tmp_path):
    template = tmp_path / "template.yaml"
    template.write_text(yaml.safe_dump({"map": {}, "reference": {"altitude_m": 20},
                                      "calibration": {}}))
    output = tmp_path / "georeference.yaml"
    write_georeference(template, output, [10, -5, .1, math.pi / 2], [.2, .3, .7], "test")
    config = yaml.safe_load(output.read_text())
    assert config["map_from_enu"]["xyz"] == pytest.approx([9.7, -4.8, .8])
    assert config["map_from_enu"]["rpy"] == [0., 0., 0.]
    assert config["reference"]["altitude_m"] == 20


def test_unknown_scene_rejected(tmp_path):
    with pytest.raises(ValueError):
        build("../bad", tmp_path)
    assert not (tmp_path.parent / "bad").exists()


@pytest.mark.parametrize("scenario,center_x", [("strawberry_field", 6), ("vineyard", 12)])
def test_headland_has_consistent_circle_and_tangent_headings(scenario, center_x):
    points = route(scenario)
    for x, y, yaw in points[1:-1]:
        assert math.hypot(x - center_x, y) == pytest.approx(3)
        expected = math.atan2(y, x - center_x) + math.pi / 2
        assert math.atan2(math.sin(yaw - expected), math.cos(yaw - expected)) == pytest.approx(0)


def test_vineyard_preserves_original_targets_and_adds_tangent_midpoints():
    points = route("vineyard")
    assert len(points) == 7
    assert [points[index] for index in (0, 1, 3, 5, 6)] == [
        (-4, -3, 0), (12, -3, 0), (15, 0, math.pi / 2),
        (12, 3, math.pi), (-12, 3, math.pi)]


def test_vegetable_route_has_two_tangent_headlands():
    points = route("vegetable_field")
    assert len(points) == 12
    assert points[11] == (-8, 0, 0)
    for indices, cx in ((range(1, 6), 12), (range(6, 11), -12)):
        for index in indices:
            x, y, yaw = points[index]
            assert math.hypot(x - cx, y - 4.5) == pytest.approx(4.5)
            expected = math.atan2(y - 4.5, x - cx) + math.pi / 2
            assert math.atan2(math.sin(yaw - expected), math.cos(yaw - expected)) == pytest.approx(0)


def test_large_vehicle_guidance_does_not_force_small_car_turns(tmp_path):
    from agribot_farm_sim.scenarios import write_vehicle_route
    large = {"vehicleId": "large_tractor", "vehicleType": "ackermann",
             "geometry": {"minTurningRadiusM": 5.6,
                          "bodyExtents": dict(frontM=4., rearM=2., leftM=1., rightM=1.)},
             "navigation": {"clearance": dict(frontM=.1, rearM=.1, leftM=.15, rightM=.15)}}
    points = route("vegetable_field", large)
    assert len(points) == 12
    assert points[3] == pytest.approx((17.6, 5.6, math.pi / 2))
    assert points[5] == pytest.approx((12, 11.2, math.pi))
    vineyard = route("vineyard", large)
    assert len(vineyard) == 5
    assert vineyard[2] == pytest.approx((18., 3., math.pi / 2))
    assert vineyard[-2:] == [(12, 9, math.pi), (-12, 9, math.pi)]
    for x, y, yaw in vineyard[1:-1]:
        assert math.hypot(x - 12, y - 3) == pytest.approx(6.)
        expected = math.atan2(y - 3, x - 12) + math.pi / 2
        assert normalize(yaw - expected) == pytest.approx(0.)
    assert route("strawberry_field", large) == [(-3, -3, 0), (6, -3, 0),
                                                (6, 3, math.pi), (-6, 3, math.pi)]
    path = write_vehicle_route("vineyard", large, tmp_path)
    data = yaml.safe_load(path.read_text())
    assert data["routePolicy"]["minimumTurningRadiusM"] == 5.6
    assert len(data["waypoints"]) == 5
    assert data["navigationSurface"]["bounds"] == [-22., -15., 22., 15.]
    assert data["navigationSurface"]["marginM"] == .25
    assert path.name == "large_tractor_waypoints.yaml"


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def test_car2_wide_vineyard_turn_full_footprint_stays_on_ground_and_clear():
    from ament_index_python.packages import get_package_share_directory
    from agribot_vehicle_config_tools.model import footprint
    from PIL import ImageDraw
    description = Path(get_package_share_directory("agribot_vehicle_description"))
    vehicle = json.loads((description / "config/vehicles/car2_cultivator/vehicle_config.json").read_text())
    polygon = footprint(vehicle)
    asset = Path(get_package_share_directory("agribot_farm_sim")) / "assets/cropcraft_vineyard"
    image = Image.open(asset / "map.pgm")
    for index in range(361):
        yaw = math.pi * index / 360
        x, y = 12 + 6 * math.sin(yaw), 3 - 6 * math.cos(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x+c*px-s*py, y+s*px+c*py) for px, py in polygon]
        assert all(-21.75 <= px <= 21.75 and -14.75 <= py <= 14.75 for px, py in corners)
        mask = Image.new("1", image.size)
        ImageDraw.Draw(mask).polygon([((px+24)/.05, image.height-1-(py+17)/.05)
                                     for px, py in corners], fill=1)
        assert image.histogram(mask)[0] == 0


def test_vehicle_aware_routes_preserve_compact_and_other_scene_targets():
    compact = {"vehicleType": "ackermann", "geometry": {"minTurningRadiusM": 1.3,
               "bodyExtents": dict(frontM=.65, rearM=.13, leftM=.33, rightM=.33)},
               "navigation": {"clearance": dict(frontM=.1, rearM=.1, leftM=.15, rightM=.15)}}
    for scenario in ("vineyard", "strawberry_field", "arable_field", "greenhouse", "wide_headland"):
        assert route(scenario, compact) == route(scenario)


@pytest.mark.parametrize("scenario,start,license_id", [("farmyard", (0, 0, 0), "BSD-3-Clause"),
                                                     ("arable_field", (4, 34, 0), "CC-BY-SA-4.0"),
                                                     ("greenhouse", (-4, -10, 0), "MIT"),
                                                     ("tomato_greenhouse", (-7, 0, 0), "Apache-2.0"),
                                                     ("strawberry_field", (-10, -3, 0), "BSD-2-Clause"),
                                                     ("vineyard", (-15, -3, 0), "Apache-2.0"),
                                                     ("wide_headland", (-25, 0, 0), "CC-BY-SA-4.0"),
                                                     ("vegetable_field", (-14, 0, 0), "MIT")])
def test_prepared_farm_scene_has_portable_meshes_and_clear_targets(tmp_path, scenario, start, license_id):
    directory = build(scenario, tmp_path)
    info = yaml.safe_load((directory / "map.yaml").read_text())
    image = Image.open(info["image"])
    x0, y0, _ = info["origin"]
    resolution = info["resolution"]
    for x, y, _ in [start] + route(scenario):
        px, py = round((x-x0)/resolution), image.height - 1 - round((y-y0)/resolution)
        margin = round(0.8 / resolution)
        assert image.crop((px-margin, py-margin, px+margin+1, py+margin+1)).getextrema()[0] == 254
    world = ET.parse(directory / "world.sdf")
    for uri in world.findall(".//mesh/uri"):
        assert Path(uri.text.removeprefix("file://")).is_file()
    assert not world.findall(".//plugin")
    source = json.loads((directory / "geometry.json").read_text())
    assert source["license"] == license_id
    assert len(source["commit"]) == 40
    if scenario == "vegetable_field":
        assert source["commit"] == "7d8ebd6e52ade4ce8e0c89b67e2f048ec9e6989e"
        assert source["plantAssets"] and all(len(item["sha256"]) == 64 for item in source["plantAssets"])
        for family, count, low, high in (("artichoke", 10, 1.2, 1.45), ("zucchini", 18, .6, .8),
                                        ("chard", 65, .2, .3), ("lettuce", 65, .2, .3)):
            plants = [obj for obj in source["objects"] if obj["name"].startswith("vegetable_" + family + "_")]
            assert len(plants) == count
            for plant in plants:
                assert -.001 < plant["minimum"][2] < .03
                assert low < plant["maximum"][2] < high
        assert source["visualFaces"] < 600000
        assert source["map"]["maximum"] == [24, 17]
        asset = Path(info["image"]).parent
        assert "MIT License" in (asset / "LICENSE.upstream.txt").read_text()
        for line in (asset / "scene.mtl").read_text().splitlines():
            tokens = line.split()
            if tokens and tokens[0].startswith("map_"):
                texture = Path(" ".join(tokens[1:]))
                assert not texture.is_absolute() and ".." not in texture.parts
                assert (asset / texture).is_file()
    if scenario == "vineyard":
        vines = [obj for obj in source["objects"] if obj["name"].startswith("vine_")]
        assert len(vines) == 33
        assert len([obj for obj in source["objects"] if obj["name"].startswith("trellis_post_")]) == 15
        assert source["commit"] == "7128cd2acade50cc4a5a1761210b55989ab62527"
        assert source["plantAssets"] and all(len(item["sha256"]) == 64 for item in source["plantAssets"])
        for vine in vines:
            assert abs(vine["minimum"][2]) < .001
            assert 1.3 < vine["maximum"][2] < 1.9
        assert source["visualFaces"] < 150000
        asset = Path(info["image"]).parent
        for line in (asset / "scene.mtl").read_text().splitlines():
            tokens = line.split()
            if tokens and tokens[0].startswith("map_"):
                texture = Path(" ".join(tokens[1:]))
                assert not texture.is_absolute() and ".." not in texture.parts
                assert (asset / texture).is_file()
    if scenario == "strawberry_field":
        plants = [obj for obj in source["objects"] if obj["name"].startswith("strawberry_")
                  and not obj["name"].startswith("strawberry_bed_")]
        beds = [obj for obj in source["objects"] if obj["name"].startswith("strawberry_bed_")]
        assert len(plants) == 72 and len(beds) == 12
        assert source["commit"] == "14ff00da0e3c5955f8f6934171978c9523ce8003"
        for plant in plants:
            assert .4 < plant["minimum"][2] < .5
            assert .6 < plant["maximum"][2] < .8
            assert .4 < plant["maximum"][0] - plant["minimum"][0] < 1.1
        assert source["visualFaces"] < 500000
        assert len(route(scenario)) == 5
        assert len([obj for obj in source["objects"] if obj["name"].startswith("field_fence_picket_")]) > 500
        asset = Path(info["image"]).parent
        for line in (asset / "scene.mtl").read_text().splitlines():
            tokens = line.split()
            if tokens and tokens[0].startswith("map_"):
                texture = Path(" ".join(tokens[1:]))
                assert not texture.is_absolute() and ".." not in texture.parts
                assert (asset / texture).is_file()
    if scenario == "farmyard":
        assert source["map"]["projection"] == "non_terrain_geometry"
        assert any(obj["navigationSurface"] for obj in source["objects"])
    if scenario == "tomato_greenhouse":
        assert len([obj for obj in source["objects"] if obj["name"].startswith("aoc_structure_")]) == 3
        asset = Path(info["image"]).parent
        for material in asset.glob("*.mtl"):
            for line in material.read_text().splitlines():
                tokens = line.split()
                if tokens and tokens[0].startswith("map_"):
                    texture = Path(" ".join(tokens[1:]))
                    assert not texture.is_absolute() and ".." not in texture.parts
                    assert (asset / texture).is_file(), texture
        pots = [obj for obj in source["objects"] if obj["name"].startswith("aoc_flowerpot_")]
        assert len(pots) == 38
        for pot in pots:
            assert .1 < pot["maximum"][0] - pot["minimum"][0] < .3
            assert .15 < pot["minimum"][2] < .2
            assert .2 < pot["maximum"][2] < .3
        assert source["map"]["maximum"] == [20, 14]
