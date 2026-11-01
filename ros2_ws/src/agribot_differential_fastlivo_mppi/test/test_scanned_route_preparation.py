import importlib.util
import json
from pathlib import Path
import struct
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


def module():
    path = Path(__file__).resolve().parents[1]/"scripts/prepare_scanned_route_validation.py"
    spec = importlib.util.spec_from_file_location("scanned_route_preparation", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def stl(path, vertices):
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    data = np.zeros(len(vertices), dtype=dtype)
    data["vertices"] = vertices
    path.write_bytes(bytes(80)+struct.pack("<I", len(data))+data.tobytes())


def test_binary_stl_and_vertical_ground_intersections(tmp_path):
    m = module()
    path = tmp_path/"ground.stl"
    stl(path, [[[0, 0, 1], [2, 0, 1], [0, 2, 2]], [[0, 0, 3], [2, 0, 3], [0, 2, 3]]])
    triangles = m.stl_triangles(path)
    assert triangles.shape == (2, 3, 3)
    assert m.surface_heights(triangles, [.5, .5]).tolist() == [1.25, 3.]
    assert len(m.surface_heights(triangles, [4, 4])) == 0
    path.write_bytes(path.read_bytes()+b"bad")
    with pytest.raises(ValueError, match="binary STL"):
        m.stl_triangles(path)


def test_preparation_preserves_source_path_map_and_scene(tmp_path, monkeypatch):
    m = module()
    scene = tmp_path/"scan"
    (scene/"source").mkdir(parents=True)
    (scene/"meshes").mkdir()
    (scene/"source/input.lcc2").write_text(json.dumps({"offset": [0, 0, 0], "epsg": 32650, "shift": [0, 0, 0], "scale": [1, 1, 1]}))
    (scene/"scene.json").write_text(json.dumps({"world": "world.sdf"}))
    (scene/"world.sdf").write_text('<sdf><world><model name="k2_automation_sensor_scene"><pose>0 0 0 0 0 0</pose><link><collision name="original_obstacle"/></link></model></world></sdf>')
    stl(scene/"meshes/collision_00.stl", [[[-2, -2, .1], [200, -2, .1], [-2, 200, .1]]])
    base = tmp_path/"real"
    base.with_suffix(".pgm").write_bytes(b"test map")
    base.with_suffix(".yaml").write_text("image: real.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    geo = Path(str(base)+"_georeference.yaml")
    geo.write_text("reference: {}\nmap_from_enu: {xyz: [0, 0, 0], rpy: [0, 0, 0]}\n")
    route = {"id": "manual104_saved", "path": [{"x": float(i), "y": 0., "yaw": 0.} for i in range(104)], "anchor_path_indices": list(range(104))}
    route_file = Path(str(base)+"_saved_paths.json")
    route_file.write_text(json.dumps({"routes": [route], "fingerprints": {key: m.sha256(file) for key, file in (("pgm", base.with_suffix(".pgm")), ("yaml", base.with_suffix(".yaml")), ("georeference", geo))}}))
    params = tmp_path/"nav2.yaml"
    params.write_text(yaml.safe_dump({"controller_server": {"ros__parameters": {"use_sim_time": False, "odom_topic": "/fastlivo_rtk/odometry", "controller_frequency": 5., "goal_checker": {"xy_goal_tolerance": .25}, "FollowPath": {"plugin": "mppi", "wz_std": .08, "PathAngleCritic": {"forward_preference": False, "cost_weight": 2.}}}},
                                     "planner_server": {"ros__parameters": {"GridBased": {"plugin": "lattice"}}},
                                     **{name: {name: {"ros__parameters": {"footprint": "old"}}}
                                        for name in ("global_costmap", "local_costmap")}}))
    overlay = tmp_path/"overlay.yaml"
    overlay.write_text("FollowPath:\n  wz_std: 0.24\n  PathAngleCritic:\n    forward_preference: true\n")
    vehicle = tmp_path/"vehicle.json"
    vehicle.write_text(json.dumps({"geometry": {"wheelbaseM": .8, "bodyExtents":
                                               {"frontM": .6, "rearM": .5, "leftM": .38, "rightM": .38}}}))
    class Frame:
        def projected(self, points): return points
    monkeypatch.setattr(m, "projected_to_enu", lambda *a: (0., np.zeros(2), Frame()))
    protected = [scene/"world.sdf", base.with_suffix(".pgm"), route_file, params]
    before = [p.read_bytes() for p in protected]
    out = tmp_path/"validation"
    args = SimpleNamespace(scene_manifest=scene/"scene.json", map_base=base, output=out, nav2_params=params,
                           route_id=route["id"], nav2_overlay=overlay, vehicle_config=vehicle)
    m.prepare(args)
    assert json.loads((out/"saved_route.json").read_text()) == route
    assert [p.read_bytes() for p in protected] == before
    assert ET.parse(out/"world.sdf").find(".//collision[@name='original_obstacle']") is not None
    generated = yaml.safe_load((out/"nav2_params.yaml").read_text())
    assert generated["controller_server"]["ros__parameters"]["controller_frequency"] == 5.
    assert generated["controller_server"]["ros__parameters"]["use_sim_time"] is True
    controller = generated["controller_server"]["ros__parameters"]
    assert controller["goal_checker"]["xy_goal_tolerance"] == .4
    assert controller["FollowPath"]["plugin"] == "mppi"
    assert controller["FollowPath"]["wz_std"] == .24
    assert controller["FollowPath"]["PathAngleCritic"] == {"forward_preference": True, "cost_weight": 2.}
    expected_footprint = [[.65, .43], [.65, -.43], [-.55, -.43], [-.55, .43]]
    for name in ("global_costmap", "local_costmap"):
        assert np.asarray(json.loads(generated[name][name]["ros__parameters"]["footprint"])) == pytest.approx(np.asarray(expected_footprint))
    report = json.loads((out/"preparation.json").read_text())
    assert np.asarray(report["navigation_footprint"]) == pytest.approx(np.asarray(expected_footprint))
    assert report["simulation_clearance_m"] == .05
    assert json.loads((out/"scene.json").read_text())["spawn_ground_z"] == pytest.approx(.1)
    with pytest.raises(ValueError, match="immutable"):
        m.prepare(args)


def test_navigation_uses_estimator_without_ground_truth_pose_override():
    # Production path sampling is never replaced by a shorter fixture route.
    source = (Path(__file__).resolve().parents[1]/"scripts/run_scanned_saved_route.py").read_text()
    assert 'self.runner.start(self.path,' in source
    assert 'self.create_subscription(Odometry, "/odometry/filtered_navsat"' in source
    assert 'TransformBroadcaster' not in source
    assert 'SetEntityState' not in source
    assert 'self.full_run and not self.physical_failure and len(self.anchors) == 104' in source


def test_continuous_ground_fills_holes_below_observed_grade():
    import sys
    scripts = Path(__file__).resolve().parents[1]/"scripts"
    sys.path.insert(0, str(scripts))
    from add_scanned_ground_support import lower_terrain
    x, y = np.meshgrid(np.arange(0, 10, .25), np.arange(0, 10, .25))
    vertices = np.column_stack((x.ravel(), y.ravel(), (x*.02).ravel()))
    hole = np.linalg.norm(vertices[:, :2]-[5, 5], axis=1) < .75
    triangles, report = lower_terrain(vertices[~hole], np.array([0., 0.]), np.array([10., 10.]))
    assert report["filled_cells"] > 0
    assert np.isfinite(triangles).all()
    m = module()
    for point in ([0, 0], [5, 5], [9.9, 9.9]):
        assert len(m.surface_heights(triangles, point)) > 0
    assert m.surface_heights(triangles, [5, 5])[-1] <= .1


def test_raised_access_road_is_preserved_as_terrain():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from add_scanned_ground_support import lower_terrain
    x, y = np.meshgrid(np.arange(0, 10, .25), np.arange(0, 10, .25))
    z = ((y >= 4) & (y <= 6)).astype(float)
    triangles, unused = lower_terrain(np.column_stack((x.ravel(), y.ravel(), z.ravel())),
                                     np.array([0., 0.]), np.array([10., 10.]))
    assert module().surface_heights(triangles, [5, 5])[-1] > .8


def test_contact_proxy_only_replaces_low_geometry_in_verified_free_cells():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import retained_contacts
    image = np.full((100, 100), 254, dtype=np.uint8)
    geometry = {"resolution": .1, "origin": [0, 0, 0]}
    ground = np.zeros((21, 21))
    low = np.array([[1, 1, .1], [2, 1, .2], [1, 2, .3]])
    high = low + [0, 0, 1]
    outside = low - [5, 0, 0]
    assert retained_contacts(np.array([low, high, outside]), image, geometry, ground,
                             np.zeros(2), .5).tolist() == [False, True, True]
    # Even a tiny mapped obstacle inside a large triangle bbox is retained.
    image[15, 15] = 0
    assert retained_contacts(np.array([low]), image, geometry, ground,
                             np.zeros(2), .5).tolist() == [True]
    image[15, 15] = 205
    assert retained_contacts(np.array([low]), image, geometry, ground,
                             np.zeros(2), .5).tolist() == [True]


def test_contact_proxy_uses_grade_not_absolute_world_height():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import retained_contacts
    image = np.full((100, 100), 254, dtype=np.uint8)
    ground = np.full((21, 21), 3.)
    low = np.array([[1, 1, 3.1], [2, 1, 3.2], [1, 2, 3.3]])
    keep = retained_contacts(np.array([low, low+[0, 0, .5]]), image,
                             {"resolution": .1, "origin": [0, 0, 0]}, ground,
                             np.zeros(2), .5)
    assert keep.tolist() == [False, True]


def test_verified_free_vehicle_envelope_keeps_mapped_obstacles_and_overhead_geometry():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import ContactMask, refined_contacts
    image = np.full((100, 100), 255, dtype=np.uint8)
    geometry = {"resolution": .05, "origin": [0, 0, 0]}
    low = np.array([[[1, 1, .55], [1.2, 1, .6], [1, 1.2, .58]]])
    mask = ContactMask(image, geometry, np.zeros((11, 11)), np.zeros(2), .5, height_limit=.95)
    assert len(refined_contacts(low, mask)[0]) == 0
    assert len(refined_contacts(low+[0, 0, 1], mask)[0]) == 1
    image[20:25, 20:25] = 0
    mask = ContactMask(image, geometry, np.zeros((11, 11)), np.zeros(2), .5, height_limit=.95)
    assert len(refined_contacts(low, mask)[0]) > 0


def test_support_grid_reconstruction_preserves_mesh_heights():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from add_scanned_ground_support import lower_terrain
    from prepare_scanned_contact_proxy import terrain_grid
    x, y = np.meshgrid(np.arange(0, 3, .25), np.arange(0, 3, .25))
    triangles, stats = lower_terrain(np.column_stack((x.ravel(), y.ravel(), (x*.02).ravel())),
                                    np.zeros(2), np.array([3., 3.]))
    origin, grid = terrain_grid(triangles, stats)
    assert origin.tolist() == [0., 0.]
    assert grid.shape == (stats["height"], stats["width"])
    assert np.isfinite(grid).all()
    assert grid[2, 2] == pytest.approx(module().surface_heights(triangles, [1, 1])[-1])


def test_heightfield_has_correct_extent_orientation_and_bounded_heights(tmp_path):
    import sys
    from PIL import Image
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from add_scanned_ground_support import heightfield_geometry
    x, y = np.meshgrid(np.arange(4)*.5+2, np.arange(6)*.5-3)
    points = np.stack((x, y, .1*x+.2*y), axis=-1)
    a, b, c, d = points[:-1, :-1], points[:-1, 1:], points[1:, :-1], points[1:, 1:]
    triangles = np.concatenate([np.stack(face, axis=2).reshape(-1, 3, 3)
                                for face in ((a, b, c), (b, d, c))])
    stats = {"width": 4, "height": 6, "cell_m": .5}
    geometry, pose = heightfield_geometry(tmp_path, triangles, stats)
    field = geometry.find("heightmap")
    size = np.array(list(map(float, field.findtext("size").split())))
    assert size[:2] == pytest.approx([1.5, 2.5])
    assert list(map(float, pose.split()))[:3] == pytest.approx([2.75, -1.75, 0])
    image = np.asarray(Image.open(tmp_path/"ground_support_height.png"))
    assert image.shape == (9, 9) and image.dtype == np.uint8
    assert image[0, 0] > image[-1, 0]  # North is the first image row.
    decoded = image[::-1]/255*size[2]+float(field.findtext("pos").split()[2])
    expected = .1*np.linspace(2, 3.5, 9)[None, :]+.2*np.linspace(-3, -.5, 9)[:, None]
    assert np.max(abs(decoded-expected)) <= stats["heightfield_quantization_bound_m"]+1e-12
    assert stats["collision_representation"] == "ODE heightfield"


def test_native_track_command_bounds_acceleration_braking_and_belt_speed(tmp_path):
    import subprocess
    source = tmp_path/"track_limits.cpp"
    source.write_text('''
#include <cassert>
#include "track_command_limits.hpp"
using namespace agribot_differential_fastlivo_mppi;
int main() {
  TrackCommand state;
  for (auto target : {TrackCommand{.5, 0}, TrackCommand{0, .8},
                       TrackCommand{-.5, -.5}, TrackCommand{0, 0}}) {
    for (int step=0; step<2000; ++step) {
      const auto next = limitedTrackCommand(state, target, .002, .5, .8, .3006, .36, .78);
      assert(std::abs(next.linear-state.linear) <= .001+1e-10);
      assert(std::abs(next.angular-state.angular) <= .0016+1e-10);
      assert(std::abs(next.linear)+std::abs(next.angular)*.3006/.36 <= .78+1e-10);
      state=next;
    }
  }
  assert(std::abs(state.linear)<1e-10 && std::abs(state.angular)<1e-10);
  auto unchanged=limitedTrackCommand(state, {.5, .5}, -1, .5, .8, .3006, .36, .78);
  assert(unchanged.linear==state.linear && unchanged.angular==state.angular);
  auto bounded=limitedTrackCommand(state, {.5, .5}, 10, .5, .8, .3006, .36, .78);
  assert(bounded.linear<=.01 && bounded.angular<=.016);
}
''')
    executable = tmp_path/"track_limits"
    include = Path(__file__).resolve().parents[1]/"include"
    subprocess.run(["g++", "-std=c++17", "-I", str(include), str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_native_query_layers_retain_ground_contact_and_full_source_lidar(tmp_path):
    import subprocess
    source = tmp_path/"query_layers.cpp"
    source.write_text('''
#include <cassert>
#include <initializer_list>
#include <gazebo/ode/ode.h>
#include "scanned_collision_layers.hpp"
using namespace agribot_differential_fastlivo_mppi;
int hits=0;
void contact(void*, dGeomID, dGeomID) { ++hits; }
int main() {
  dInitODE();
  auto a=dSimpleSpaceCreate(nullptr), b=dSimpleSpaceCreate(nullptr);
  auto first=dCreateSphere(a, 1), second=dCreateSphere(b, 1);
  auto pair=[&](unsigned c1, unsigned m1, unsigned c2, unsigned m2) {
    dGeomSetCategoryBits(first,c1); dGeomSetCollideBits(first,m1);
    dGeomSetCategoryBits(second,c2); dGeomSetCollideBits(second,m2);
    hits=0; dSpaceCollide2((dGeomID)a,(dGeomID)b,nullptr,contact);
    return hits!=0;
  };
  assert(!pair(kScannedRayCategory,kScannedRayMask,kScannedRayCategory,kScannedPhysicalMask));
  for (unsigned native : {0x10000000u,0x30000000u,0x70000000u}) {
    assert(!pair(kScannedRayCategory,kScannedRayMask,native|kScannedVehicleCategory,kScannedPhysicalMask));
    assert(pair(native,1,kScannedRayCategory,kScannedPhysicalMask));
  }
  assert(pair(kScannedRayCategory,kScannedRayMask,1,~1u));
  assert(pair(kScannedVehicleCategory,1,kScannedRayCategory,kScannedPhysicalMask));
  assert(pair(kScannedVehicleCategory,1,1,~1u));
  assert(!pair(kScannedVehicleCategory,1,kScannedVehicleCategory,1));
  dSpaceDestroy(a); dSpaceDestroy(b); dCloseODE();
}
''')
    executable = tmp_path/"query_layers"
    include = Path(__file__).resolve().parents[1]/"include"
    subprocess.run(["g++", "-std=c++17", "-I", str(include), "-I/usr/include/gazebo-11",
                    str(source), "-lgazebo_ode", "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_refinement_removes_free_low_fragment_without_erasing_mapped_obstacle():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import ContactMask, refined_contacts
    image = np.full((100, 100), 254, dtype=np.uint8)
    image[:, :5] = 0
    # One coarse face crosses both a mapped wall and verified free ground.
    coarse = np.array([[[.1, .5, .2], [1.5, .5, .2], [.1, 2., .2]]])
    mask = ContactMask(image, {"resolution": .05, "origin": [0, 0, 0]},
                       np.zeros((11, 11)), np.zeros(2), .5)
    assert mask.classify(coarse)[0].tolist() == [True]
    refined, removed = refined_contacts(coarse, mask)
    assert removed > 0
    assert len(module().surface_heights(refined, [.15, .7])) > 0
    assert len(module().surface_heights(refined, [.8, .7])) == 0
    high, removed_high = refined_contacts(coarse+[0, 0, 1], mask)
    assert len(high) == 1 and removed_high == 0


def test_manually_edited_near_white_pixels_match_nav2_free_space():
    m = module()
    values = np.array([[0, 80, 128, 205, 206, 248, 254, 255]], dtype=np.uint8)
    geometry = {"free_thresh": .196, "occupied_thresh": .65, "negate": 0}
    assert m.nav2_free_pixels(values, geometry).tolist() == [[False, False, False, False, True, True, True, True]]
    assert m.nav2_free_pixels(255-values, {**geometry, "negate": 1}).tolist() == m.nav2_free_pixels(values, geometry).tolist()
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import ContactMask
    image = np.full((100, 100), 248, dtype=np.uint8)
    mask = ContactMask(image, {**geometry, "resolution": .05, "origin": [0, 0, 0]},
                       np.zeros((11, 11)), np.zeros(2), .5)
    low = np.array([[[1, 1, .2], [2, 1, .2], [1, 2, .2]]])
    assert mask.classify(low)[0].tolist() == [False]


def test_unknown_ground_policy_matches_planner_without_erasing_obstacles():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
    from prepare_scanned_contact_proxy import ContactMask
    image = np.full((100, 100), 205, dtype=np.uint8)
    geometry = {"resolution": .05, "origin": [0, 0, 0]}
    low = np.array([[[1, 1, .2], [2, 1, .2], [1, 2, .2]]])
    args = image, geometry, np.zeros((11, 11)), np.zeros(2), .5
    assert ContactMask(*args).classify(low)[0].tolist() == [True]
    assert ContactMask(*args, allow_unknown=True).classify(low)[0].tolist() == [False]
    assert ContactMask(*args, allow_unknown=True).classify(low+[0, 0, 1])[0].tolist() == [True]
    image[25, 25] = 0
    assert ContactMask(*args, allow_unknown=True).classify(low)[0].tolist() == [True]
