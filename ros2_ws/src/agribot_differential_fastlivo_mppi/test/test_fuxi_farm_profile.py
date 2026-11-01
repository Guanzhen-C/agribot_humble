import importlib.util
import json
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("fuxi_profile", PACKAGE / "scripts/fuxi_farm_profile.py")
profile = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(profile)


def vehicle():
    return {"vehicleId": "unity_car3", "vehicleType": "tracked", "coordinateConvention": "FLU",
            "baseFrame": "base_link", "geometry": {"wheelbaseM": .837003}, "navigation": {},
            "chassis": {}, "dynamics": {}, "collision": {},
            "sensors": [{"id": "imu", "pose": {"xyz": [.4, .1, .6]}}], "manipulators": []}


@pytest.fixture
def prepared(tmp_path):
    root = tmp_path / "profile"
    template = root / "template"
    reference = template / "reference"
    reference.mkdir(parents=True)
    workspace = tmp_path / "workspace"
    generated = workspace / "src/agribot_vehicle_description/generated/unity_car3"
    generated.mkdir(parents=True)
    config = generated / "vehicle_config.json"
    profile.write_json(config, vehicle())
    profile.write_json(reference / "vehicle_config.json", vehicle())
    route = {"path": [{"x": i * .01, "y": 0., "yaw": 0.} for i in range(11285)],
             "anchor_path_indices": [round(i * 11284 / 103) for i in range(104)]}
    profile.write_json(template / "saved_route.json", route)
    (template / "scene.json").write_text('{"world":"world.sdf"}')
    (template / "world.sdf").write_text("<sdf/>")
    (template / "ground_support.stl").write_bytes(b"immutable terrain")
    values = {"schema_version": 1, "id": profile.PROFILE_ID, "name": "伏羲农场",
              "vehicle_id": profile.VEHICLE_ID, "algorithm_id": profile.ALGORITHM, "template": "template",
              "route_sha256": profile.digest(template / "saved_route.json"),
              "template_hashes": {str(p.relative_to(root)): profile.digest(p)
                                  for p in template.rglob("*") if p.is_file()},
              "vehicle_hashes": {"vehicle_config.json": profile.digest(config)}}
    manifest = root / "profile.json"
    profile.write_json(manifest, values)
    return manifest, config, workspace, tmp_path / "runs"


def test_new_runs_preserve_exact_route_and_never_inherit_success(prepared):
    manifest, config, workspace, runs = prepared
    first = profile.prepare_run(manifest, config, workspace, runs)
    profile.write_json(first / "result.json", {"passed": True})
    second = profile.prepare_run(manifest, config, workspace, runs)
    assert first != second
    assert not (second / "result.json").exists()
    assert (second / "saved_route.json").read_bytes() == (manifest.parent / "template/saved_route.json").read_bytes()
    assert (second / "ground_support.stl").is_symlink()
    assert profile.read_json(second / "software_launch.json")["source"] == "Unity simulation launcher"


def test_changed_route_is_rejected_before_creating_run(prepared):
    manifest, config, workspace, runs = prepared
    route = manifest.parent / "template/saved_route.json"
    route.write_text(route.read_text() + " ")
    with pytest.raises(ValueError, match="input changed"):
        profile.prepare_run(manifest, config, workspace, runs)
    assert not runs.exists()


@pytest.mark.parametrize("key,value", [("vehicleId", "unity_car0"), ("vehicleType", "ackermann"),
                                     ("implements", [{"id": "plow"}])])
def test_other_vehicle_or_implement_is_not_silently_substituted(key, value):
    actual = {**vehicle(), key: value}
    with pytest.raises(ValueError):
        profile.validate_vehicle(actual, vehicle())


def test_different_sensor_pose_is_rejected_but_float_export_roundoff_is_allowed():
    actual = vehicle()
    actual["sensors"][0]["pose"]["xyz"][0] += 1e-7
    profile.validate_vehicle(actual, vehicle())
    actual["sensors"][0]["pose"]["xyz"][0] += .01
    with pytest.raises(ValueError, match="sensors"):
        profile.validate_vehicle(actual, vehicle())


def test_incomplete_or_out_of_order_routes_are_rejected(prepared):
    manifest, *_ = prepared
    route = profile.read_json(manifest.parent / "template/saved_route.json")
    route["anchor_path_indices"] = route["anchor_path_indices"][::-1]
    with pytest.raises(ValueError, match="order"):
        profile.check_route(route)
    route["anchor_path_indices"].pop()
    with pytest.raises(ValueError, match="104"):
        profile.check_route(route)


def test_changed_installed_model_is_rejected(prepared):
    manifest, config, workspace, runs = prepared
    actual = profile.read_json(config)
    actual["displayName"] = "changed source"
    profile.write_json(config, actual)
    with pytest.raises(ValueError, match="Car3 model changed"):
        profile.prepare_run(manifest, config, workspace, runs)


def test_standalone_runtime_is_exact_production_snapshot():
    source = PACKAGE.parent / "agribot_mobile_app/agribot_mobile_app"
    vendored = PACKAGE / "agribot_differential_fastlivo_mppi/saved_route_runtime"
    if not source.is_dir():
        snapshot = json.loads((vendored / "snapshot.json").read_text())
        assert all(profile.digest(vendored / file) == sha for file, sha in snapshot.items())
    else:
        for name in ("catalog.py", "navigation_health.py", "saved_path.py", "saved_path_runner.py", "route_preflight.py"):
            assert (source / name).read_bytes() == (vendored / name).read_bytes()


def test_smac_reference_check_requires_every_original_coordinate_and_heading():
    points = [{"x": 1., "y": 2., "yaw": .2}, {"x": 2., "y": 3., "yaw": .4}]
    assert profile.compare_route_poses(points, points)["exactly_equal"] is True
    changed = [dict(point) for point in points]
    changed[-1]["y"] += 1e-10
    result = profile.compare_route_poses(points, changed)
    assert result["exactly_equal"] is False
    assert result["max_absolute_difference"]["y"] > 0
    assert profile.compare_route_poses(points, points[:-1])["exactly_equal"] is False


def test_private_planner_costmap_uses_shared_map_without_changing_smac_parameters(tmp_path):
    import yaml
    spec = importlib.util.spec_from_file_location("fuxi_launch", PACKAGE / "launch/scanned_saved_route_validation.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    parameters = {"fuxi_route": {
        "planner_server": {"ros__parameters": {"GridBased": {"cost_penalty": 2., "rotation_penalty": 2.}}},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "use_sim_time": True, "resolution": .05, "footprint_padding": 0.,
            "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer"},
            "inflation_layer": {"inflation_radius": 2., "cost_scaling_factor": 1.}}}}}}
    source = tmp_path / "route_planner.yaml"
    source.write_text(yaml.safe_dump(parameters))
    original = source.read_bytes()
    output = launch.route_planner_bindings(tmp_path)
    actual = yaml.safe_load(output.read_text())
    assert actual["fuxi_route"]["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"].pop("map_topic") == "/map"
    assert actual == parameters
    assert source.read_bytes() == original


@pytest.mark.parametrize("legacy_preview", [False, True])
def test_sky_view_preserves_physics_and_manual_camera_without_preview(tmp_path, legacy_preview):
    import xml.etree.ElementTree as ET
    spec = importlib.util.spec_from_file_location("fuxi_recording_launch", PACKAGE / "launch/scanned_saved_route_validation.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    source = tmp_path / "world.sdf"
    source.write_text('<sdf version="1.6"><world name="scan"><scene><background>.1 .2 .3 1</background></scene>'
                      '<physics name="ode" type="ode"><max_step_size>.001</max_step_size></physics>'
                      '<model name="original"><link name="scan"><collision name="unchanged"/></link></model>'
                      '<gui><camera name="user_camera"><pose>1 2 3 0 0 0</pose></camera>'
                      '<plugin name="unrelated" filename="other.so"/>'
                      + ('<plugin name="agribot_camera_preview" filename="old_camera.so"/>' if legacy_preview else '')
                      + '</gui></world></sdf>')
    scene = tmp_path / "scene.json"
    manifest = {"schema_version": 1, "world": "world.sdf", "map": "map.yaml",
                "waypoints": "waypoints.yaml", "spawn": [1, 2, 0], "spawn_ground_z": 0.}
    for name in ("map.yaml", "waypoints.yaml"):
        (tmp_path / name).write_text("{}")
    scene.write_text(json.dumps(manifest))
    original = source.read_bytes()
    runtime = launch.scene_with_sky(tmp_path, scene)
    actual = json.loads(runtime.read_text())
    assert actual["map"] == "map.yaml" and actual["spawn"] == [1, 2, 0]
    assert not Path(actual["world"]).is_absolute()
    configured_spec = importlib.util.spec_from_file_location("configured_scene_binding",
        PACKAGE.parent / "agribot_vehicle_description/launch/configured_ackermann_sim.launch.py")
    configured = importlib.util.module_from_spec(configured_spec)
    configured_spec.loader.exec_module(configured)
    assert configured._external_scene_bundle(runtime)["world"] == str(tmp_path / actual["world"])
    world = ET.parse(tmp_path / actual["world"]).getroot().find("world")
    assert world.findtext("scene/sky/time") == "12"
    gui = world.find("gui")
    assert gui.find("plugin[@name='agribot_camera_preview']") is None
    assert gui.findtext("camera/pose") == "1 2 3 0 0 0"
    world.find("scene").remove(world.find("scene/sky"))
    expected = ET.fromstring(original).find("world")
    if legacy_preview:
        expected.find("gui").remove(expected.find("gui/plugin[@name='agribot_camera_preview']"))
    assert ET.tostring(world) == ET.tostring(expected)
    assert source.read_bytes() == original
    assert json.loads(scene.read_text()) == manifest
