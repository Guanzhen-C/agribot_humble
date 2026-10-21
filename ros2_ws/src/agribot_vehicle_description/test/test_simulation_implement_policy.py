"""Body-only simulation navigation must not change deployable vehicle bundles."""
import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def launch_module(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "configured_body_only", PACKAGE / "launch/configured_ackermann_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    return module


TOOL_VEHICLES = [path.parent.name for path in sorted((PACKAGE / "generated").glob("*/vehicle_config.json"))
                 if any(json.loads(path.read_text()).get(key) for key in ("implements", "manipulators"))]


@pytest.mark.parametrize("vehicle_id", TOOL_VEHICLES)
def test_implements_are_visual_and_inertial_without_contact_geometry(tmp_path, monkeypatch, vehicle_id):
    module = launch_module(tmp_path, monkeypatch)
    folder = PACKAGE / "generated" / vehicle_id
    vehicle = json.loads((folder / "vehicle_config.json").read_text())
    original = copy.deepcopy(vehicle)
    sdf = folder / "models" / (vehicle_id + ".sdf")
    urdf = folder / "urdf" / (vehicle_id + ".urdf.xacro")
    before = {path: path.read_bytes() for path in (sdf, urdf)}
    sdf_output = module._write_gazebo_vehicle_model(sdf, vehicle=vehicle)
    urdf_output = module._write_manipulator_urdf(urdf, vehicle)
    roots = [(ET.fromstring(before[sdf]).find("model"), ET.parse(sdf_output).find("model")),
             (ET.fromstring(before[urdf]), ET.parse(urdf_output).getroot())]
    for source, output in roots:
        for item in vehicle.get("implements", []):
            link = output.find(f"link[@name='{item['frameId']}']")
            old_link = source.find(f"link[@name='{item['frameId']}']")
            assert old_link.findall("collision")
            assert not link.findall("collision")
            assert ET.tostring(link.find("inertial")) == ET.tostring(old_link.find("inertial"))
            assert len(link.findall("visual")) == len(old_link.findall("visual")) > 0
        host_links = {"base_link"} | {wheel["id"] + "_wheel_link" for wheel in vehicle["geometry"]["wheelCenters"]}
        for link in source.findall("link"):
            if link.get("name") in host_links:
                actual = output.find(f"link[@name='{link.get('name')}']")
                assert [ET.tostring(c) for c in actual.findall("collision")] == [
                    ET.tostring(c) for c in link.findall("collision")]
    assert vehicle == original
    assert all(path.read_bytes() == contents for path, contents in before.items())


def test_collision_removal_follows_implement_children_but_not_vehicle_siblings(tmp_path, monkeypatch):
    module = launch_module(tmp_path, monkeypatch)
    for sdf in (False, True):
        root = ET.Element("model" if sdf else "robot")
        for name in ("base_link", "tool", "tool_wheel", "rear_left_wheel_link"):
            ET.SubElement(ET.SubElement(root, "link", name=name), "collision")
        for parent, child in (("base_link", "tool"), ("tool", "tool_wheel"),
                              ("base_link", "rear_left_wheel_link")):
            joint = ET.SubElement(root, "joint")
            for name, frame in (("parent", parent), ("child", child)):
                element = ET.SubElement(joint, name)
                if sdf:
                    element.text = frame
                else:
                    element.set("link", frame)
        module._remove_implement_collisions(root, {"implements": [{"frameId": "tool"}]}, sdf)
        for name in ("tool", "tool_wheel"):
            assert root.find(f"link[@name='{name}']/collision") is None
        for name in ("base_link", "rear_left_wheel_link"):
            assert root.find(f"link[@name='{name}']/collision") is not None


def test_all_controller_and_planner_profiles_share_body_only_footprint(tmp_path, monkeypatch):
    module = launch_module(tmp_path, monkeypatch)
    folder = PACKAGE / "generated/car4_harrow"
    vehicle = json.loads((folder / "vehicle_config.json").read_text())
    original = copy.deepcopy(vehicle)
    body = module._body_navigation_footprint(vehicle)
    assert body == module._body_navigation_footprint({**vehicle, "implements": []})
    full_config = folder / "config/simulation/nav2_params_ackermann_fastlio_static.yaml"
    before = full_config.read_bytes()
    profiles = module._write_nav2_profiles(full_config, PACKAGE / "config", "body_only", True,
                                         navigation_footprint=body)
    assert len(profiles) == 24
    for path in profiles.values():
        config = yaml.safe_load(Path(path).read_text())
        for name in ("global_costmap", "local_costmap"):
            params = config[name][name]["ros__parameters"]
            assert yaml.safe_load(params["footprint"]) == body
            # Humble's footprint parser does not accept YAML line wrapping.
            assert "\n" not in params["footprint"]
            assert json.loads(params["footprint"]) == body
    assert full_config.read_bytes() == before
    assert vehicle == original
    old = yaml.safe_load(before)["global_costmap"]["global_costmap"]["ros__parameters"]
    assert yaml.safe_load(old["footprint"]) != body


def test_large_ackermann_tuning_keeps_kinematics_and_hard_collision_checks(tmp_path, monkeypatch):
    module = launch_module(tmp_path, monkeypatch)
    source = PACKAGE / "generated/car4_harrow/config/simulation/nav2_params_ackermann_fastlio_static.yaml"
    original = yaml.safe_load(source.read_text())
    path = module._write_nav2_profile_config(source, PACKAGE / "config/rpp_controller.yaml",
        PACKAGE / "config/dwb_controller.yaml", "turn_tuning", "stvl", "mppi", "smac_hybrid", True)
    output = yaml.safe_load(Path(path).read_text())
    old_follow = original["controller_server"]["ros__parameters"]["FollowPath"]
    follow = output["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["time_steps"] == 60
    assert follow["PathAngleCritic"]["cost_weight"] == 14
    assert follow["PathAngleCritic"]["offset_from_furthest"] == 12
    assert follow["CostCritic"]["cost_weight"] == 2.5
    assert follow["CostCritic"]["collision_cost"] == old_follow["CostCritic"]["collision_cost"]
    assert follow["CostCritic"]["consider_footprint"]
    for key in ("vx_max", "vx_min", "wz_max", "AckermannConstraints"):
        assert follow[key] == old_follow[key]
    assert output["controller_server"]["ros__parameters"]["goal_checker"] == {
        "plugin": "nav2_controller::PositionGoalChecker", "xy_goal_tolerance": .25, "stateful": True}
    grid = output["planner_server"]["ros__parameters"]["GridBased"]
    old_grid = original["planner_server"]["ros__parameters"]["GridBased"]
    assert not grid["downsample_costmap"]
    assert grid["max_iterations"] == 5000000
    assert grid["cost_penalty"] == 1.5
    assert grid["non_straight_penalty"] == 1.05
    assert grid["retrospective_penalty"] == 0
    assert grid["minimum_turning_radius"] == old_grid["minimum_turning_radius"]
    assert grid["motion_model_for_search"] == old_grid["motion_model_for_search"]
