"""All configured vehicles stop at the final position, independent of yaw."""
import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
VEHICLES = sorted((PACKAGE / "generated").glob("*/vehicle_config.json"))


def load_module():
    spec = importlib.util.spec_from_file_location(
        "configured_terminal_stop", PACKAGE / "launch/configured_ackermann_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("vehicle_path", VEHICLES, ids=lambda path: path.parent.name)
@pytest.mark.parametrize("localization", ["fastlio", "navsat"])
def test_every_vehicle_and_controller_has_the_same_final_stop_policy(tmp_path, monkeypatch, vehicle_path, localization):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    vehicle = json.loads(vehicle_path.read_text())
    source = vehicle_path.parent / f"config/simulation/nav2_params_ackermann_{localization}_static.yaml"
    original = source.read_bytes()
    template = yaml.safe_load(original)
    body = module._body_navigation_footprint(vehicle)
    saved_vehicle = copy.deepcopy(vehicle)
    tolerance = vehicle["geometry"]["wheelbaseM"] * 0.5
    assert module._final_position_tolerance(vehicle) == tolerance
    # Every exported geometry passes all planner/controller paths; localization
    # changes only the input template, not the terminal policy.
    for planner in ("smac_hybrid", "navfn", "theta_star", "smac_2d"):
        for controller in ("mppi", "rpp", "dwb"):
            output = module._write_nav2_profile_config(source, PACKAGE / "config/rpp_controller.yaml",
                PACKAGE / "config/dwb_controller.yaml", "terminal", "stvl", controller, planner,
                module._uses_local_cad_profile(vehicle), body, tolerance)
            config = yaml.safe_load(Path(output).read_text())
            navigator = config["bt_navigator"]["ros__parameters"]
            assert navigator == template["bt_navigator"]["ros__parameters"]
            params = config["controller_server"]["ros__parameters"]
            assert params["goal_checker"] == {
                "plugin": "nav2_controller::PositionGoalChecker", "xy_goal_tolerance": tolerance, "stateful": True}
            assert params["publish_zero_velocity"]
            follow = params["FollowPath"]
            if controller == "mppi":
                assert not follow["GoalAngleCritic"]["enabled"]
                assert follow["CostCritic"]["collision_cost"] == 1000000
                assert follow["CostCritic"]["consider_footprint"]
                for key in ("vx_max", "vx_min", "wz_max"):
                    assert follow[key] == template["controller_server"]["ros__parameters"]["FollowPath"][key]
            elif controller == "dwb":
                assert "RotateToGoal" not in follow["critics"]
                assert "ObstacleFootprint" in follow["critics"]
            else:
                assert follow["use_collision_detection"]
            for name in ("local_costmap", "global_costmap"):
                assert json.loads(config[name][name]["ros__parameters"]["footprint"]) == body
    assert vehicle == saved_vehicle
    assert source.read_bytes() == original


def test_terminal_stop_remains_inside_the_existing_ordered_route_guard(tmp_path, monkeypatch):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    source = PACKAGE / "generated/ackermann_current/config/simulation/nav2_params_ackermann_fastlio_static.yaml"
    vehicle = json.loads((source.parents[2] / "vehicle_config.json").read_text())
    tolerance = module._final_position_tolerance(vehicle)
    profiles = module._write_nav2_profiles(source, PACKAGE / "config", "guarded_terminal", guard_route_progress=True,
                                           final_position_tolerance=tolerance)
    for path in profiles.values():
        params = yaml.safe_load(Path(path).read_text())["controller_server"]["ros__parameters"]
        assert params["route_goal_checker"]["wrapped_plugin"] == "nav2_controller::PositionGoalChecker"
        assert params["route_goal_checker"]["xy_goal_tolerance"] == tolerance
        assert params["route_goal_checker"]["requested_goal_topic"] == "/current_goal"
        assert params["route_goal_checker"]["requested_goal_frame"] == "map"
        assert params["goal_checker_plugins"] == ["goal_checker", "route_goal_checker"]


@pytest.mark.parametrize("template_vehicle", ["ackermann_current", "unity_car3"])
@pytest.mark.parametrize("localization", ["fastlio", "navsat"])
@pytest.mark.parametrize("wheelbase", [0.36, 8.2])
def test_new_vehicle_uses_its_own_half_wheelbase_in_every_profile(
        tmp_path, monkeypatch, template_vehicle, localization, wheelbase):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    bundle = PACKAGE / "generated" / template_vehicle
    vehicle = json.loads((bundle / "vehicle_config.json").read_text())
    vehicle["vehicleId"] = "future_unregistered_farm_vehicle"
    vehicle["geometry"]["wheelbaseM"] = wheelbase
    source = bundle / f"config/simulation/nav2_params_ackermann_{localization}_static.yaml"
    original = source.read_bytes()
    profiles = module._write_nav2_profiles(
        source, PACKAGE / "config", "future_vehicle",
        navigation_footprint=module._body_navigation_footprint(vehicle),
        final_position_tolerance=module._final_position_tolerance(vehicle))
    assert len(profiles) == 2 * 4 * 3
    for path in profiles.values():
        params = yaml.safe_load(Path(path).read_text())["controller_server"]["ros__parameters"]
        assert params["goal_checker"]["plugin"] == "nav2_controller::PositionGoalChecker"
        assert params["goal_checker"]["xy_goal_tolerance"] == wheelbase / 2.0
    assert source.read_bytes() == original


@pytest.mark.parametrize("wheelbase", [0.0, -1.0, float("inf"), float("nan")])
def test_invalid_wheelbase_is_rejected(wheelbase):
    module = load_module()
    with pytest.raises(ValueError, match="wheelbaseM"):
        module._final_position_tolerance({"geometry": {"wheelbaseM": wheelbase}})


@pytest.mark.parametrize("controller", ["mppi", "rpp", "dwb"])
def test_tolerance_changes_only_final_goal_acceptance_not_planning_or_intermediates(tmp_path, monkeypatch, controller):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    source = PACKAGE / "generated/car1_cultivator/config/simulation/nav2_params_ackermann_navsat_static.yaml"
    configs = []
    for tolerance in (.25, 1.4068571184316436):
        path = module._write_nav2_profile_config(source, PACKAGE / "config/rpp_controller.yaml",
            PACKAGE / "config/dwb_controller.yaml", "tolerance_only", "stvl", controller, "smac_hybrid",
            final_position_tolerance=tolerance)
        config = yaml.safe_load(Path(path).read_text())
        assert config["controller_server"]["ros__parameters"]["goal_checker"].pop("xy_goal_tolerance") == tolerance
        configs.append(config)
    assert configs[0] == configs[1]
    launch = (PACKAGE / "launch/configured_ackermann_sim.launch.py").read_text()
    assert launch.count("_body_navigation_footprint(vehicle), _final_position_tolerance(vehicle)") == 2
    assert "FinishAtFinalPosition" not in launch
    assert "simulation_plan_display" not in launch


def test_rviz_shows_unfiltered_native_planning_topics(tmp_path, monkeypatch):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    source = PACKAGE.parent / "agribot_autonomy/rviz/robot_map_global_plan_only.rviz"
    original = source.read_bytes()
    config = yaml.safe_load(Path(module._write_simulation_rviz_config(source)).read_text())
    displays = config["Visualization Manager"]["Displays"]
    paths = {display["Name"]: display["Topic"]["Value"] for display in displays
             if display["Class"] == "scout_viz/NavigationPath"}
    assert paths["Global Plan"] == "/plan"
    assert paths["Local Plan"] == "/local_plan"
    assert paths["Fixed Reference Path"] == "/waypoint_path"
    assert source.read_bytes() == original


@pytest.mark.parametrize("filename, configured", [
    ("navigate_w_replanning_ackermann_no_spin.xml", "navigate_initial_plan_with_recovery.xml"),
    ("navigate_through_poses_w_replanning_ackermann.xml", "navigate_through_poses_initial_plan_with_recovery.xml")])
def test_simulation_disables_periodic_replanning_but_keeps_recovery(filename, configured):
    source = PACKAGE.parent / "agribot_ackermann_mppi/behavior_trees" / filename
    original = source.read_bytes()
    expected = ET.fromstring(original)
    for node in expected.iter("RateController"):
        if any(child.tag in ("ComputePathToPose", "ComputePathThroughPoses") for child in node.iter()):
            node.tag = "GoalUpdatedController"
            node.attrib.pop("hz", None)
    path = PACKAGE / "config/behavior_trees" / configured
    actual = ET.parse(path).getroot()
    for root in (expected, actual):
        for node in root.iter():
            node.text = None
            node.tail = None
    assert ET.tostring(actual) == ET.tostring(expected)
    assert not list(actual.iter("RateController"))
    assert len(list(actual.iter("RecoveryNode"))) == 3
    assert list(actual.iter("GoalUpdatedController"))
    assert source.read_bytes() == original
    assert configured in (PACKAGE / "launch/configured_ackermann_sim.launch.py").read_text()
