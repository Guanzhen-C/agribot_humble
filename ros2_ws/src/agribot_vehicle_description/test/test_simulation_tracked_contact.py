"""Keep the measured Car3 contact override local to configured simulation."""
import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config/tracked_contact_simulation.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "configured_tracked_contact", PACKAGE / "launch/configured_ackermann_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_vehicle(vehicle_id):
    folder = PACKAGE / "generated" / vehicle_id
    config = json.loads((folder / "vehicle_config.json").read_text())
    return config, folder / "models" / (vehicle_id + ".sdf")


def test_car3_override_only_changes_wheel_friction(tmp_path, monkeypatch):
    module = load_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    config, source = load_vehicle("unity_car3")
    config["manipulators"] = []
    original_config = copy.deepcopy(config)
    source_bytes = source.read_bytes()
    before_path = module._write_gazebo_vehicle_model(source, vehicle=config)
    before = ET.parse(before_path).find("model")
    path = module._write_gazebo_vehicle_model(source, vehicle=config, contact_config=PROFILE)
    after = ET.parse(path).find("model")
    plugin = after.find("plugin[@name='configured_tracked_wheel_slip']")
    assert plugin.get("filename") == "libagribot_simulation_tracked_wheel_slip.so"
    assert float(plugin.findtext("straight_compliance")) == .1
    assert float(plugin.findtext("turning_slip")) == .003
    normal_force = sum(float(item.text) for item in before.findall("link/inertial/mass")) * 9.81 / 4
    assert len(plugin.findall("wheel")) == 4
    for wheel in config["geometry"]["wheelCenters"]:
        query = f"link[@name='{wheel['id']}_wheel_link']/collision/surface/friction/ode"
        ode = after.find(query)
        assert ode.findtext("fdir1") == "0 0 1"
        assert float(ode.findtext("mu")) == .36
        assert float(ode.findtext("mu2")) == 1.8
        assert float(ode.findtext("slip1")) == 0
        assert float(ode.findtext("slip2")) == 0
        node = plugin.find(f"wheel[@link_name='{wheel['id']}_wheel_link']")
        assert float(node.findtext("wheel_normal_force")) == pytest.approx(normal_force)
        assert float(node.findtext("wheel_radius")) == wheel["radiusM"]
        assert float(node.findtext("slip_compliance_lateral")) == .1
        assert float(node.findtext("slip_compliance_longitudinal")) == 0
        ode.remove(ode.find("slip1"))
        ode.remove(ode.find("slip2"))
        for key in ("fdir1", "mu", "mu2"):
            ode.find(key).text = before.find(query + "/" + key).text
    after.remove(plugin)
    ET.indent(after)
    ET.indent(before)
    assert ET.tostring(after) == ET.tostring(before)
    assert source.read_bytes() == source_bytes
    assert config == original_config


def test_legacy_short_proxy_keeps_its_separate_friction_profile():
    config, source = load_vehicle("unity_car3")
    config["metadata"].pop("simulationTrackContactGeometry")
    model = ET.parse(source).find("model")
    load_module()._configure_tracked_contact(model, config, PROFILE)
    for wheel in config["geometry"]["wheelCenters"]:
        ode = model.find(f"link[@name='{wheel['id']}_wheel_link']/collision/surface/friction/ode")
        assert float(ode.findtext("mu")) == (.36 if wheel["id"].startswith("front_") else .6)
        assert ode.find("slip1") is None


@pytest.mark.parametrize("vehicle_id", ["ackermann_current", "unity_car1", "unity_car2",
    "unity_car4", "unity_car5"])
def test_other_vehicles_keep_original_contacts(vehicle_id):
    config, source = load_vehicle(vehicle_id)
    model = ET.parse(source).find("model")
    before = ET.tostring(model)
    load_module()._configure_tracked_contact(model, config, PROFILE)
    assert ET.tostring(model) == before


@pytest.mark.parametrize("vehicle_id", ["tracked_harvester", "harvester_header"])
def test_combine_uses_mass_appropriate_adaptive_slip_without_changing_geometry(vehicle_id):
    config, source = load_vehicle(vehicle_id)
    original_config = copy.deepcopy(config)
    before = ET.parse(source).find("model")
    after = copy.deepcopy(before)
    profile = yaml.safe_load(PROFILE.read_text())[vehicle_id]
    load_module()._configure_tracked_contact(after, config, PROFILE)
    plugin = after.find("plugin[@name='configured_tracked_wheel_slip']")
    assert float(plugin.findtext("turning_slip")) == 1e-6
    assert float(plugin.findtext("straight_compliance")) == .1
    assert len(plugin.findall("wheel")) == 4
    assert config == original_config
    normal_force = sum(float(item.text) for item in before.findall("link/inertial/mass")) * 9.81 / 4
    for wheel in config["geometry"]["wheelCenters"]:
        query = f"link[@name='{wheel['id']}_wheel_link']/collision/surface/friction/ode"
        ode = after.find(query)
        prior = before.find(query)
        key = "front_lateral_friction" if wheel["id"].startswith("front_") else "rear_lateral_friction"
        assert float(ode.findtext("mu")) == pytest.approx(profile[key])
        assert float(ode.findtext("mu")) / float(prior.findtext("mu")) == pytest.approx(2.)
        assert float(ode.findtext("mu2")) == 1.8
        assert float(ode.findtext("slip1")) == float(ode.findtext("slip2")) == 0
        node = plugin.find(f"wheel[@link_name='{wheel['id']}_wheel_link']")
        assert float(node.findtext("wheel_radius")) == wheel["radiusM"]
        assert float(node.findtext("wheel_normal_force")) == pytest.approx(normal_force)
        for item in list(ode):
            ode.remove(item)
        for item in prior:
            ode.append(copy.deepcopy(item))
    after.remove(plugin)
    ET.indent(after)
    ET.indent(before)
    assert ET.tostring(after) == ET.tostring(before)


@pytest.mark.parametrize("value", [-1, 0, True, float("nan"), float("inf")])
def test_invalid_friction_is_rejected(tmp_path, value):
    config, source = load_vehicle("unity_car3")
    profile = yaml.safe_load(PROFILE.read_text())
    profile["unity_car3"]["longitudinal_friction"] = value
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="finite and positive"):
        load_module()._configure_tracked_contact(ET.parse(source).find("model"), config, path)


def test_wrong_vehicle_type_and_missing_contact_fail_explicitly():
    config, source = load_vehicle("unity_car3")
    module = load_module()
    model = ET.parse(source).find("model")
    config["vehicleType"] = "ackermann"
    with pytest.raises(ValueError, match="requires a tracked vehicle"):
        module._configure_tracked_contact(model, config, PROFILE)
    config["vehicleType"] = "tracked"
    ode = model.find("link[@name='front_left_wheel_link']/collision/surface/friction")
    ode.remove(ode.find("ode"))
    with pytest.raises(ValueError, match="missing ODE friction"):
        module._configure_tracked_contact(model, config, PROFILE)


@pytest.mark.parametrize("value", [-1, 0, True, float("nan"), float("inf")])
def test_invalid_idler_slip_compliance_is_rejected(tmp_path, value):
    config, source = load_vehicle("unity_car3")
    profile = yaml.safe_load(PROFILE.read_text())
    profile["unity_car3"]["idler_aligned_slip_compliance"] = value
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="finite and positive"):
        load_module()._configure_tracked_contact(ET.parse(source).find("model"), config, path)


def test_wheel_slip_plugin_is_not_duplicated_on_reapplication():
    config, source = load_vehicle("unity_car3")
    module = load_module()
    model = ET.parse(source).find("model")
    module._configure_tracked_contact(model, config, PROFILE)
    module._configure_tracked_contact(model, config, PROFILE)
    assert len(model.findall("plugin[@filename='libagribot_simulation_tracked_wheel_slip.so']")) == 1


def test_turn_transition_and_normal_load_are_validated(tmp_path):
    config, source = load_vehicle("unity_car3")
    module = load_module()
    profile = yaml.safe_load(PROFILE.read_text())
    profile["unity_car3"]["turn_full_speed_difference"] = .01
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="increasing"):
        module._configure_tracked_contact(ET.parse(source).find("model"), config, path)
    model = ET.parse(source).find("model")
    model.find("link/inertial/mass").text = "nan"
    with pytest.raises(ValueError, match="masses"):
        module._configure_tracked_contact(model, config, PROFILE)
