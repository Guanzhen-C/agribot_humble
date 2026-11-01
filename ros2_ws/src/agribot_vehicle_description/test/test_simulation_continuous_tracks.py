import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location("configured_continuous_tracks", PACKAGE/"launch/configured_ackermann_sim.launch.py")
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_track_contact_follows_measured_idlers_without_changing_cad_sensors_or_body(tmp_path):
    source = PACKAGE/"generated/unity_car3/models/unity_car3.sdf"
    data = source.read_bytes()
    model = ET.fromstring(data).find("model")
    config = json.loads((PACKAGE/"generated/unity_car3/vehicle_config.json").read_text())
    base = model.find("link[@name='base_link']")
    visuals_before = [ET.tostring(v) for v in base.findall("visual")]
    inertia_before = ET.tostring(base.find("inertial"))
    sensors = [ET.tostring(s) for s in model.findall(".//sensor")]
    plugin = tmp_path/"track.so"
    plugin.write_bytes(b"test")
    module()._configure_continuous_tracks(model, config, str(plugin))
    assert [ET.tostring(v) for v in base.findall("visual")] == visuals_before
    assert ET.tostring(base.find("inertial")) == inertia_before
    pose = list(map(float, base.findtext("collision[@name='vehicle_body_collision']/pose").split()))
    size = list(map(float, base.findtext("collision[@name='vehicle_body_collision']/geometry/box/size").split()))
    assert pose[2]-size[2]/2 == pytest.approx(-.029919)
    assert len(base.findall("collision")) == 3
    assert [ET.tostring(s) for s in model.findall(".//sensor")] == sensors
    for wheel in config["geometry"]["wheelCenters"]:
        assert model.findall(f"link[@name='{wheel['id']}_wheel_link']/collision") == []
    for side in ("left", "right"):
        link = model.find(f"link[@name='{side}_continuous_track']")
        assert len(link.findall("collision")) == 3
        box = list(map(float, link.findtext("collision[@name='belt']/geometry/box/size").split()))
        assert box[0] == pytest.approx(config["geometry"]["wheelbaseM"])
        assert box[1] == pytest.approx(config["geometry"]["wheelWidthM"])
        assert box[2] == pytest.approx(2*config["geometry"]["wheelRadiusM"])
        for contact in link.findall("collision/surface/contact/ode"):
            assert float(contact.findtext("kp")) == 100000
            assert float(contact.findtext("kd")) == 1000
        assert model.findtext(f"joint[@name='{side}_continuous_track_fixed']/parent") == "base_link"
    drive = model.find("plugin[@name='continuous_track_drive']")
    assert drive.get("filename") == str(plugin)
    assert float(drive.findtext("tracks_separation")) == pytest.approx(config["geometry"]["frontTrackM"])
    assert float(drive.findtext("steering_efficiency")) == .36
    assert source.read_bytes() == data


def test_continuous_track_option_rejects_unbuilt_plugin_and_other_vehicle_types(tmp_path):
    config = json.loads((PACKAGE/"generated/unity_car3/vehicle_config.json").read_text())
    model = ET.parse(PACKAGE/"generated/unity_car3/models/unity_car3.sdf").find("model")
    with pytest.raises(ValueError, match="not been built"):
        module()._configure_continuous_tracks(model, config, str(tmp_path/"missing.so"))
    config["vehicleType"] = "ackermann"
    with pytest.raises(ValueError, match="tracked vehicle"):
        module()._configure_continuous_tracks(model, config, "unused")


def test_ros_adapter_never_overwrites_physical_vehicle_state():
    source = (PACKAGE.parent/"agribot_differential_fastlivo_mppi/src/scanned_track_drive.cpp").read_text()
    assert "gazebo::SimpleTrackedVehiclePlugin::Load" in source
    assert "SetBodyVelocity" in source
    for forbidden in ("SetWorldPose", "SetLinearVel(", "SetAngularVel(", "SetWorldPosition"):
        assert forbidden not in source
