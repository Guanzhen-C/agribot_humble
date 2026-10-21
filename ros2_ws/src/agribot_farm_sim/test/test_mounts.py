import json
import os
from pathlib import Path

import pytest

from agribot_farm_sim.mounts import mounted_configuration, prepare_mount, transport_balance, transport_wheel_clearance
from agribot_farm_sim.hitches import HOSTS, TOOLS
from agribot_vehicle_config_tools.model import ConfigError, load_and_validate


BASELINE = Path(__file__).parents[2] / "agribot_vehicle_description/generated"


def inputs(vehicle_id="compact_tractor"):
    vehicle, _ = load_and_validate(BASELINE / vehicle_id / "vehicle_config.json")
    vehicle["manipulators"] = []
    vehicle["implements"] = []
    vehicle["dynamics"] = {"massKg": 2000.0, "centerOfMass": [.2, 0., .2]}
    vehicle["visuals"][0]["meshUri"] = f"package://agribot_vehicle_description/generated/{vehicle_id}/models/vehicle_visual.obj"
    asset = {"id": "compact_chisel_plow", "displayName": "Chisel plow", "coordinateConvention": "ROS_FLU",
             "units": "metres", "originDefinition": "front_ground_center", "minimumM": [-2., -1., 0.],
             "maximumM": [0., 1., 1.], "sourceSha256": "a" * 64,
             "redistributionLicense": "unverified_user_supplied"}
    return vehicle, asset


def test_mount_keeps_original_config_and_sensor_calibration():
    vehicle, asset = inputs()
    before = json.dumps(vehicle, sort_keys=True)
    mounted = mounted_configuration(vehicle, asset, "tractor_plow")
    assert json.dumps(vehicle, sort_keys=True) == before
    assert mounted["sensors"] == vehicle["sensors"]
    assert mounted["geometry"] == vehicle["geometry"]
    assert "/generated/tractor_plow/" in mounted["visuals"][0]["meshUri"]
    assert mounted["implements"][0]["simulationOnly"] is True
    assert 0 < mounted["metadata"]["implementTransportBalance"]["frontAxleLoadFraction"] < 1
    minimum_ground = min(w["position"][2] - w.get("radiusM", vehicle["geometry"]["wheelRadiusM"])
                         for w in vehicle["geometry"]["wheelCenters"])
    assert mounted["implements"][0]["pose"]["xyz"][2] - minimum_ground == pytest.approx(.15)


def test_large_tool_not_silently_mounted_on_original_small_car():
    vehicle, asset = inputs()
    vehicle["vehicleId"] = "ackermann_current"
    with pytest.raises(ConfigError, match="only available"):
        mounted_configuration(vehicle, asset, "test_vehicle")


def test_combine_header_is_separate_and_preserves_inspected_mount():
    vehicle, asset = inputs()
    vehicle["vehicleId"] = "tracked_harvester"
    vehicle["vehicleType"] = "tracked"
    vehicle["dynamics"].update(massKg=14500., centerOfMass=[.3, 0., 1.9])
    vehicle["metadata"]["simulationSkidContact"] = {
        "lateralFriction": .1, "longitudinalFriction": 1.8,
    }
    vehicle["geometry"].update(wheelbaseM=3.564, minTurningRadiusM=5.)
    for wheel in vehicle["geometry"]["wheelCenters"]:
        wheel.update(steering=False, driven=True)
        wheel["position"][0] = 1.835 if wheel["id"].startswith("front_") else -1.729
    asset.update(id="combine_header", originDefinition="inspected_host_mount",
        minimumM=[-.1, -6.34, -.705], maximumM=[3.063, 6.389, .758],
        mountPoseXyz=[4.12, 0., .83], collisionBoxes=[dict(name="header_envelope",
            center=[1.4815, .0245, .0265], size=[3.165, 12.731, 1.465])])
    before = json.dumps(vehicle, sort_keys=True)
    mounted = mounted_configuration(vehicle, asset, "harvester_header")
    assert json.dumps(vehicle, sort_keys=True) == before
    assert mounted["sensors"] == vehicle["sensors"]
    assert mounted["geometry"] == vehicle["geometry"]
    assert mounted["implements"][0]["pose"]["xyz"] == [4.12, 0., .83]
    assert mounted["implements"][0]["frameId"] == "combine_header_link"
    assert mounted["implements"][0]["massKg"] == 2500.
    fraction = mounted["metadata"]["implementTransportBalance"]["frontAxleLoadFraction"]
    contact = mounted["metadata"]["simulationSkidContact"]
    assert contact["frontLateralFriction"] * fraction == pytest.approx(
        contact["rearLateralFriction"] * (1.0 - fraction))
    assert contact["longitudinalFriction"] == 1.8
    vehicle["vehicleId"] = "compact_tractor"
    with pytest.raises(ConfigError, match="tracked_harvester"):
        mounted_configuration(vehicle, asset, "wrong_header")


def test_mount_rejects_front_axle_unloading_instead_of_changing_mass():
    vehicle, asset = inputs()
    vehicle["dynamics"]["massKg"] = 10.0
    before = json.dumps(vehicle, sort_keys=True)
    with pytest.raises(ConfigError, match="outside the axle support span"):
        mounted_configuration(vehicle, asset, "unbalanced")
    assert json.dumps(vehicle, sort_keys=True) == before


def test_missing_inspected_anchors_never_falls_back_to_a_fake_box(monkeypatch):
    vehicle, asset = inputs()
    monkeypatch.setattr("agribot_farm_sim.mounts.hitches.layout", lambda *args: None)
    with pytest.raises(ConfigError, match="requires inspected"):
        mounted_configuration(vehicle, asset, "uninspected_mount")


@pytest.mark.parametrize("tool", ["garlic_planter", "mulch_planter"])
def test_frame_only_tools_have_explicit_receiver_collision_and_keep_calibration(tool):
    vehicle, asset = inputs()
    asset["id"] = tool
    mounted = mounted_configuration(vehicle, asset, "new_receiver")
    item = mounted["implements"][0]
    names = {box["name"] for box in item["collisionBoxes"]}
    assert names == {"implement_envelope", "hitch_lower_left", "hitch_lower_right", "hitch_top_link", "hitch_receiver_frame"}
    assert mounted["metadata"]["implementHitch"]["receiverFrame"]["footings"]
    assert mounted["sensors"] == vehicle["sensors"]
    assert mounted["geometry"] == vehicle["geometry"]


def test_wide_groove_maker_keeps_its_width_on_an_inspected_host():
    vehicle, asset = inputs()
    asset.update(id="groove_maker", minimumM=[-1.2, -2.756, 0.], maximumM=[0., 2.756, 2.105])
    compact = mounted_configuration(vehicle, asset, "tractor_groove")
    assert compact["implements"][0]["collisionBoxes"][0]["size"][1] == pytest.approx(5.514)
    vehicle, _ = inputs("cab_tractor")
    vehicle["visuals"][0]["meshUri"] = "package://agribot_vehicle_description/generated/cab_tractor/models/vehicle_visual.obj"
    mounted = mounted_configuration(vehicle, asset, "cab_groove")
    assert mounted["sensors"] == vehicle["sensors"]
    assert "/generated/cab_groove/" in mounted["visuals"][0]["meshUri"]
    box = mounted["implements"][0]["collisionBoxes"][0]
    assert box["size"][1] == pytest.approx(5.514)
    assert mounted["implements"][0]["massKg"] == 220.0


def test_prepare_is_local_and_never_overwrites_original(tmp_path):
    vehicle, asset = inputs()
    original = tmp_path / "original"
    original.mkdir()
    path = original / "vehicle_config.json"
    path.write_text(json.dumps(vehicle))
    assets = tmp_path / "asset"
    assets.mkdir()
    (assets / "implement_asset.json").write_text(json.dumps(asset))
    (assets / "implement_visual.obj").write_text("mtllib implement_visual.mtl\nv -1 0 .5\n")
    (assets / "implement_visual.mtl").write_text("newmtl steel\nKd .2 .2 .2\n")
    output = tmp_path / "mounted"
    result = prepare_mount(path, assets, output, "tractor_plow")
    assert load_and_validate(result)[0]["vehicleId"] == "tractor_plow"
    assert json.loads(path.read_text()) == vehicle
    assert "mtllib implement_compact_chisel_plow_visual.mtl" in (output / "implement_compact_chisel_plow_visual.obj").read_text()
    with pytest.raises(FileExistsError):
        prepare_mount(path, assets, output, "tractor_plow")


def test_cultivator_preserves_width_and_rejects_uninspected_small_cars():
    vehicle, asset = inputs("cab_tractor")
    asset.update(id="mounted_cultivator", minimumM=[-2.27035, -1.341, 0.], maximumM=[0., 1.341, 1.133])
    mounted = mounted_configuration(vehicle, asset, "cab_cultivator")
    item = mounted["implements"][0]
    assert item["collisionBoxes"][0]["size"][1] == pytest.approx(2.684)
    assert item["massKg"] == 180.0
    assert item["workingWidthM"] == 2.6
    assert mounted["sensors"] == vehicle["sensors"]
    assert mounted["metadata"]["calibrationState"] == "unity_nominal_simulation_only"
    for identifier in ("ackermann_current", "unity_car3", "ride_on_mower"):
        vehicle["vehicleId"] = identifier
        with pytest.raises(ConfigError, match="only available"):
            mounted_configuration(vehicle, asset, "wrong_mount")


def test_crusher_is_nominal_transport_on_inspected_tractors():
    vehicle, asset = inputs("cab_tractor")
    asset.update(id="forest_chain_crusher", minimumM=[-1.125, -1.42312, 0.],
                 maximumM=[0., 1.42312, 1.293])
    mounted = mounted_configuration(vehicle, asset, "cab_crusher")
    item = mounted["implements"][0]
    assert item["massKg"] == 350.0
    assert item["workingWidthM"] == 2.4
    assert item["collisionBoxes"][0]["size"][1] == pytest.approx(2.84824)
    assert item["mountMode"] == "fixed_transport"
    assert item["simulationOnly"] is True
    assert mounted["sensors"] == vehicle["sensors"]
    for identifier in ("ackermann_current", "unity_car3", "ride_on_mower"):
        vehicle["vehicleId"] = identifier
        with pytest.raises(ConfigError, match="only available"):
            mounted_configuration(vehicle, asset, "wrong_crusher_mount")


def test_interrow_mill_preserves_four_row_envelope_and_existing_sensors():
    vehicle, asset = inputs("cab_tractor")
    vehicle["dynamics"]["centerOfMass"][0] = 1.15
    asset.update(id="inter_row_milling_cutter", minimumM=[-1.812, -1.5, 0.],
                 maximumM=[0., 1.5, 1.868])
    mounted = mounted_configuration(vehicle, asset, "cab_interrow")
    item = mounted["implements"][0]
    assert item["massKg"] == 450.0
    assert item["collisionBoxes"][0]["size"] == pytest.approx([1.814, 3.002, 1.870])
    assert item["mountMode"] == "fixed_transport"
    assert mounted["sensors"] == vehicle["sensors"]
    for identifier in ("ackermann_current", "unity_car3", "ride_on_mower"):
        vehicle["vehicleId"] = identifier
        with pytest.raises(ConfigError, match="only available"):
            mounted_configuration(vehicle, asset, "wrong_interrow_mount")


def test_wheel_collision_is_rejected_instead_of_hidden():
    vehicle, asset = inputs()
    config = mounted_configuration(vehicle, asset, "wheel_check")
    wheel = config["geometry"]["wheelCenters"][2]
    config["implements"][0]["pose"]["xyz"] = list(wheel["position"])
    config["implements"][0]["collisionBoxes"] = [dict(name="collision", center=[0., 0., 0.], size=[.2, .2, .2])]
    with pytest.raises(ConfigError, match="wheel envelope"):
        transport_wheel_clearance(config)


@pytest.mark.parametrize("host", HOSTS)
@pytest.mark.parametrize("tool", TOOLS)
def test_all_rear_pairings_keep_real_geometry_and_positive_axle_loading(host, tool):
    assets = os.environ.get("AGRIBOT_FARM_ASSET_ROOT")
    if not assets:
        pytest.skip("Set AGRIBOT_FARM_ASSET_ROOT for inspected local CAD profiles")
    root = Path(assets)
    source = root / "original_tractors_metric_v5" / host if host.startswith("unity_") else root / host / ("ros_v6" if host == "compact_tractor" else "ros_v7")
    vehicle, _ = load_and_validate(source / "vehicle_config.json")
    version = "ros_v2" if tool == "mounted_cultivator" else "ros_v1"
    asset = json.loads((root / tool / version / "implement_asset.json").read_text())
    before = json.dumps(vehicle, sort_keys=True)
    if host == "compact_tractor" and tool in ("forest_chain_crusher", "inter_row_milling_cutter"):
        with pytest.raises(ConfigError, match="outside the axle support"):
            mounted_configuration(vehicle, asset, "needs_ballast")
    else:
        config = mounted_configuration(vehicle, asset, "pairing_check")
        assert config["sensors"] == vehicle["sensors"]
        assert config["geometry"] == vehicle["geometry"]
        assert config["metadata"]["implementWheelClearance"]["minimumSeparationLowerBoundM"] > 0
        assert 0 < config["metadata"]["implementTransportBalance"]["frontAxleLoadFraction"] < 1
    assert json.dumps(vehicle, sort_keys=True) == before


@pytest.mark.parametrize("folder", ["tractor_chisel_v4", "tractor_harrow_v4", "cab_groove_v1", "cab_cultivator_v2", "cab_crusher_v1", "cab_interrow_v1"])
def test_local_mount_has_positive_nominal_front_and_rear_axle_loading(folder):
    root = os.environ.get("AGRIBOT_FARM_ASSET_ROOT")
    if not root:
        pytest.skip("Set AGRIBOT_FARM_ASSET_ROOT to inspect mounted CAD exports")
    config, _ = load_and_validate(Path(root) / "mounted" / folder / "vehicle_config.json")
    balance = transport_balance(config)
    assert balance["totalMassKg"] > config["dynamics"]["massKg"]
    assert 0 < balance["frontAxleLoadFraction"] < 1
