import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.generator import _patch_simulation_nav2, generate_sdf, generate_simulation_sdf, generate_urdf
from agribot_vehicle_config_tools.model import ConfigError, validate


BASELINE = Path(__file__).resolve().parents[2] / "agribot_vehicle_description/config/vehicles/ackermann_current/vehicle_config.json"


@pytest.mark.parametrize("nominal", [False, True])
def test_nominal_ackermann_inner_wheel_limit_matches_bicycle_command(tmp_path, nominal):
    config = json.loads(BASELINE.read_text())
    config["geometry"].update(wheelbaseM=3.037, frontTrackM=2.523, maxSteeringAngleRad=.5)
    radius = 3.037 / math.tan(.5)
    config["geometry"]["minTurningRadiusM"] = radius
    if nominal:
        config.setdefault("metadata", {}).update(authoringTool="inspected_local_CAD_profile",
            calibrationState="unity_nominal_simulation_only")
    expected = math.atan2(3.037, radius - 2.523 / 2) if nominal else .5
    for sdf in (generate_sdf(config), generate_simulation_sdf(config, BASELINE.parents[4], tmp_path)):
        root = ET.fromstring(sdf)
        for side in ("left", "right"):
            limit = root.find(f".//joint[@name='front_{side}_steering_joint']/axis/limit")
            assert float(limit.findtext("upper")) == pytest.approx(expected)
            assert float(limit.findtext("lower")) == pytest.approx(-expected)
        assert float(root.findtext(".//plugin[@name='ackermann_drive']/max_steering_angle")) == .5
    root = ET.fromstring(generate_urdf(config))
    limit = root.find(".//joint[@name='front_left_steering_joint']/limit")
    assert float(limit.get("upper")) == pytest.approx(expected)
    assert config["geometry"]["maxSteeringAngleRad"] == .5


def test_high_lidar_explicit_base_height_band():
    config = json.loads(BASELINE.read_text())
    config["navigation"].update(obstacleMinHeightM=-.39, obstacleMaxHeightM=1.41)
    for sensor in config["sensors"]:
        if sensor["id"] == "lidar":
            sensor["pose"]["xyz"][2] = 1.6
    validate(config)
    template = {name: {name: {"ros__parameters": {"stvl_layer": {
        "lidar_mark": {"min_obstacle_height": .1, "max_obstacle_height": 1.0}}}}}
        for name in ("global_costmap", "local_costmap")}
    result = _patch_simulation_nav2(copy.deepcopy(template), config)
    for name in template:
        bounds = result[name][name]["ros__parameters"]["stvl_layer"]["lidar_mark"]
        assert bounds == {"min_obstacle_height": -.39, "max_obstacle_height": 1.41}
    assert template["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]["lidar_mark"]["max_obstacle_height"] == 1.0


@pytest.mark.parametrize("bounds", [{"obstacleMinHeightM": .1},
    {"obstacleMinHeightM": 2., "obstacleMaxHeightM": 1.},
    {"obstacleMinHeightM": float("nan"), "obstacleMaxHeightM": 1.}])
def test_invalid_height_band_is_rejected(bounds):
    config = json.loads(BASELINE.read_text())
    config["navigation"].update(bounds)
    with pytest.raises(ConfigError):
        validate(config)


def test_nominal_large_wheel_inertia_is_used_by_both_sdf_exporters(tmp_path):
    config = json.loads(BASELINE.read_text())
    wheel = config["geometry"]["wheelCenters"][0]
    wheel.update(radiusM=.5, widthM=.2, massKg=30., steeringMassKg=4.)
    validate(config)
    for sdf in (generate_sdf(config), generate_simulation_sdf(config, BASELINE.parents[4], tmp_path)):
        root = ET.fromstring(sdf)
        link = root.find(f".//link[@name='{wheel['id']}_wheel_link']")
        assert float(link.findtext("inertial/mass")) == 30.
        assert float(link.findtext("inertial/inertia/iyy")) == pytest.approx(3.75)
        assert float(link.findtext("inertial/inertia/ixx")) == pytest.approx(1.975)
        steering = root.find(f".//link[@name='{wheel['id']}_steering_link']")
        assert float(steering.findtext("inertial/mass")) == 4.


@pytest.mark.parametrize("value", [0., -1., float("inf")])
def test_invalid_wheel_mass_is_rejected(value):
    config = json.loads(BASELINE.read_text())
    config["geometry"]["wheelCenters"][0]["massKg"] = value
    with pytest.raises(ConfigError):
        validate(config)


def test_default_height_band_stays_sensor_relative():
    config = json.loads(BASELINE.read_text())
    lidar_height = next(sensor for sensor in config["sensors"] if sensor["id"] == "lidar")["pose"]["xyz"][2]
    template = {"local_costmap": {"local_costmap": {"ros__parameters": {
        "stvl_layer": {"lidar_mark": {"min_obstacle_height": .25, "max_obstacle_height": 1.}}}}}}
    marking = _patch_simulation_nav2(template, config)["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]["lidar_mark"]
    assert marking["min_obstacle_height"] == pytest.approx(lidar_height + .02)
    assert marking["max_obstacle_height"] == pytest.approx(1.)


@pytest.mark.parametrize("large,expected", [(False, 8), (True, 14)])
def test_local_window_contains_large_vehicle_and_forward_margin(large, expected):
    config = json.loads(BASELINE.read_text())
    if large:
        config["geometry"]["bodyExtents"] = dict(frontM=4.5, rearM=1.2, leftM=1.6, rightM=1.6)
    template = {"local_costmap": {"local_costmap": {"ros__parameters": {"width": 8, "height": 8}}}}
    parameters = _patch_simulation_nav2(template, config)["local_costmap"]["local_costmap"]["ros__parameters"]
    assert parameters["width"] == parameters["height"] == expected


@pytest.mark.parametrize("torque", [None, 1500.])
def test_nominal_drive_torque_reaches_plugin_and_joint_limits(tmp_path, torque):
    config = json.loads(BASELINE.read_text())
    if torque is not None:
        config["dynamics"]["wheelTorqueNm"] = torque
    validate(config)
    root = ET.fromstring(generate_simulation_sdf(config, BASELINE.parents[4], tmp_path))
    assert float(root.findtext(".//plugin[@name='ackermann_drive']/wheel_torque")) == (torque or 200.)
    if torque is not None:
        for wheel in config["geometry"]["wheelCenters"]:
            assert float(root.findtext(f".//joint[@name='{wheel['id']}_wheel_joint']/axis/limit/effort")) == torque
