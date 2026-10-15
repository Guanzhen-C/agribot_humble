import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from agribot_vehicle_config_tools.generator import generate_sdf, generate_urdf, generate_simulation_sdf
from agribot_vehicle_config_tools.manipulator import default_manipulator, model_data
from agribot_vehicle_config_tools.model import ConfigError, footprint, load_and_validate, validate

SRC = Path(__file__).resolve().parents[2]


def load(vehicle):
    config = load_and_validate(SRC / "agribot_vehicle_description/config/vehicles" / vehicle / "vehicle_config.json")[0]
    config.pop("manipulators", None)
    return config


def signature(node):
    return (node.tag, node.attrib, (node.text or "").strip(), [signature(child) for child in node])


@pytest.mark.parametrize("vehicle", ["ackermann_current", "unity_car3"])
def test_optional_arm_preserves_vehicle_and_sensors(vehicle, tmp_path):
    baseline = load(vehicle)
    config = copy.deepcopy(baseline)
    config["manipulators"] = [default_manipulator(config)]
    assert validate(config) == validate(baseline)
    assert footprint(config) == footprint(baseline)
    robot = ET.fromstring(generate_urdf(config))
    assert len(robot.findall("./joint[@type='revolute']")) >= 6
    assert robot.find("./joint[@name='manipulator_mount_joint']/parent").get("link") == "base_link"
    assert robot.find("./link[@name='manipulator_link_base']/visual/geometry/mesh").get("filename").endswith("link_base.stl")
    assert robot.find("./joint[@name='manipulator_gripper_mount']/parent").get("link") == "manipulator_link6"
    for index, direction in ((1, "0 1 0"), (2, "0 -1 0")):
        joint = robot.find(f"./joint[@name='manipulator_finger_joint{index}']")
        assert joint.get("type") == "prismatic"
        assert joint.find("axis").get("xyz") == direction
        assert float(joint.find("limit").get("upper")) == 0.0089
    for original in ET.fromstring(generate_urdf(baseline)):
        match = robot.find(f"./{original.tag}[@name='{original.get('name')}']")
        if original.tag in ("link", "joint"):
            assert signature(match) == signature(original)
    for xml in (generate_sdf(config), generate_simulation_sdf(config, SRC, tmp_path)):
        root = ET.fromstring(xml)
        assert len([joint for joint in root.findall(".//joint") if joint.get("name").startswith("manipulator_joint")]) == 6
        assert root.find(".//plugin[@name='vehicle_manipulator_trajectory']") is not None
        assert len(root.findall(".//plugin[@name='vehicle_manipulator_state']/joint_name")) == 8
        assert root.find(".//plugin[@name='vehicle_manipulator_state']/ros/remapping").text == "joint_states:=/joint_states"
        # Display-only meshes cannot become false LiDAR collision obstacles.
        assert root.find(".//link[@name='manipulator_link1']/collision") is None


@pytest.mark.parametrize("fault", ["model", "parent", "count", "flag", "pose", "frame"])
def test_invalid_arm_rejected(fault):
    config = load("ackermann_current")
    item = default_manipulator(config)
    config["manipulators"] = [item]
    if fault == "model": item["model"] = "unknown"
    elif fault == "parent": item["parentFrameId"] = "world"
    elif fault == "count": config["manipulators"] *= 2
    elif fault == "flag": item["demoEnabled"] = "true"
    elif fault == "pose": item["pose"]["xyz"][0] = math.nan
    else: config["sensors"][0]["frameId"] = "manipulator_link1"
    with pytest.raises(ConfigError):
        validate(config)


def test_arm_is_unavailable_on_other_vehicles():
    config = load("unity_car1")
    with pytest.raises(ConfigError):
        default_manipulator(config)
    config["manipulators"] = [default_manipulator(load("ackermann_current"))]
    with pytest.raises(ConfigError):
        validate(config)


def test_removed_arm_is_not_generated():
    config = load("ackermann_current")
    baseline = generate_urdf(config)
    config["manipulators"] = []
    assert generate_urdf(config) == baseline
    assert "manipulator_" not in generate_sdf(config)


def test_upstream_meshes_are_small_binary_stl():
    import struct
    for name in model_data()["links"]:
        path = SRC / "agribot_manipulator_sim/models/lite6/meshes" / (name + ".stl")
        content = path.read_bytes()
        count = struct.unpack_from("<I", content, 80)[0]
        assert len(content) == 84 + count * 50
        assert 100 < count < 20000


def test_arm_stows_on_navigation_commands_without_repeated_pose_updates():
    launch = (SRC / "agribot_vehicle_description/launch/configured_ackermann_sim.launch.py").read_text()
    assert 'remappings=[("/cmd_vel", "/nav2/cmd_vel")]' in launch
    source = (SRC / "agribot_manipulator_sim/agribot_manipulator_sim/arm_demo.py").read_text()
    assert "if not active and self.stowed:" in source
    assert "if not active and not any(point.positions):" in source
