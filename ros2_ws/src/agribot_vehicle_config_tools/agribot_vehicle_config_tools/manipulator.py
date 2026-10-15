"""Optional Lite6 model assembly. The current mode is kinematic display only."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

PREFIX = "manipulator_"
MODEL_URI = "package://agribot_manipulator_sim/models/lite6/meshes/"


def model_data():
    source = Path(__file__).resolve().parents[2] / "agribot_manipulator_sim/models/lite6/model.json"
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory("agribot_manipulator_sim")) / "models/lite6/model.json"
    return json.loads(source.read_text(encoding="utf-8"))


def default_manipulator(config):
    if config["vehicleId"] not in ("ackermann_current", "unity_car3"):
        from .model import ConfigError
        raise ConfigError("manipulator is supported only on ackermann_current and unity_car3")
    collision = config["collision"]
    body = config["geometry"]["bodyExtents"]
    # Nominal simulation mount, never a measured real-machine calibration.
    x = -body["rearM"] + min(0.20, (body["frontM"] + body["rearM"]) * 0.25)
    z = collision["center"][2] + collision["size"][2] * 0.5 + 0.015
    return {"model": "lite6", "parentFrameId": "base_link", "demoEnabled": True,
            "pose": {"xyz": [x, 0.0, z], "rpy": [0.0, 0.0, 0.0],
                     "quaternion": [0.0, 0.0, 0.0, 1.0]}}


def validate_manipulators(config, frames):
    from .model import ConfigError, _validate_pose
    items = config.get("manipulators", [])
    if not isinstance(items, list) or len(items) > 1:
        raise ConfigError("manipulators must be an array containing at most one arm")
    for item in items:
        if config["vehicleId"] not in ("ackermann_current", "unity_car3"):
            raise ConfigError("manipulator is supported only on ackermann_current and unity_car3")
        if not isinstance(item, dict) or item.get("model") != "lite6" or item.get("parentFrameId") != "base_link":
            raise ConfigError("manipulator requires model lite6 and parentFrameId base_link")
        if type(item.get("demoEnabled")) is not bool:
            raise ConfigError("manipulator.demoEnabled must be boolean")
        _validate_pose(item.get("pose"), "manipulator.pose")
        if any(PREFIX + name in frames for name in model_data()["links"]):
            raise ConfigError("manipulator frame conflicts with a sensor frame")


def link_color(name):
    return "0.20 0.22 0.24 1" if name.startswith("finger") else (
        "0.60 0.64 0.68 1" if name == "link6" else "0.85 0.88 0.90 1")


def _text(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def _vector(values):
    return " ".join(format(value, ".10g") for value in values)


def append_urdf(robot, config):
    for arm in config.get("manipulators", []):
        data = model_data()
        for name in data["links"]:
            link = ET.SubElement(robot, "link", name=PREFIX + name)
            visual = ET.SubElement(link, "visual")
            offset = [0.0, 0.0, 0.0]
            if name == "link2":
                offset[2] = -data["joints"][1]["xyz"][1]
            ET.SubElement(visual, "origin", xyz=_vector(offset), rpy="0 0 0")
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename=MODEL_URI + name + ".stl")
            material = ET.SubElement(visual, "material", name=PREFIX + name + "_material")
            ET.SubElement(material, "color", rgba=link_color(name))
        mount = ET.SubElement(robot, "joint", name=PREFIX + "mount_joint", type="fixed")
        ET.SubElement(mount, "parent", link="base_link")
        ET.SubElement(mount, "child", link=PREFIX + "link_base")
        ET.SubElement(mount, "origin", xyz=_vector(arm["pose"]["xyz"]), rpy=_vector(arm["pose"]["rpy"]))
        for entry in data["joints"]:
            kind = entry.get("type", "revolute")
            joint = ET.SubElement(robot, "joint", name=PREFIX + entry["name"], type=kind)
            ET.SubElement(joint, "parent", link=PREFIX + entry["parent"])
            ET.SubElement(joint, "child", link=PREFIX + entry["child"])
            ET.SubElement(joint, "origin", xyz=_vector(entry["xyz"]), rpy=_vector(entry["rpy"]))
            if kind != "fixed":
                ET.SubElement(joint, "axis", xyz=_vector(entry.get("axis", [0, 0, 1])))
                ET.SubElement(joint, "limit", lower=str(entry["lower"]), upper=str(entry["upper"]),
                              effort="5" if kind == "prismatic" else "50", velocity="0.05" if kind == "prismatic" else "3.14")
                if kind == "revolute":
                    ET.SubElement(joint, "dynamics", damping="1.0", friction="1.0")


def append_sdf(model, config):
    for arm in config.get("manipulators", []):
        data = model_data()
        by_child = {entry["child"]: entry for entry in data["joints"]}
        for name in data["links"]:
            link = ET.SubElement(model, "link", name=PREFIX + name)
            if name == "link_base":
                pose = ET.SubElement(link, "pose", relative_to="base_link")
                pose.text = _vector(arm["pose"]["xyz"] + arm["pose"]["rpy"])
            else:
                entry = by_child[name]
                pose = ET.SubElement(link, "pose", relative_to=PREFIX + entry["parent"])
                pose.text = _vector(entry["xyz"] + entry["rpy"])
            # These links are a display mechanism, not a payload/contact model.
            _text(link, "gravity", "false")
            inertial = ET.SubElement(link, "inertial")
            _text(inertial, "mass", 0.01)
            inertia = ET.SubElement(inertial, "inertia")
            for key in ("ixx", "iyy", "izz"):
                _text(inertia, key, 0.0001)
            for key in ("ixy", "ixz", "iyz"):
                _text(inertia, key, 0)
            visual = ET.SubElement(link, "visual", name=PREFIX + name + "_visual")
            if name == "link2":
                _text(visual, "pose", "0 0 " + str(-data["joints"][1]["xyz"][1]) + " 0 0 0")
            mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
            _text(mesh, "uri", MODEL_URI + name + ".stl")
            material = ET.SubElement(visual, "material")
            _text(material, "ambient", link_color(name))
            _text(material, "diffuse", link_color(name))
        mount = ET.SubElement(model, "joint", name=PREFIX + "mount_joint", type="fixed")
        _text(mount, "parent", "base_link")
        _text(mount, "child", PREFIX + "link_base")
        for entry in data["joints"]:
            kind = entry.get("type", "revolute")
            joint = ET.SubElement(model, "joint", name=PREFIX + entry["name"], type=kind)
            _text(joint, "parent", PREFIX + entry["parent"])
            _text(joint, "child", PREFIX + entry["child"])
            if kind == "fixed":
                continue
            axis = ET.SubElement(joint, "axis")
            _text(axis, "xyz", _vector(entry.get("axis", [0, 0, 1])))
            limit = ET.SubElement(axis, "limit")
            _text(limit, "lower", entry["lower"])
            _text(limit, "upper", entry["upper"])
            _text(limit, "effort", 5 if kind == "prismatic" else 50)
            _text(limit, "velocity", 0.05 if kind == "prismatic" else 3.14)
            if kind == "revolute":
                dynamics = ET.SubElement(axis, "dynamics")
                _text(dynamics, "damping", 1.0)
                _text(dynamics, "friction", 1.0)
        plugin = ET.SubElement(model, "plugin", name="vehicle_manipulator_trajectory",
                               filename="libgazebo_ros_joint_pose_trajectory.so")
        ros = ET.SubElement(plugin, "ros")
        _text(ros, "remapping", "set_joint_trajectory:=/manipulator/joint_trajectory")
        _text(ros, "argument", "--ros-args")
        _text(ros, "argument", "--log-level")
        _text(ros, "argument", "warn")
        _text(plugin, "update_rate", 50)
        plugin = ET.SubElement(model, "plugin", name="vehicle_manipulator_state",
                               filename="libgazebo_ros_joint_state_publisher.so")
        ros = ET.SubElement(plugin, "ros")
        _text(ros, "remapping", "joint_states:=/joint_states")
        _text(plugin, "update_rate", 20)
        for entry in data["joints"]:
            if entry.get("type") != "fixed":
                _text(plugin, "joint_name", PREFIX + entry["name"])
