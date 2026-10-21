"""Fork visuals and pivots for passive casters or controlled steering assemblies."""

from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def wheels(config):
    return [wheel for wheel in config["geometry"]["wheelCenters"] if "caster" in wheel]


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def copy_meshes(config, source, output):
    for wheel in config["geometry"]["wheelCenters"]:
        spec = wheel.get("caster", wheel.get("steeringAssembly"))
        if not spec:
            continue
        name = Path(spec["meshUri"]).name
        for filename in (name, Path(name).with_suffix(".mtl").name):
            path = source / filename
            if not path.is_file():
                raise ValueError("Missing caster mesh/material: " + filename)
            shutil.copy2(path, output / filename)


def append_sdf(model, config):
    for wheel in config["geometry"]["wheelCenters"]:
        spec = wheel.get("steeringAssembly")
        if not spec:
            continue
        key = wheel["id"] + "_steering"
        link = model.find(f"./link[@name='{key}_link']")
        link.find("pose").text = numbers(spec["pivotPosition"]) + " 0 0 0"
        joint = model.find(f"./joint[@name='{key}_joint']")
        joint.find("pose").text = numbers(spec["pivotPosition"]) + " 0 0 0"
        ET.SubElement(ET.SubElement(joint.find("axis"), "dynamics"), "damping").text = str(spec["damping"])
        visual = ET.SubElement(link, "visual", name="fork_visual")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(visual.find("./geometry/mesh"), "uri").text = spec["meshUri"]
    for wheel in wheels(config):
        spec = wheel["caster"]
        key = wheel["id"] + "_caster"
        pivot = spec["pivotPosition"]
        link = ET.SubElement(model, "link", name=key + "_link")
        ET.SubElement(link, "pose", relative_to="__model__").text = numbers(pivot) + " 0 0 0"
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = str(spec["massKg"])
        inertia = ET.SubElement(inertial, "inertia")
        for axis in ("ixx", "iyy", "izz"):
            ET.SubElement(inertia, axis).text = str(spec["massKg"] * .01)
        visual = ET.SubElement(link, "visual", name="fork_visual")
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = spec["meshUri"]
        joint = ET.SubElement(model, "joint", name=key + "_joint", type="revolute")
        ET.SubElement(joint, "pose", relative_to="__model__").text = numbers(pivot) + " 0 0 0"
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = key + "_link"
        axis = ET.SubElement(joint, "axis")
        ET.SubElement(axis, "xyz").text = "0 0 1"
        limit = ET.SubElement(axis, "limit")
        for name, value in (("lower", -1e16), ("upper", 1e16), ("effort", 20), ("velocity", 20)):
            ET.SubElement(limit, name).text = str(value)
        ET.SubElement(ET.SubElement(axis, "dynamics"), "damping").text = str(spec["damping"])
        model.find(f"./joint[@name='{wheel['id']}_wheel_joint']/parent").text = key + "_link"


def append_urdf(robot, config):
    for wheel in config["geometry"]["wheelCenters"]:
        spec = wheel.get("steeringAssembly")
        if not spec:
            continue
        key = wheel["id"] + "_steering"
        link = robot.find(f"./link[@name='{key}_link']")
        visual = ET.SubElement(link, "visual")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=spec["meshUri"])
        robot.find(f"./joint[@name='{key}_joint']/origin").set("xyz", numbers(spec["pivotPosition"]))
        spin = robot.find(f"./joint[@name='{wheel['id']}_wheel_joint']/origin")
        spin.set("xyz", numbers([a-b for a,b in zip(wheel["position"], spec["pivotPosition"])]))
    for wheel in wheels(config):
        spec = wheel["caster"]
        key = wheel["id"] + "_caster"
        link = ET.SubElement(robot, "link", name=key + "_link")
        visual = ET.SubElement(link, "visual")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=spec["meshUri"])
        joint = ET.SubElement(robot, "joint", name=key + "_joint", type="continuous")
        ET.SubElement(joint, "origin", xyz=numbers(spec["pivotPosition"]), rpy="0 0 0")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=key + "_link")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(joint, "dynamics", damping=str(spec["damping"]))
        spin = robot.find(f"./joint[@name='{wheel['id']}_wheel_joint']")
        spin.find("parent").set("link", key + "_link")
        spin.find("origin").set("xyz", numbers([a-b for a,b in zip(wheel["position"], spec["pivotPosition"])]))
