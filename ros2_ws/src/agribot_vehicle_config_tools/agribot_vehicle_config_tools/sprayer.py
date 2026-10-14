"""Shared nominal sprayer geometry for Unity exports and simulation models."""

import itertools
import math
import xml.etree.ElementTree as ET

from .model import ConfigError, matrix_from_rpy


def default_sprayer(config):
    body = config["geometry"]["bodyExtents"]
    collision = config["collision"]
    top = collision["center"][2] + collision["size"][2] / 2
    return {
        "id": "sprayer", "implementType": "sprayer", "parentFrameId": "base_link",
        "frameId": "sprayer_link", "workFrameId": "sprayer_work_frame",
        "pose": {"xyz": [-body["rearM"] - .16, 0.0, max(.14, min(.45, top * .55))],
                 "rpy": [0.0, 0.0, 0.0], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        "workingWidthM": max(.9, body["leftM"] + body["rightM"] + .2),
        "massKg": 8.0,
    }


def validate_implements(config, existing_frames):
    items = config.get("implements", [])
    if not isinstance(items, list) or len(items) > 1:
        raise ConfigError("implements must be an array containing at most one sprayer")
    for item in items:
        if not isinstance(item, dict):
            raise ConfigError("implement must be an object")
        expected = {"id": "sprayer", "implementType": "sprayer", "parentFrameId": "base_link",
                    "frameId": "sprayer_link", "workFrameId": "sprayer_work_frame"}
        if any(item.get(key) != value for key, value in expected.items()):
            raise ConfigError("unsupported implement type or sprayer frame identifiers")
        from .model import _positive, _validate_pose
        for key in ("workingWidthM", "massKg"):
            _positive(item.get(key), "implements[0]." + key)
        _validate_pose(item.get("pose"), "implements[0].pose")
        for key in ("frameId", "workFrameId"):
            if item[key] in existing_frames:
                raise ConfigError("duplicate implement frame: " + item[key])
            existing_frames.add(item[key])


def parts(item):
    """Boxes are shared by visuals, physical collisions and the self-return mask."""
    width = item["workingWidthM"]
    result = [
        ("hitch", [.075, 0, 0], [.23, .06, .06], [.20, .23, .25, 1]),
        ("tank_cradle", [-.12, 0, .035], [.32, .42, .04], [.20, .23, .25, 1]),
        ("tank", [-.12, 0, .225], [.28, .38, .34], [.12, .48, .72, 1]),
        ("cap", [-.12, 0, .415], [.08, .08, .04], [.10, .12, .14, 1]),
        ("boom_mount", [-.29, 0, .015], [.045, .045, .16], [.20, .23, .25, 1]),
        ("boom", [-.29, 0, -.045], [.055, width, .04], [.60, .65, .68, 1]),
    ]
    for index in range(7):
        result.append((f"nozzle_{index}", [-.29, width * (index / 6 - .5), -.085],
                       [.035, .035, .045], [.94, .68, .12, 1]))
    return result


def work_offset():
    return [-.29, 0.0, -.11]


def collision_boxes(config, padding=0.0):
    boxes = []
    for item in config.get("implements", []):
        rotation = matrix_from_rpy(item["pose"]["rpy"])
        translation = item["pose"]["xyz"]
        for _, center, size, _ in parts(item):
            corners = [[center[i] + sign[i] * size[i] / 2 for i in range(3)]
                       for sign in itertools.product((-1, 1), repeat=3)]
            moved = [[sum(rotation[i][j] * point[j] for j in range(3)) + translation[i]
                      for i in range(3)] for point in corners]
            boxes.append([min(point[i] for point in moved) - padding for i in range(3)] +
                         [max(point[i] for point in moved) + padding for i in range(3)])
    return boxes


def _text(values):
    return " ".join(f"{value:.12g}" for value in values)


def _inertia(item):
    shape = parts(item)
    minimum = [min(center[i] - size[i] / 2 for _, center, size, _ in shape) for i in range(3)]
    maximum = [max(center[i] + size[i] / 2 for _, center, size, _ in shape) for i in range(3)]
    center = [(low + high) / 2 for low, high in zip(minimum, maximum)]
    size = [high - low for low, high in zip(minimum, maximum)]
    diagonal = [item["massKg"] * sum(size[j] ** 2 for j in range(3) if j != i) / 12
                for i in range(3)]
    return center, diagonal


def append_sdf(model, config):
    for item in config.get("implements", []):
        link = ET.SubElement(model, "link", name=item["frameId"])
        ET.SubElement(link, "pose").text = _text(item["pose"]["xyz"] + item["pose"]["rpy"])
        center, diagonal = _inertia(item)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "pose").text = _text(center + [0, 0, 0])
        ET.SubElement(inertial, "mass").text = str(item["massKg"])
        inertia = ET.SubElement(inertial, "inertia")
        for name, value in zip(("ixx", "iyy", "izz"), diagonal):
            ET.SubElement(inertia, name).text = str(value)
        for name in ("ixy", "ixz", "iyz"):
            ET.SubElement(inertia, name).text = "0"
        for name, center, size, color in parts(item):
            for tag in ("visual", "collision"):
                block = ET.SubElement(link, tag, name="sprayer_" + name + "_" + tag)
                ET.SubElement(block, "pose").text = _text(center + [0, 0, 0])
                geometry = ET.SubElement(block, "geometry")
                ET.SubElement(ET.SubElement(geometry, "box"), "size").text = _text(size)
                if tag == "visual":
                    material = ET.SubElement(block, "material")
                    for channel in ("ambient", "diffuse"):
                        ET.SubElement(material, channel).text = _text(color)
        joint = ET.SubElement(model, "joint", name="sprayer_mount_joint", type="fixed")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = item["frameId"]


def append_urdf(robot, config):
    for item in config.get("implements", []):
        link = ET.SubElement(robot, "link", name=item["frameId"])
        center, diagonal = _inertia(item)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=_text(center), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(item["massKg"]))
        ET.SubElement(inertial, "inertia", ixx=str(diagonal[0]), iyy=str(diagonal[1]),
                      izz=str(diagonal[2]), ixy="0", ixz="0", iyz="0")
        for name, center, size, color in parts(item):
            for tag in ("visual", "collision"):
                block = ET.SubElement(link, tag, name="sprayer_" + name + "_" + tag)
                ET.SubElement(block, "origin", xyz=_text(center), rpy="0 0 0")
                ET.SubElement(ET.SubElement(block, "geometry"), "box", size=_text(size))
                if tag == "visual":
                    material = ET.SubElement(block, "material", name="sprayer_" + name)
                    ET.SubElement(material, "color", rgba=_text(color))
        joint = ET.SubElement(robot, "joint", name="sprayer_mount_joint", type="fixed")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=item["frameId"])
        ET.SubElement(joint, "origin", xyz=_text(item["pose"]["xyz"]), rpy=_text(item["pose"]["rpy"]))
        ET.SubElement(robot, "link", name=item["workFrameId"])
        work = ET.SubElement(robot, "joint", name="sprayer_work_joint", type="fixed")
        ET.SubElement(work, "parent", link=item["frameId"])
        ET.SubElement(work, "child", link=item["workFrameId"])
        ET.SubElement(work, "origin", xyz=_text(work_offset()), rpy="0 0 0")
