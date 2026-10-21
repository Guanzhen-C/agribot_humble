"""Fixed mounted CAD implements, sharing collision geometry with navigation."""

import itertools
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from . import sprayer, trailers
from .model import ConfigError, _positive, _validate_pose, _vector, matrix_from_rpy


MOUNTED_TYPES = {"compact_chisel_plow", "mounted_disc_harrow", "groove_maker", "mounted_cultivator",
                 "forest_chain_crusher", "inter_row_milling_cutter", "front_sickle_bar",
                 "garlic_planter", "mulch_planter", "combine_header"}


def _sprayer_config(config):
    return {"implements": [item for item in config.get("implements", [])
                           if item["implementType"] == "sprayer"]}


def mounted_items(config):
    return [item for item in config.get("implements", [])
            if item["implementType"] in MOUNTED_TYPES]


def validate_implements(config, existing_frames):
    items = config.get("implements", [])
    if not isinstance(items, list) or len(items) > 1:
        raise ConfigError("implements must contain at most one mounted implement")
    for item in items:
        if not isinstance(item, dict):
            raise ConfigError("implement must be an object")
        kind = item.get("implementType")
        if kind == "sprayer":
            sprayer.validate_implements({"implements": [item]}, existing_frames)
            continue
        if kind == "farm_trailer":
            trailers.validate(item, existing_frames)
            continue
        if kind not in MOUNTED_TYPES:
            raise ConfigError("unsupported mounted implement type")
        expected = {"id": kind, "parentFrameId": "base_link", "frameId": kind + "_link",
                    "workFrameId": kind + "_work_frame", "mountMode": "fixed_transport",
                    "meshFile": "implement_" + kind + "_visual.obj", "simulationOnly": True}
        if any(item.get(key) != value for key, value in expected.items()):
            raise ConfigError("mounted implement identifiers or transport mode are invalid")
        _validate_pose(item.get("pose"), "implement.pose")
        _vector(item.get("workOffset"), 3, "implement.workOffset")
        for key in ("massKg", "workingWidthM"):
            _positive(item.get(key), "implement." + key)
        boxes = item.get("collisionBoxes")
        if not isinstance(boxes, list) or not boxes:
            raise ConfigError("mounted implement requires collisionBoxes")
        names = set()
        for box in boxes:
            if not isinstance(box, dict) or not isinstance(box.get("name"), str):
                raise ConfigError("invalid implement collision box")
            name = box["name"]
            if not name.isidentifier() or name in names:
                raise ConfigError("invalid or duplicate implement collision name")
            names.add(name)
            _vector(box.get("center"), 3, "implement.collision.center")
            _vector(box.get("size"), 3, "implement.collision.size")
            if "visualize" in box and not isinstance(box["visualize"], bool):
                raise ConfigError("implement collision visualize must be boolean")
            for size in box["size"]:
                _positive(size, "implement.collision.size")
        for key in ("frameId", "workFrameId"):
            if item[key] in existing_frames:
                raise ConfigError("duplicate implement frame: " + item[key])
            existing_frames.add(item[key])


def collision_boxes(config, padding=0.0):
    boxes = sprayer.collision_boxes(_sprayer_config(config), padding) + trailers.collision_boxes(config, padding)
    for item in mounted_items(config):
        rotation = matrix_from_rpy(item["pose"]["rpy"])
        translation = item["pose"]["xyz"]
        for box in item["collisionBoxes"]:
            corners = [[box["center"][i] + sign[i] * box["size"][i] / 2 for i in range(3)]
                       for sign in itertools.product((-1, 1), repeat=3)]
            moved = [[sum(rotation[i][j] * point[j] for j in range(3)) + translation[i]
                      for i in range(3)] for point in corners]
            boxes.append([min(point[i] for point in moved) - padding for i in range(3)] +
                         [max(point[i] for point in moved) + padding for i in range(3)])
    return boxes


def _text(values):
    return " ".join(f"{value:.12g}" for value in values)


def _inertia(item):
    boxes = item["collisionBoxes"]
    low = [min(box["center"][i] - box["size"][i] / 2 for box in boxes) for i in range(3)]
    high = [max(box["center"][i] + box["size"][i] / 2 for box in boxes) for i in range(3)]
    center = [(a + b) / 2 for a, b in zip(low, high)]
    size = [b - a for a, b in zip(low, high)]
    diagonal = [item["massKg"] * sum(size[j] ** 2 for j in range(3) if j != i) / 12
                for i in range(3)]
    return center, diagonal


def mesh_uri(config, item):
    return ("package://agribot_vehicle_description/generated/" + config["vehicleId"] +
            "/models/" + item["meshFile"])


def append_sdf(model, config):
    sprayer.append_sdf(model, _sprayer_config(config))
    trailers.append_sdf(model, config)
    for item in mounted_items(config):
        link = ET.SubElement(model, "link", name=item["frameId"])
        ET.SubElement(link, "pose").text = _text(item["pose"]["xyz"] + item["pose"]["rpy"])
        center, diagonal = _inertia(item)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "pose").text = _text(center + [0, 0, 0])
        ET.SubElement(inertial, "mass").text = str(item["massKg"])
        tensor = ET.SubElement(inertial, "inertia")
        for name, value in zip(("ixx", "iyy", "izz"), diagonal):
            ET.SubElement(tensor, name).text = str(value)
        for name in ("ixy", "ixz", "iyz"):
            ET.SubElement(tensor, name).text = "0"
        visual = ET.SubElement(link, "visual", name=item["id"] + "_visual")
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = mesh_uri(config, item)
        for box in item["collisionBoxes"]:
            collision = ET.SubElement(link, "collision", name=box["name"])
            ET.SubElement(collision, "pose").text = _text(box["center"] + [0, 0, 0])
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(ET.SubElement(geometry, "box"), "size").text = _text(box["size"])
            if box.get("visualize", False):
                visible = ET.SubElement(link, "visual", name=box["name"] + "_visual")
                ET.SubElement(visible, "pose").text = _text(box["center"] + [0, 0, 0])
                ET.SubElement(ET.SubElement(ET.SubElement(visible, "geometry"), "box"), "size").text = _text(box["size"])
                material = ET.SubElement(visible, "material")
                for channel in ("ambient", "diffuse"):
                    ET.SubElement(material, channel).text = "0.18 0.20 0.22 1"
        joint = ET.SubElement(model, "joint", name=item["id"] + "_mount_joint", type="fixed")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = item["frameId"]


def append_urdf(robot, config):
    sprayer.append_urdf(robot, _sprayer_config(config))
    trailers.append_urdf(robot, config)
    for item in mounted_items(config):
        link = ET.SubElement(robot, "link", name=item["frameId"])
        center, diagonal = _inertia(item)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=_text(center), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(item["massKg"]))
        ET.SubElement(inertial, "inertia", ixx=str(diagonal[0]), iyy=str(diagonal[1]),
                      izz=str(diagonal[2]), ixy="0", ixz="0", iyz="0")
        visual = ET.SubElement(link, "visual", name=item["id"] + "_visual")
        ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=mesh_uri(config, item))
        for box in item["collisionBoxes"]:
            collision = ET.SubElement(link, "collision", name=box["name"])
            ET.SubElement(collision, "origin", xyz=_text(box["center"]), rpy="0 0 0")
            ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=_text(box["size"]))
            if box.get("visualize", False):
                visible = ET.SubElement(link, "visual", name=box["name"] + "_visual")
                ET.SubElement(visible, "origin", xyz=_text(box["center"]), rpy="0 0 0")
                ET.SubElement(ET.SubElement(visible, "geometry"), "box", size=_text(box["size"]))
                material = ET.SubElement(visible, "material", name="implement_hitch_steel")
                ET.SubElement(material, "color", rgba="0.18 0.20 0.22 1")
        joint = ET.SubElement(robot, "joint", name=item["id"] + "_mount_joint", type="fixed")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=item["frameId"])
        ET.SubElement(joint, "origin", xyz=_text(item["pose"]["xyz"]), rpy=_text(item["pose"]["rpy"]))
        ET.SubElement(robot, "link", name=item["workFrameId"])
        work = ET.SubElement(robot, "joint", name=item["id"] + "_work_joint", type="fixed")
        ET.SubElement(work, "parent", link=item["frameId"])
        ET.SubElement(work, "child", link=item["workFrameId"])
        ET.SubElement(work, "origin", xyz=_text(item["workOffset"]), rpy="0 0 0")


def copy_meshes(config, source_dir, output_dir):
    """Only self-contained, local OBJ/MTL assets may enter the generated bundle."""
    trailers.copy_meshes(config, source_dir, output_dir)
    for item in mounted_items(config):
        filename = item["meshFile"]
        if Path(filename).name != filename:
            raise ConfigError("implement mesh must be an export-local file")
        mesh = Path(source_dir) / filename
        material = mesh.with_suffix(".mtl")
        if not mesh.is_file() or not material.is_file():
            raise ConfigError("missing implement OBJ/MTL: " + filename)
        boxes = item["collisionBoxes"]
        vertices = 0
        with mesh.open(encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("mtllib ") and line.split()[1:] != [material.name]:
                    raise ConfigError("implement OBJ has an external material dependency")
                if line.startswith("v "):
                    xyz = [float(value) for value in line.split()[1:4]]
                    if len(xyz) != 3 or not any(all(
                            abs(xyz[i] - box["center"][i]) <= box["size"][i] / 2 + 1e-5
                            for i in range(3)) for box in boxes):
                        raise ConfigError("implement visual exceeds its collision envelope")
                    vertices += 1
        if not vertices:
            raise ConfigError("empty implement mesh")
        with material.open(encoding="utf-8") as stream:
            if any(line.strip().lower().startswith(("map_", "bump ", "disp ", "decal ")) for line in stream):
                raise ConfigError("implement material must be self-contained (no external textures)")
        for path in (mesh, material):
            shutil.copy2(path, Path(output_dir) / path.name)
