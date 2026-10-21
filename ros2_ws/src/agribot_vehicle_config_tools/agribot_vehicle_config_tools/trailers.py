"""Passive articulated transport trailers with conservative swept footprints."""

import itertools
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from .model import ConfigError, _positive, _validate_pose, _vector


def items(config):
    return [item for item in config.get("implements", []) if item.get("implementType") == "farm_trailer"]


def validate(item, existing_frames):
    expected = dict(id="farm_trailer", parentFrameId="base_link", frameId="farm_trailer_link",
        workFrameId="farm_trailer_work_frame", mountMode="articulated_transport",
        meshFile="implement_farm_trailer_visual.obj", simulationOnly=True)
    if any(item.get(k) != v for k, v in expected.items()):
        raise ConfigError("invalid farm trailer identifiers or transport mode")
    _validate_pose(item.get("pose"), "trailer.pose")
    if any(abs(v) > 1e-8 for v in item["pose"]["rpy"]):
        raise ConfigError("nominal trailer mount requires a level neutral hitch")
    for key in ("massKg", "workingWidthM", "yawLimitRad", "pitchLimitRad"):
        _positive(item.get(key), "trailer." + key)
    if item["yawLimitRad"] > math.pi / 3:
        raise ConfigError("trailer yaw limit exceeds the verified transport range")
    if item["pitchLimitRad"] > .3:
        raise ConfigError("trailer pitch limit exceeds the transport range")
    _vector(item.get("workOffset"), 3, "trailer.workOffset")
    boxes = item.get("collisionBoxes")
    if not isinstance(boxes, list) or not boxes:
        raise ConfigError("trailer requires body collision boxes")
    for box in boxes:
        if not isinstance(box.get("name"), str) or not box["name"].isidentifier():
            raise ConfigError("invalid trailer collision name")
        _vector(box.get("center"), 3, "trailer.box.center")
        _vector(box.get("size"), 3, "trailer.box.size")
        for size in box["size"]:
            _positive(size, "trailer.box.size")
    wheels = item.get("wheels")
    if not isinstance(wheels, list) or len(wheels) != 4:
        raise ConfigError("trailer requires four passive wheels")
    if {w.get("id") for w in wheels} != {"front_left", "front_right", "rear_left", "rear_right"}:
        raise ConfigError("invalid or duplicate trailer wheel identifiers")
    frames = [item["frameId"], item["frameId"]+"_hitch", item["workFrameId"]]
    for wheel in wheels:
        _vector(wheel.get("position"), 3, "trailer.wheel.position")
        for key in ("radiusM", "widthM", "massKg"):
            _positive(wheel.get(key), "trailer.wheel." + key)
        if wheel.get("meshFile") != "trailer_" + wheel["id"] + ".obj":
            raise ConfigError("invalid trailer wheel mesh name")
        frames.append(item["frameId"] + "_" + wheel["id"])
    for frame in frames:
        if frame in existing_frames:
            raise ConfigError("duplicate trailer frame: " + frame)
        existing_frames.add(frame)


def _corners(box):
    return [[box["center"][i] + sign[i]*box["size"][i]/2 for i in range(3)]
            for sign in itertools.product((-1, 1), repeat=3)]


def collision_boxes(config, padding=0.):
    result = []
    for item in items(config):
        boxes = list(item["collisionBoxes"]) + [dict(center=w["position"],
            size=[2*w["radiusM"], w["widthM"], 2*w["radiusM"]]) for w in item["wheels"]]
        for box in boxes:
            pitched = []
            for x, y, z in _corners(box):
                angles = [-item["pitchLimitRad"], item["pitchLimitRad"], 0.]
                for angle in (math.atan2(z, x), math.atan2(-x, z)):
                    angles += [angle+n*math.pi for n in range(-2, 3)
                               if -item["pitchLimitRad"] <= angle+n*math.pi <= item["pitchLimitRad"]]
                pitched += [[math.cos(a)*x+math.sin(a)*z, y, -math.sin(a)*x+math.cos(a)*z] for a in angles]
            low = [min(p[i] for p in pitched) for i in range(3)]
            high = [max(p[i] for p in pitched) for i in range(3)]
            pitch_box = dict(center=[(a+b)/2 for a, b in zip(low, high)], size=[b-a for a, b in zip(low, high)])
            points = []
            for x, y, z in _corners(pitch_box):
                # Coordinate extrema occur at interval ends or stationary angles.
                candidates = [-item["yawLimitRad"], item["yawLimitRad"], 0.]
                for angle in (math.atan2(-y, x), math.atan2(x, y)):
                    candidates += [angle + n*math.pi for n in range(-2, 3)
                                   if -item["yawLimitRad"] <= angle+n*math.pi <= item["yawLimitRad"]]
                for yaw in candidates:
                    c, s = math.cos(yaw), math.sin(yaw)
                    points.append([c*x-s*y, s*x+c*y, z])
            pose = item["pose"]["xyz"]
            result.append([min(p[i] for p in points)+pose[i]-padding for i in range(3)] +
                          [max(p[i] for p in points)+pose[i]+padding for i in range(3)])
    return result


def _text(values):
    return " ".join(f"{v:.12g}" for v in values)


def _inertial(parent, mass, center, size, urdf=False):
    inertia = ET.SubElement(parent, "inertial")
    diagonal = [mass*sum(size[j]**2 for j in range(3) if j != i)/12 for i in range(3)]
    values = dict(zip(("ixx", "iyy", "izz"), map(str, diagonal)), ixy="0", ixz="0", iyz="0")
    if urdf:
        ET.SubElement(inertia, "origin", xyz=_text(center), rpy="0 0 0")
        ET.SubElement(inertia, "mass", value=str(mass))
        ET.SubElement(inertia, "inertia", **values)
    else:
        ET.SubElement(inertia, "pose").text = _text(center+[0, 0, 0])
        ET.SubElement(inertia, "mass").text = str(mass)
        tensor = ET.SubElement(inertia, "inertia")
        for key, value in values.items():
            ET.SubElement(tensor, key).text = value


def _uri(config, filename):
    return f"package://agribot_vehicle_description/generated/{config['vehicleId']}/models/{filename}"


def append_sdf(model, config):
    for item in items(config):
        pose, frame = item["pose"]["xyz"], item["frameId"]
        hitch_frame = frame+"_hitch"
        hitch_link = ET.SubElement(model, "link", name=hitch_frame)
        ET.SubElement(hitch_link, "pose").text = _text(pose+[0, 0, 0])
        _inertial(hitch_link, .5, [0, 0, 0], [.1, .1, .1])
        link = ET.SubElement(model, "link", name=frame)
        ET.SubElement(link, "pose").text = _text(pose+[0, 0, 0])
        box = item["collisionBoxes"][0]
        _inertial(link, item["massKg"], box["center"], box["size"])
        mesh = ET.SubElement(ET.SubElement(ET.SubElement(link, "visual", name="trailer_body"), "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = _uri(config, item["meshFile"])
        for box in item["collisionBoxes"]:
            collision = ET.SubElement(link, "collision", name=box["name"])
            ET.SubElement(collision, "pose").text = _text(box["center"]+[0, 0, 0])
            ET.SubElement(ET.SubElement(ET.SubElement(collision, "geometry"), "box"), "size").text = _text(box["size"])
        joint = ET.SubElement(model, "joint", name="farm_trailer_mount_joint", type="revolute")
        ET.SubElement(joint, "pose", relative_to="__model__").text = _text(pose+[0, 0, 0])
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = hitch_frame
        axis = ET.SubElement(joint, "axis")
        ET.SubElement(axis, "xyz").text = "0 0 1"
        limit = ET.SubElement(axis, "limit")
        ET.SubElement(limit, "lower").text = str(-item["yawLimitRad"])
        ET.SubElement(limit, "upper").text = str(item["yawLimitRad"])
        ET.SubElement(ET.SubElement(axis, "dynamics"), "damping").text = "20"
        pitch = ET.SubElement(model, "joint", name="farm_trailer_pitch_joint", type="revolute")
        ET.SubElement(pitch, "pose", relative_to="__model__").text = _text(pose+[0, 0, 0])
        ET.SubElement(pitch, "parent").text = hitch_frame
        ET.SubElement(pitch, "child").text = frame
        axis = ET.SubElement(pitch, "axis")
        ET.SubElement(axis, "xyz").text = "0 1 0"
        limit = ET.SubElement(axis, "limit")
        ET.SubElement(limit, "lower").text = str(-item["pitchLimitRad"])
        ET.SubElement(limit, "upper").text = str(item["pitchLimitRad"])
        ET.SubElement(ET.SubElement(axis, "dynamics"), "damping").text = "10"
        names = ["farm_trailer_mount_joint", "farm_trailer_pitch_joint"]
        for wheel in item["wheels"]:
            name = frame + "_" + wheel["id"]
            position = [a+b for a, b in zip(pose, wheel["position"])]
            child = ET.SubElement(model, "link", name=name)
            ET.SubElement(child, "pose").text = _text(position+[0, 0, 0])
            radius, width = wheel["radiusM"], wheel["widthM"]
            _inertial(child, wheel["massKg"], [0, 0, 0], [2*radius, width, 2*radius])
            mesh = ET.SubElement(ET.SubElement(ET.SubElement(child, "visual", name="wheel"), "geometry"), "mesh")
            ET.SubElement(mesh, "uri").text = _uri(config, wheel["meshFile"])
            collision = ET.SubElement(child, "collision", name="wheel_contact")
            ET.SubElement(collision, "pose").text = "0 0 0 1.570796326795 0 0"
            cylinder = ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder")
            ET.SubElement(cylinder, "radius").text = str(radius)
            ET.SubElement(cylinder, "length").text = str(width)
            friction = ET.SubElement(ET.SubElement(ET.SubElement(collision, "surface"), "friction"), "ode")
            ET.SubElement(friction, "mu").text = "0.8"
            ET.SubElement(friction, "mu2").text = "0.3"
            contact = ET.SubElement(ET.SubElement(collision.find("surface"), "contact"), "ode")
            for key, value in (("kp", "1000000"), ("kd", "100"), ("min_depth", "0.001"), ("max_vel", "0.1")):
                ET.SubElement(contact, key).text = value
            joint_name = name+"_joint"
            joint = ET.SubElement(model, "joint", name=joint_name, type="revolute")
            ET.SubElement(joint, "pose", relative_to="__model__").text = _text(position+[0, 0, 0])
            ET.SubElement(joint, "parent").text = frame
            ET.SubElement(joint, "child").text = name
            axis = ET.SubElement(joint, "axis")
            ET.SubElement(axis, "xyz").text = "0 1 0"
            limit = ET.SubElement(axis, "limit")
            ET.SubElement(limit, "lower").text = "-1e16"
            ET.SubElement(limit, "upper").text = "1e16"
            ET.SubElement(ET.SubElement(axis, "dynamics"), "damping").text = "0.02"
            names.append(joint_name)
        plugin = ET.SubElement(model, "plugin", name="trailer_joint_states", filename="libgazebo_ros_joint_state_publisher.so")
        ET.SubElement(plugin, "update_rate").text = "30"
        for name in names:
            ET.SubElement(plugin, "joint_name").text = name


def append_urdf(robot, config):
    for item in items(config):
        frame = item["frameId"]
        hitch_frame = frame+"_hitch"
        hitch = ET.SubElement(robot, "link", name=hitch_frame)
        _inertial(hitch, .5, [0, 0, 0], [.1, .1, .1], True)
        link = ET.SubElement(robot, "link", name=frame)
        box = item["collisionBoxes"][0]
        _inertial(link, item["massKg"], box["center"], box["size"], True)
        ET.SubElement(ET.SubElement(ET.SubElement(link, "visual"), "geometry"), "mesh", filename=_uri(config, item["meshFile"]))
        for box in item["collisionBoxes"]:
            collision = ET.SubElement(link, "collision", name=box["name"])
            ET.SubElement(collision, "origin", xyz=_text(box["center"]), rpy="0 0 0")
            ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=_text(box["size"]))
        joint = ET.SubElement(robot, "joint", name="farm_trailer_mount_joint", type="revolute")
        ET.SubElement(joint, "origin", xyz=_text(item["pose"]["xyz"]), rpy="0 0 0")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=hitch_frame)
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(joint, "limit", lower=str(-item["yawLimitRad"]), upper=str(item["yawLimitRad"]), effort="1000", velocity="2")
        pitch = ET.SubElement(robot, "joint", name="farm_trailer_pitch_joint", type="revolute")
        ET.SubElement(pitch, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(pitch, "parent", link=hitch_frame)
        ET.SubElement(pitch, "child", link=frame)
        ET.SubElement(pitch, "axis", xyz="0 1 0")
        ET.SubElement(pitch, "limit", lower=str(-item["pitchLimitRad"]), upper=str(item["pitchLimitRad"]), effort="1000", velocity="2")
        for wheel in item["wheels"]:
            name = frame+"_"+wheel["id"]
            link = ET.SubElement(robot, "link", name=name)
            _inertial(link, wheel["massKg"], [0, 0, 0], [2*wheel["radiusM"], wheel["widthM"], 2*wheel["radiusM"]], True)
            ET.SubElement(ET.SubElement(ET.SubElement(link, "visual"), "geometry"), "mesh", filename=_uri(config, wheel["meshFile"]))
            collision = ET.SubElement(link, "collision")
            ET.SubElement(collision, "origin", xyz="0 0 0", rpy="1.570796326795 0 0")
            ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder", radius=str(wheel["radiusM"]), length=str(wheel["widthM"]))
            joint = ET.SubElement(robot, "joint", name=name+"_joint", type="continuous")
            ET.SubElement(joint, "parent", link=frame)
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz=_text(wheel["position"]), rpy="0 0 0")
            ET.SubElement(joint, "axis", xyz="0 1 0")
        ET.SubElement(robot, "link", name=item["workFrameId"])
        joint = ET.SubElement(robot, "joint", name="farm_trailer_work_joint", type="fixed")
        ET.SubElement(joint, "parent", link=frame)
        ET.SubElement(joint, "child", link=item["workFrameId"])
        ET.SubElement(joint, "origin", xyz=_text(item["workOffset"]), rpy="0 0 0")


def copy_meshes(config, source_dir, output_dir):
    for item in items(config):
        for filename in [item["meshFile"]] + [w["meshFile"] for w in item["wheels"]]:
            source = Path(source_dir)/filename
            material = source.with_suffix(".mtl")
            if not source.is_file() or not material.is_file():
                raise ConfigError("missing trailer mesh/material: " + filename)
            for line in source.read_text().splitlines():
                if line.startswith("mtllib ") and line.split()[1:] != [material.name]:
                    raise ConfigError("trailer OBJ has external material dependencies")
            if any(line.strip().lower().startswith(("map_", "bump ", "disp ", "decal ")) for line in material.read_text().splitlines()):
                raise ConfigError("trailer material must be self-contained")
            for path in (source, material):
                shutil.copy2(path, Path(output_dir)/path.name)
