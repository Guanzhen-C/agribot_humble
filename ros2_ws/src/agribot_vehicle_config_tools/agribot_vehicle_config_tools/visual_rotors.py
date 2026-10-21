"""Optional CAD wheel animation without adding simulated mass or contacts."""

import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def rotors(config):
    items = config.get("metadata", {}).get("simulationVisualRotors", [])
    if not isinstance(items, list):
        raise ValueError("simulationVisualRotors must be a list")
    wheels = {wheel["id"] for wheel in config["geometry"]["wheelCenters"]}
    names = set()
    for item in items:
        key = item.get("id", "")
        if not key.isidentifier() or key in names or item.get("sourceWheel") not in wheels:
            raise ValueError("Invalid visual rotor identifier or source wheel")
        names.add(key)
        center = item.get("center", [])
        ratio = item.get("ratio")
        if (len(center) != 3 or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in center)
                or isinstance(ratio, bool) or not isinstance(ratio, (int, float))
                or not math.isfinite(ratio) or ratio <= 0):
            raise ValueError("Invalid visual rotor center or ratio")
        if item.get("meshFile") != "rotor_" + key + ".obj":
            raise ValueError("Visual rotor mesh must be an export-local OBJ")
    return items


def uri(config, filename):
    return config["visuals"][0]["meshUri"].rsplit("/", 1)[0] + "/" + filename


def append_urdf(robot, config):
    for item in rotors(config):
        key = "display_" + item["id"]
        link = ET.SubElement(robot, "link", name=key + "_link")
        mesh = ET.SubElement(ET.SubElement(ET.SubElement(link, "visual"), "geometry"), "mesh")
        mesh.set("filename", uri(config, item["meshFile"]))
        joint = ET.SubElement(robot, "joint", name=key + "_joint", type="continuous")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=key + "_link")
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, item["center"])), rpy="0 0 0")
        ET.SubElement(joint, "axis", xyz="0 1 0")
        ET.SubElement(joint, "mimic", joint=item["sourceWheel"] + "_wheel_joint",
                      multiplier=str(item["ratio"]), offset="0")


def append_sdf(model, config):
    for item in rotors(config):
        visual = ET.SubElement(model.find("link[@name='base_link']"), "visual", name="display_" + item["id"])
        ET.SubElement(visual, "pose").text = " ".join(map(str, item["center"])) + " 0 0 0"
        ET.SubElement(ET.SubElement(ET.SubElement(visual, "geometry"), "mesh"), "uri").text = uri(config, item["meshFile"])
        plugin = ET.SubElement(visual, "plugin", name="wheel_animation", filename="libagribot_simulation_visual_rotor.so")
        ET.SubElement(plugin, "source_link").text = item["sourceWheel"] + "_wheel_link"
        ET.SubElement(plugin, "ratio").text = str(item["ratio"])


def copy_assets(config, source, output):
    for item in rotors(config):
        path = source / item["meshFile"]
        if not path.is_file():
            raise ValueError("Missing visual rotor: " + path.name)
        shutil.copy2(path, output / path.name)
    material = source / "vehicle_visual.mtl"
    if material.is_file():
        for line in material.read_text().splitlines():
            if not line.strip().startswith("map_Kd "):
                continue
            filename = line.strip()[7:].strip()
            if Path(filename).name != filename or Path(filename).suffix.lower() not in {".png", ".jpg", ".jpeg"}:
                raise ValueError("Body texture must be an export-local PNG/JPEG")
            path = source / filename
            if not path.is_file():
                raise ValueError("Missing body texture: " + filename)
            shutil.copy2(path, output / filename)
