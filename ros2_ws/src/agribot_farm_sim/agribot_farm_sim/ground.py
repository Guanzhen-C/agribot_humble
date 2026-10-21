"""Material-independent ground contacts for the configured simulation worlds."""

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def is_navigation_surface(item, scene):
    if item.get("navigationSurface") is True:
        return True
    # This older importer did not annotate its floor/grass/pathway meshes.
    return scene == "greenhouse" and re.fullmatch(
        r"greenhouse_(?:base|greenhouse|grass|pathway)_link(?:\.\d+)?",
        item["name"],
    ) is not None


def set_ground_contact(collision):
    """Let the vehicle's tire contact, not a visual material, limit traction."""
    surface = collision.find("surface")
    if surface is not None:
        collision.remove(surface)
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
    for name in ("mu", "mu2"):
        ET.SubElement(friction, name).text = "100"
    contact = ET.SubElement(ET.SubElement(surface, "contact"), "ode")
    for name, value in (("kp", "1000000000"), ("kd", "1"),
                        ("max_vel", "0.1"), ("min_depth", "0.001")):
        ET.SubElement(contact, name).text = value


def _split_collision_mesh(source, destinations, surfaces, raw_names=False):
    streams = {key: path.open("w", encoding="utf-8") for key, path in destinations.items()}
    counts = {key: 0 for key in streams}
    seen = set()
    selected = "obstacles"
    names = sorted(surfaces, key=len, reverse=True)
    try:
        for stream in streams.values():
            stream.write("mtllib contact_material.mtl\n")
        with source.open(encoding="utf-8") as original:
            for line in original:
                fields = line.split()
                if not fields:
                    continue
                if fields[0] == "o":
                    name = " ".join(fields[1:])
                    match = next((item for item in names if name == item + "_collision"
                                  or name.startswith(item + "_collision_")
                                  or (raw_names and (name == item or name.startswith(item + "_")))), None)
                    selected = "obstacles" if match is None else surfaces[match]
                    if match is not None:
                        seen.add(match)
                if fields[0] == "f":
                    if selected in streams:
                        streams[selected].write(line)
                        counts[selected] += 1
                elif fields[0] in ("v", "vt", "vn"):
                    # Preserve global OBJ indices, including negative face indices.
                    for stream in streams.values():
                        stream.write(line)
                elif fields[0] in ("o", "g", "s") and selected in streams:
                    streams[selected].write(line)
                    if fields[0] == "o":
                        streams[selected].write("usemtl ground_contact\n")
    finally:
        for stream in streams.values():
            stream.close()
    if seen != set(surfaces):
        raise ValueError(f"Ground objects missing from collision mesh: {sorted(set(surfaces) - seen)}")
    return counts


def apply_prepared_ground_policy(root, source, directory, scene):
    """Replace authored flat floor patches by solid slabs; retain uneven terrain."""
    link = root.find("./world/model[@name='agricultural_environment']/link[@name='environment']")
    if link is None or link.find("collision/geometry/mesh/uri") is None:
        raise ValueError("Expected the prepared scene's combined environment collision")
    original = link.find("collision")
    mesh = Path(original.findtext("geometry/mesh/uri").removeprefix("file://"))
    surfaces = {}
    patches = []
    for item in source["objects"]:
        if not is_navigation_surface(item, scene):
            continue
        low, high = item["minimum"], item["maximum"]
        if (len(low) != 3 or len(high) != 3 or not all(math.isfinite(v) for v in [*low, *high])
                or any(a > b for a, b in zip(low, high))):
            raise ValueError("Invalid navigation surface bounds")
        planar = high[2] - low[2] <= 1e-4 or (
            scene == "greenhouse" and item["name"] == "greenhouse_base_link")
        surfaces[item["name"]] = "slab" if planar else "terrain"
        if planar:
            if min(high[0] - low[0], high[1] - low[1]) <= 0:
                raise ValueError("Ground slab has no horizontal area")
            patches.append(item)
    if not surfaces:
        raise ValueError("Scene has no explicitly identified navigation surface")
    (directory / "contact_material.mtl").write_text(
        "newmtl ground_contact\nKa 0.5 0.5 0.5\nKd 0.5 0.5 0.5\n", encoding="utf-8")
    outputs = {key: directory / (key + "_collision.obj") for key in ("obstacles", "terrain")}
    counts = _split_collision_mesh(mesh, outputs, surfaces, raw_names=scene == "wide_headland")
    link.remove(original)
    for key, path in outputs.items():
        if counts[key] == 0:
            path.unlink()
            continue
        collision = ET.SubElement(link, "collision", name=key + "_collision")
        ET.SubElement(ET.SubElement(ET.SubElement(collision, "geometry"), "mesh"), "uri").text = path.as_uri()
        if key == "terrain":
            set_ground_contact(collision)
    for index, item in enumerate(patches):
        low, high = item["minimum"], item["maximum"]
        collision = ET.SubElement(link, "collision", name=f"drivable_ground_{index}")
        thickness = 0.5
        ET.SubElement(collision, "pose").text = " ".join(map(str, [
            (low[0] + high[0]) / 2, (low[1] + high[1]) / 2, high[2] - thickness / 2, 0, 0, 0]))
        ET.SubElement(ET.SubElement(ET.SubElement(collision, "geometry"), "box"), "size").text = (
            f"{high[0] - low[0]} {high[1] - low[1]} {thickness}")
        set_ground_contact(collision)
    return {"version": 1, "materialIndependent": True, "solidFlatPatches": len(patches),
            "unevenTerrainFaces": counts["terrain"], "obstacleFaces": counts["obstacles"],
            "surfaces": sorted(surfaces), "visualsAndObstacleGeometryUnchanged": True}


def prepare_orchard_ground(world_file, directory):
    """The orchard already has separate, uneven ground; keep its geometry."""
    root = ET.parse(world_file)
    collision = root.find("./world/model[@name='orchard_geometry']/link/collision[@name='ground_collision']")
    if collision is None:
        raise ValueError("Orchard ground collision is missing")
    set_ground_contact(collision)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / "world.sdf"
    root.write(target, encoding="utf-8", xml_declaration=True)
    return str(target)
