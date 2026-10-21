"""Align Car3's four contact cylinders with its inspected visible idlers."""

import copy
import math
from pathlib import Path

from .visual_belts import BeltPath


def _vertices(path):
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "v":
                point = tuple(float(value) for value in fields[1:4])
                if len(point) != 3 or not all(math.isfinite(value) for value in point):
                    raise ValueError("Invalid contact reference mesh vertex")
                yield point


def align_car3_contacts(config, meshes):
    """Keep the base frame/body/sensors fixed, changing only contact geometry."""
    if config.get("vehicleId") != "unity_car3" or config.get("vehicleType") != "tracked":
        raise ValueError("This inspected contact profile is only for tracked Car3")
    if config.get("metadata", {}).get("simulationTrackContactGeometry"):
        raise ValueError("Contact geometry has already been aligned")
    result = copy.deepcopy(config)
    geometry = result["geometry"]
    metadata = result["metadata"]
    rotors = {item["id"]: item for item in metadata["simulationVisualRotors"]}
    belts = {item["id"]: item for item in metadata["simulationVisualBelts"]}
    wheels = {item["id"]: item for item in geometry["wheelCenters"]}
    old_radii = {key: item.get("radiusM") or geometry["wheelRadiusM"] for key, item in wheels.items()}
    audit = []
    for side in ("left", "right"):
        belt = belts[side]
        path = BeltPath(belt["points"])
        bottom = math.inf
        y_min, y_max = math.inf, -math.inf
        for part in belt["segments"]:
            _, z, angle = path.sample(part["distance"])
            for vx, vy, vz in _vertices(Path(meshes) / part["meshFile"]):
                bottom = min(bottom, z - math.sin(angle) * vx + math.cos(angle) * vz)
                y_min, y_max = min(y_min, vy), max(y_max, vy)
        if not math.isfinite(bottom) or y_max <= y_min:
            raise ValueError("Missing tread support envelope")
        for axle, suffix in (("front", "front"), ("rear", "back")):
            key = axle + "_" + side
            rotor = rotors[side + "_idler_" + suffix]
            center = list(rotor["center"])
            radius = center[2] - bottom
            core = list(_vertices(Path(meshes) / rotor["meshFile"]))
            core_radius = max(math.hypot(point[0], point[2]) for point in core)
            if not .0 < core_radius <= radius < .25:
                raise ValueError("Unexpected idler/tread contact dimensions")
            wheels[key].update(position=center, radiusM=radius, widthM=y_max-y_min)
            audit.append(dict(wheel=key, rotor=rotor["id"], center=center,
                              idlerRadiusM=core_radius, contactRadiusM=radius,
                              treadAllowanceM=radius-core_radius, treadBottomM=bottom))
    geometry["wheelbaseM"] = sum(wheels["front_"+s]["position"][0] - wheels["rear_"+s]["position"][0]
                                for s in ("left", "right")) / 2
    for axle in ("front", "rear"):
        geometry[axle + "TrackM"] = abs(wheels[axle+"_left"]["position"][1] - wheels[axle+"_right"]["position"][1])
    geometry["wheelRadiusM"] = sum(item["radiusM"] for item in wheels.values()) / 4
    geometry["wheelWidthM"] = sum(item["widthM"] for item in wheels.values()) / 4
    for rotor in metadata["simulationVisualRotors"]:
        previous_source = rotor["sourceWheel"]
        visual_radius = old_radii[previous_source] / rotor["ratio"]
        side = "left" if rotor["id"].startswith("left_") else "right"
        rotor["sourceWheel"] = ("rear_" if "idler_back" in rotor["id"] else "front_") + side
        rotor["ratio"] = wheels[rotor["sourceWheel"]]["radiusM"] / visual_radius
    for belt in metadata["simulationVisualBelts"]:
        belt["sourceRadius"] = wheels[belt["sourceWheel"]]["radiusM"]
    metadata["simulationTrackContactGeometry"] = {
        "revision": "car3_visible_idler_centers_v1", "baseFrameUnchanged": True,
        "model": "four_contact_cylinders_at_visual_idlers_with_tread_allowance",
        "physicalMeasurement": False, "contacts": audit,
    }
    return result
