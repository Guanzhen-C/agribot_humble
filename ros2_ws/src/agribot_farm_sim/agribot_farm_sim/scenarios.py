"""Prepare portable agricultural worlds, matching occupancy maps and routes."""

import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


SCENARIO_IDS = ("orchard", "arable_field", "greenhouse", "farmyard", "tomato_greenhouse", "strawberry_field", "vineyard", "vegetable_field", "wide_headland")
PREPARED_SCENES = {
    "farmyard": "clearpath_farm", "greenhouse": "harbot_greenhouse", "arable_field": "maize_field",
    "tomato_greenhouse": "aoc_tomato",
    "strawberry_field": "harvester_strawberry",
    "vineyard": "cropcraft_vineyard",
    "vegetable_field": "pic4ser_vegetables",
    "wide_headland": "wide_headland",
}


def vehicle_spawn(config, scenario, default):
    """Use an explicit vehicle-specific spawn when its transport length requires it."""
    selected = config.get("metadata", {}).get("scenarioSpawnOverrides", {}).get(scenario, default)
    if not isinstance(selected, (list, tuple)) or len(selected) != 3 or any(
        isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
        for value in selected
    ):
        raise ValueError("Expected finite vehicle spawn x/y/yaw")
    return [float(value) for value in selected]


def write_georeference(template, destination, spawn, antenna, scenario):
    """Match SimNavSatBridge's ENU origin at the initial master antenna."""
    values = [*spawn, *antenna]
    if len(spawn) != 4 or len(antenna) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected finite spawn x/y/z/yaw and antenna x/y/z")
    x, y, z, yaw = spawn
    ax, ay, az = antenna
    config = yaml.safe_load(Path(template).read_text())
    config["map"]["id"] = scenario + "_configured_sim"
    config["map_from_enu"] = {
        "xyz": [x + math.cos(yaw) * ax - math.sin(yaw) * ay,
                y + math.sin(yaw) * ax + math.cos(yaw) * ay, z + az],
        "rpy": [0.0, 0.0, 0.0],
    }
    config["calibration"]["hash"] = "deterministic-" + scenario + "-enu"
    Path(destination).write_text(yaml.safe_dump(config, sort_keys=False))
    return str(destination)


def route(scenario, vehicle=None):
    minimum_radius = 0.0
    if vehicle is not None and vehicle["vehicleType"] == "ackermann":
        minimum_radius = float(vehicle["geometry"]["minTurningRadiusM"])
        if not math.isfinite(minimum_radius) or minimum_radius <= 0:
            raise ValueError("Expected a positive finite vehicle turning radius")
    if scenario == "wide_headland":
        diagonal = 12 / math.sqrt(2)
        return [(-15, 0, 0), (0, 0, 0), (10, 0, 0),
                (10+diagonal, 12-diagonal, math.pi/4), (22, 12, math.pi/2),
                (22, 25, math.pi/2)]
    if scenario == "vegetable_field":
        # Transit around the crops once; completion is progress-gated, not
        # forced onto a second small loop to disambiguate the shared finish.
        radius = max(4.5, minimum_radius)
        diagonal = radius / math.sqrt(2)
        return [(-3, 0, 0), (12, 0, 0),
                (12 + diagonal, radius - diagonal, math.pi / 4), (12 + radius, radius, math.pi / 2),
                (12 + diagonal, radius + diagonal, 3 * math.pi / 4), (12, 2 * radius, math.pi),
                (-12, 2 * radius, math.pi), (-12 - diagonal, radius + diagonal, -3 * math.pi / 4),
                (-12 - radius, radius, -math.pi / 2), (-12 - diagonal, radius - diagonal, -math.pi / 4),
                (-12, 0, 0), (-8, 0, 0)]
    if scenario == "vineyard":
        # A large tractor skips the adjacent aisle, so the headland is one
        # broad semicircle rather than a tight loop beyond the finite floor.
        radius = max(3.0, 3.0 * math.ceil(minimum_radius / 3.0))
        if radius > 3.0:
            # Short heading-constrained sub-arcs can provoke a full Dubins
            # loop on the discretized grid. Keep the apex, not the diagonals.
            return [(-4, -3, 0), (12, -3, 0), (12 + radius, -3 + radius, math.pi / 2),
                    (12, -3 + 2 * radius, math.pi), (-12, -3 + 2 * radius, math.pi)]
        diagonal = radius / math.sqrt(2)
        return [(-4, -3, 0), (12, -3, 0),
                (12 + diagonal, -3 + radius - diagonal, math.pi / 4),
                (12 + radius, -3 + radius, math.pi / 2),
                (12 + diagonal, -3 + radius + diagonal, 3 * math.pi / 4),
                (12, -3 + 2 * radius, math.pi), (-12, -3 + 2 * radius, math.pi)]
    if scenario == "strawberry_field":
        if minimum_radius > 3.0:
            return [(-3, -3, 0), (6, -3, 0), (6, 3, math.pi), (-6, 3, math.pi)]
        # Tangent headings on a 3 m radius headland semicircle.
        return [(-3, -3, 0), (6, -3, 0), (9, 0, math.pi / 2),
                (6, 3, math.pi), (-6, 3, math.pi)]
    if scenario == "tomato_greenhouse":
        return [(-3, 0, 0), (1, 0, 0), (5, 0, 0)]
    if scenario == "farmyard":
        return [(18, 0, 0), (22, 6, math.pi / 2), (18, 10, math.pi),
                (-48, 14, math.pi), (-55, 6, -math.pi / 2), (-48, 0, 0), (0, 0, 0)]
    if scenario == "greenhouse":
        return [(-4, 10, math.pi / 2), (-2, 14, 0), (0, 10, -math.pi / 2),
                (0, -10, -math.pi / 2), (2, -14, 0), (4, -10, math.pi / 2),
                (4, 10, math.pi / 2)]
    if scenario != "arable_field":
        raise ValueError(f"No new-scene route: {scenario}")
    levels = (34, 28, 22, 16, 10)
    result = []
    for i, y in enumerate(levels):
        result.append((46 if i % 2 == 0 else 4, y, 0 if i % 2 == 0 else math.pi))
        if i + 1 < len(levels):
            result.append((46 if i % 2 == 0 else 4, levels[i + 1], -math.pi / 2))
    return result


def write_vehicle_route(scenario, vehicle, directory):
    """Share the same vehicle-aware targets between launch and acceptance."""
    vehicle_id = vehicle["vehicleId"]
    if not re.fullmatch(r"[a-z][a-z0-9_]*", vehicle_id):
        raise ValueError("Unsafe vehicle route filename")
    destination = Path(directory) / (vehicle_id + "_waypoints.yaml")
    points = route(scenario, vehicle)
    document = {
        "waypoints": [dict(x=x, y=y, yaw=yaw) for x, y, yaw in points],
        "routePolicy": {
            "vehicleId": vehicle_id,
            "minimumTurningRadiusM": vehicle["geometry"]["minTurningRadiusM"],
            "claim": "Vehicle-aware guidance only; full footprint planning and runtime checks remain required",
        },
    }
    if scenario == "vineyard":
        from ament_index_python.packages import get_package_share_directory
        from agribot_vehicle_config_tools.model import footprint
        asset = Path(get_package_share_directory("agribot_farm_sim")) / "assets/cropcraft_vineyard/source.json"
        ground = next(item for item in json.loads(asset.read_text())["objects"]
                      if item["name"] == "Ground" and item["navigationSurface"])
        document["navigationSurface"] = {
            "bounds": [*ground["minimum"][:2], *ground["maximum"][:2]],
            "footprint": footprint(vehicle), "marginM": .25,
        }
    destination.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return destination


def build(scenario, directory):
    from ament_index_python.packages import get_package_share_directory

    if scenario not in PREPARED_SCENES:
        raise ValueError(f"Unknown prepared agricultural scene: {scenario}")
    asset = Path(get_package_share_directory("agribot_farm_sim")) / "assets" / PREPARED_SCENES[scenario]
    if not (asset / "source.json").is_file():
        raise FileNotFoundError(f"Prepared agricultural scene assets are missing: {asset}")
    root = ET.parse(asset / "world.sdf")
    for uri in root.findall(".//mesh/uri"):
        relative = Path(uri.text)
        path = asset / relative
        if relative.is_absolute() or ".." in relative.parts or not path.is_file():
            raise ValueError(f"Invalid scene mesh reference: {uri.text}")
        uri.text = path.as_uri()
    metadata = yaml.safe_load((asset / "map.yaml").read_text())
    image = Path(metadata["image"])
    if image.is_absolute() or ".." in image.parts or not (asset / image).is_file():
        raise ValueError("Invalid scene occupancy image")
    metadata["image"] = str(asset / image)
    directory = Path(directory) / scenario
    directory.mkdir(parents=True, exist_ok=True)
    source = json.loads((asset / "source.json").read_text())
    from agribot_farm_sim.ground import apply_prepared_ground_policy
    source["groundPolicy"] = apply_prepared_ground_policy(root.getroot(), source, directory, scenario)
    root.write(directory / "world.sdf", encoding="utf-8", xml_declaration=True)
    (directory / "map.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    (directory / "geometry.json").write_text(json.dumps(source, indent=2) + "\n")
    (directory / "waypoints.yaml").write_text(yaml.safe_dump({"waypoints": [dict(x=x, y=y, yaw=yaw)
        for x, y, yaw in route(scenario)]}), encoding="utf-8")
    return directory
