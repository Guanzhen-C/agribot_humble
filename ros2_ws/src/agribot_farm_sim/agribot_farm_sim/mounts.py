"""Prepare opt-in, fixed transport mounts without altering source CAD exports."""

import json
from pathlib import Path
import math
import shutil
import xml.etree.ElementTree as ET

from agribot_vehicle_config_tools.model import ConfigError, load_and_validate, validate
from .materials import normalize_ambient
from . import hitches


PRESETS = {
    "compact_chisel_plow": {"massKg": 60.0, "workingWidthM": 2.2},
    "mounted_disc_harrow": {"massKg": 100.0, "workingWidthM": 2.6},
    "groove_maker": {"massKg": 220.0, "workingWidthM": 5.4},
    "mounted_cultivator": {"massKg": 180.0, "workingWidthM": 2.6},
    "forest_chain_crusher": {"massKg": 350.0, "workingWidthM": 2.4},
    "inter_row_milling_cutter": {"massKg": 450.0, "workingWidthM": 3.0},
    "front_sickle_bar": {"massKg": 30.0, "workingWidthM": 1.45},
    "garlic_planter": {"massKg": 60.0, "workingWidthM": 1.0},
    "mulch_planter": {"massKg": 80.0, "workingWidthM": 0.9},
    "farm_trailer": {"massKg": 450.0, "workingWidthM": 2.94},
    "combine_header": {"massKg": 2500.0, "workingWidthM": 12.7},
}
COMPATIBLE_VEHICLES = {
    # Candidate hosts share inspected rear anchors. Layout, scale and axle
    # loading remain mandatory; this is not a hardware lifting-capacity rating.
    **{kind: tuple(hitches.HOSTS) for kind in hitches.TOOLS},
    "front_sickle_bar": "ride_on_mower",
    "farm_trailer": "cab_tractor",
    "combine_header": "tracked_harvester",
}


def transport_balance(config):
    """Check nominal level-ground axle loading from the actual generated inertials."""
    from agribot_vehicle_config_tools.generator import generate_sdf

    model = ET.fromstring(generate_sdf(config)).find("model")
    mass, moment = 0.0, 0.0
    for link in model.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        value = float(inertial.findtext("mass"))
        origin_x = float((link.findtext("pose") or "0").split()[0])
        local_x = float((inertial.findtext("pose") or "0").split()[0])
        mass += value
        moment += value * (origin_x + local_x)
    wheels = config["geometry"]["wheelCenters"]
    rear = sum(w["position"][0] for w in wheels if w["id"].startswith("rear_")) / 2
    front = sum(w["position"][0] for w in wheels if w["id"].startswith("front_")) / 2
    center = moment / mass
    fraction = (center - rear) / (front - rear)
    if not 0 < fraction < 1:
        raise ConfigError("nominal transport centre of mass is outside the axle support span; "
                          "choose a compatible tractor or explicitly model ballast")
    return {"totalMassKg": mass, "centerOfMassXM": center, "frontAxleLoadFraction": fraction,
            "scope": "Nominal level-ground static check only; not a dynamic stability or real lifting-capacity rating."}


def transport_wheel_clearance(config):
    """Conservative box/cylinder separation, including the added hitch links."""
    from agribot_vehicle_config_tools.implements import collision_boxes

    closest = float("inf")
    for box in collision_boxes(config, padding=0.):
        for wheel in config["geometry"]["wheelCenters"]:
            x, y, z = wheel["position"]
            radius = wheel.get("radiusM") or config["geometry"]["wheelRadiusM"]
            width = wheel.get("widthM") or config["geometry"]["wheelWidthM"]
            dy = max(box[1] - y - width / 2, y - width / 2 - box[4])
            dx = max(box[0] - x, x - box[3], 0.)
            dz = max(box[2] - z, z - box[5], 0.)
            separation = max(dy, math.hypot(dx, dz) - radius)
            if wheel["steering"]:
                # A sphere contains the full steering sweep of the cylinder.
                dy = max(box[1] - y, y - box[4], 0.)
                separation = math.sqrt(dx*dx + dy*dy + dz*dz) - math.hypot(radius, width / 2)
            closest = min(closest, separation)
            if separation <= 0:
                raise ConfigError("implement or hitch overlaps a conservative wheel envelope; inspect mounting clearance")
    return {"minimumSeparationLowerBoundM": closest,
            "scope": "Conservative boxes versus wheel cylinders/steering spheres; not articulation or deformation simulation."}


def mounted_configuration(vehicle, asset, vehicle_id):
    kind = asset["id"]
    allowed = COMPATIBLE_VEHICLES.get(kind)
    allowed = (allowed,) if isinstance(allowed, str) else allowed or ()
    base_id = vehicle["vehicleId"]
    if kind not in PRESETS or base_id not in allowed:
        raise ConfigError("this nominal mounting preset is only available for " + str(allowed))
    if base_id.startswith("unity_car") and not vehicle.get("metadata", {}).get("nominalDimensions"):
        raise ConfigError("original tractor must first be normalized to its nominal metric dimensions")
    if vehicle.get("implements") or vehicle.get("manipulators"):
        raise ConfigError("remove the existing implement/manipulator before mounting")
    front = kind == "front_sickle_bar"
    trailer = kind == "farm_trailer"
    header = kind == "combine_header"
    origin = "inspected_host_mount" if header else "articulated_hitch" if trailer else "mount_ground_center" if front else "front_ground_center"
    if (asset["coordinateConvention"] != "ROS_FLU" or asset["units"] != "metres"
            or asset["originDefinition"] != origin):
        raise ConfigError("implement requires a normalized ROS_FLU " + origin + " asset")
    config = json.loads(json.dumps(vehicle))
    config["vehicleId"] = vehicle_id
    config["displayName"] += " + " + asset["displayName"]
    for visual in config.get("visuals", []):
        visual["meshUri"] = visual["meshUri"].replace("/generated/" + base_id + "/", "/generated/" + vehicle_id + "/")
    for wheel in config["geometry"]["wheelCenters"]:
        for key in ("caster", "steeringAssembly"):
            if key in wheel:
                wheel[key]["meshUri"] = wheel[key]["meshUri"].replace(
                    "/generated/" + base_id + "/", "/generated/" + vehicle_id + "/")
    low, high = asset["minimumM"], asset["maximumM"]
    geometry = vehicle["geometry"]
    ground = min(wheel["position"][2] - (wheel.get("radiusM") or geometry["wheelRadiusM"])
                 for wheel in geometry["wheelCenters"])
    gap, clearance = 0.30, 0.15
    pose_x = -geometry["bodyExtents"]["rearM"] - gap
    boxes = [
        {"name": "implement_envelope", "center": [(a + b) / 2 for a, b in zip(low, high)],
         "size": [b - a + .002 for a, b in zip(low, high)]},
    ]
    if front:
        pose_x, clearance = geometry["bodyExtents"]["frontM"] + .18, .12
        boxes = json.loads(json.dumps(asset["collisionBoxes"]))
    if trailer:
        clearance = asset["groundBelowHitchM"]
        boxes = json.loads(json.dumps(asset["collisionBoxes"]))
    if header:
        mount = asset.get("mountPoseXyz")
        if (not isinstance(mount, list) or len(mount) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in mount)):
            raise ConfigError("combine header requires an inspected finite host mount pose")
        boxes = json.loads(json.dumps(asset["collisionBoxes"]))
        clearance = asset["mountPoseXyz"][2] + low[2] - ground
    config["implements"] = [{
        "id": kind, "implementType": kind, "parentFrameId": "base_link",
        "frameId": kind + "_link", "workFrameId": kind + "_work_frame",
        "pose": {"xyz": [pose_x, 0.0, ground + clearance],
                 "rpy": [0.0, 0.0, 0.0], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        "meshFile": "implement_" + kind + "_visual.obj", "mountMode": "fixed_transport",
        "simulationOnly": True, **PRESETS[kind],
        "workOffset": [.25 if front else low[0] * 0.65, 0.0, 0.0],
        "collisionBoxes": boxes,
    }]
    config.setdefault("metadata", {}).update({
        "calibrationState": "unity_nominal_simulation_only",
        "implementSourceSha256": asset["sourceSha256"],
        "implementRedistributionLicense": asset["redistributionLicense"],
        "implementDynamics": "Nominal mass and box inertia; raised rigid transport only, no soil or cutting forces.",
        "implementGroundClearanceM": clearance,
    })
    if not (front or trailer or header):
        hitch = hitches.layout(vehicle, kind, config["implements"][0]["pose"]["xyz"])
        if hitch is None:
            raise ConfigError("rear transport mount requires inspected host and implement anchors")
        item = config["implements"][0]
        item["pose"]["xyz"] = hitch["pose"]
        item["collisionBoxes"] = boxes + hitches.collision_boxes(hitch)
        config["metadata"].update(implementHitch=hitch,
            implementGroundClearanceM=hitch["pose"][2] + low[2] - ground)
    if header:
        config["implements"][0]["pose"]["xyz"] = list(asset["mountPoseXyz"])
        config["implements"][0]["workOffset"] = [high[0], 0., low[2]]
        config["metadata"]["implementDynamics"] = "Detachable inspected header, rigid raised transport only; no reel, crop cutting or harvesting simulation."
    if trailer:
        config["implements"][0].update(mountMode="articulated_transport", yawLimitRad=.25, pitchLimitRad=.15,
            wheels=json.loads(json.dumps(asset["wheels"])))
        # Tandem-axle transport needs gentler curvature than the bare tractor.
        radius = 16.
        config["geometry"].update(minTurningRadiusM=radius,
            maxSteeringAngleRad=math.atan(config["geometry"]["wheelbaseM"]/radius))
        angular = config["navigation"]["maxLinearSpeedMps"]/radius
        config["navigation"]["maxAngularSpeedRadps"] = angular
        config["chassis"]["commandMaxAngularSpeedRadps"] = angular
        config["metadata"]["implementDynamics"] = "Empty passive tandem-axle trailer with yaw/pitch hitch articulation; conservative swept footprint, forward low-speed transport only. No tipping or trailer-aware reverse planning."
        config["metadata"]["scenarioSpawnOverrides"] = {"arable_field": [13., 38., 0.]}
    validate(config)
    if not trailer:
        config["metadata"]["implementTransportBalance"] = transport_balance(config)
        if not (front or header):
            config["metadata"]["implementWheelClearance"] = transport_wheel_clearance(config)
        contact = config["metadata"].get("simulationSkidContact")
        if contact:
            # Equal lateral resistance at the two virtual roller stations keeps
            # this nominal skid proxy centred after adding a heavy front tool.
            fraction = config["metadata"]["implementTransportBalance"]["frontAxleLoadFraction"]
            contact["frontLateralFriction"] = contact["lateralFriction"] * (1.0 - fraction) / fraction
            contact["rearLateralFriction"] = contact["lateralFriction"]
    return config


def prepare_mount(vehicle_path, asset_path, output, vehicle_id):
    vehicle_path, asset_path, output = map(Path, (vehicle_path, asset_path, output))
    if output.exists():
        raise FileExistsError("refusing to overwrite a prior vehicle export: " + str(output))
    vehicle, _ = load_and_validate(vehicle_path)
    asset = json.loads((asset_path / "implement_asset.json").read_text(encoding="utf-8"))
    config = mounted_configuration(vehicle, asset, vehicle_id)
    mesh = asset_path / "implement_visual.obj"
    material = mesh.with_suffix(".mtl")
    if not mesh.is_file() or not material.is_file():
        raise FileNotFoundError("normalized implement OBJ/MTL not found")
    output.mkdir(parents=True)
    for path in vehicle_path.parent.iterdir():
        if path.is_file() and path.suffix.lower() in (".obj", ".mtl"):
            shutil.copy2(path, output / path.name)
            if path.suffix.lower() == ".mtl":
                normalize_ambient(output / path.name)
    target_name = config["implements"][0]["meshFile"]
    target_material = Path(target_name).with_suffix(".mtl").name
    with mesh.open(encoding="utf-8") as source, (output / target_name).open("w", encoding="utf-8") as target:
        for line in source:
            target.write("mtllib " + target_material + "\n" if line.startswith("mtllib ") else line)
    shutil.copy2(material, output / target_material)
    normalize_ambient(output / target_material)
    if config.get("metadata", {}).get("implementHitch"):
        hitches.append_mesh(output / target_name, output / target_material,
                           config["metadata"]["implementHitch"])
    for wheel in asset.get("wheels", []):
        for suffix in (".obj", ".mtl"):
            path = (asset_path / wheel["meshFile"]).with_suffix(suffix)
            shutil.copy2(path, output / path.name)
    target = output / "vehicle_config.json"
    target.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return target
