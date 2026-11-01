#!/usr/bin/env python3
"""Place the real saved path and an imported LCC2 scene in the same ENU frame.

Only a new validation bundle is written. Original splats, collision meshes, real
map, frozen path and hardware parameters are referenced or hashed, never edited.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nav2_free_pixels(image, geometry, include_unknown=False):
    """Use map_server's trinary grayscale semantics, including edited whites."""
    if geometry.get("mode", "trinary") != "trinary":
        raise ValueError("Scanned terrain preparation requires a trinary Nav2 map")
    free = float(geometry.get("free_thresh", .196))
    occupied = float(geometry.get("occupied_thresh", .65))
    if not 0 < free < occupied < 1:
        raise ValueError("Invalid Nav2 map occupancy thresholds")
    probability = np.asarray(image, dtype=float)/255.
    if not geometry.get("negate", 0):
        probability = 1.-probability
    return probability <= occupied if include_unknown else probability < free


def stl_triangles(path):
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    with Path(path).open("rb") as stream:
        stream.seek(80)
        count = np.fromfile(stream, dtype="<u4", count=1)
        if len(count) != 1 or Path(path).stat().st_size != 84 + int(count[0])*50:
            raise ValueError(f"Invalid binary STL: {path}")
        records = np.fromfile(stream, dtype=dtype, count=int(count[0]))
    vertices = records["vertices"].astype(float)
    if not np.isfinite(vertices).all():
        raise ValueError("Non-finite collision vertices")
    return vertices


def surface_heights(triangles, xy):
    low, high = triangles[:, :, :2].min(1), triangles[:, :, :2].max(1)
    a, b, c = triangles[((low <= xy) & (high >= xy)).all(1)].transpose(1, 0, 2)
    ab, ac = b-a, c-a
    determinant = ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0]
    valid = abs(determinant) > 1e-12
    a, ab, ac, determinant = a[valid], ab[valid], ac[valid], determinant[valid]
    dx, dy = (np.asarray(xy)-a[:, :2]).T
    u = (dx*ac[:, 1]-dy*ac[:, 0])/determinant
    v = (ab[:, 0]*dy-ab[:, 1]*dx)/determinant
    return np.sort((a[:, 2]+u*ab[:, 2]+v*ac[:, 2])[(u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8)])


def projected_to_enu(metadata, georeference):
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, str(Path(get_package_prefix("agribot_offline_mapping"))/"lib/agribot_offline_mapping"))
    from prepare_lixel_navigation_map import EnuFrame
    reference = georeference["reference"]
    frame = EnuFrame(metadata["epsg"], [reference[k] for k in ("longitude_deg", "latitude_deg", "altitude_m")])
    offset = np.array(metadata["offset"], dtype=float)
    basis = frame.projected(offset+np.array([[0, 0, 0], [100, 0, 0], [0, 100, 0]]))
    rotation_yaw = float(math.atan2(*(basis[1, :2]-basis[0, :2])[::-1]))
    if not np.allclose(georeference["map_from_enu"]["rpy"][:2], 0):
        raise ValueError("This planar validation requires a level ENU map")
    map_yaw = float(georeference["map_from_enu"]["rpy"][2])
    r = np.array([[math.cos(map_yaw), -math.sin(map_yaw)], [math.sin(map_yaw), math.cos(map_yaw)]])
    translation = basis[0, :2] @ r.T + np.array(georeference["map_from_enu"]["xyz"][:2])
    return rotation_yaw+map_yaw, translation, frame


def prepare(args):
    source = args.scene_manifest.resolve().parent
    base = args.map_base.resolve()
    out = args.output.resolve()
    if out.exists():
        raise ValueError("Use a new output directory; existing validation bundles are immutable")
    world_source = source/json.loads(args.scene_manifest.read_text())["world"]
    lcc_files = list((source/"source").glob("*.lcc2"))
    if len(lcc_files) != 1:
        raise ValueError("Expected one original LCC2 metadata file")
    metadata = json.loads(lcc_files[0].read_text())
    if metadata.get("shift") != [0, 0, 0] or metadata.get("scale") != [1, 1, 1]:
        raise ValueError("Unsupported source shift/scale")
    geofile = Path(str(base)+"_georeference.yaml")
    georef = yaml.safe_load(geofile.read_text())
    route_file = Path(str(base)+"_saved_paths.json")
    routes = json.loads(route_file.read_text())
    route = next(r for r in routes["routes"] if r["id"] == args.route_id)
    if len(route["anchor_path_indices"]) != 104 or len(route["path"]) < 104:
        raise ValueError("Validation must use the complete real 104-anchor route")
    for key, file in (("pgm", base.with_suffix(".pgm")), ("yaml", base.with_suffix(".yaml")), ("georeference", geofile)):
        if routes["fingerprints"][key] != sha256(file):
            raise ValueError(f"Saved path {key} fingerprint does not match the real map")
    protected = [world_source, lcc_files[0], base.with_suffix(".pgm"), base.with_suffix(".yaml"), geofile, route_file, args.nav2_params]
    if base.with_suffix(".pcd").is_file():
        protected.append(base.with_suffix(".pcd"))
    overlay_file = getattr(args, "nav2_overlay", None)
    vehicle_file = getattr(args, "vehicle_config", None)
    overlay_file = overlay_file.resolve() if overlay_file else None
    vehicle_file = vehicle_file.resolve() if vehicle_file else None
    protected.extend(p for p in (overlay_file, vehicle_file) if p is not None)
    hashes = {str(p): sha256(p) for p in protected}
    yaw, translation, frame = projected_to_enu(metadata, georef)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    world = ET.parse(world_source)
    model = world.find("world/model[@name='k2_automation_sensor_scene']")
    old_pose = list(map(float, model.findtext("pose").split()))
    if any(abs(old_pose[i]) > 1e-8 for i in (0, 1, 3, 4, 5)):
        raise ValueError("Unexpected pretransformed imported scene")
    model.find("pose").text = " ".join(map(str, [*translation, old_pose[2], 0, 0, yaw]))
    first = route["path"][0]
    source_xy = (np.array([first["x"], first["y"]])-translation) @ rotation
    triangles = np.concatenate([stl_triangles(p) for p in sorted((source/"meshes").glob("collision_*.stl"))])
    heights = surface_heights(triangles, source_xy)
    if not len(heights):
        raise ValueError("No scanned ground below the real route start")
    ground_z = float(heights[-1]+old_pose[2])
    if ground_z > 2 or ground_z < -2:
        raise ValueError("Route start is not on the expected ground surface")
    gui = world.find("world/gui/camera/pose")
    if gui is not None:
        gui.text = f"{first['x']-10} {first['y']-12} {ground_z+12} 0 .6 .85"
    # Preserve the production controller/planner and alter only transport/clock
    # bindings required by the simulated sensors and ROS environment.
    nav2 = yaml.safe_load(args.nav2_params.read_text())
    def clock_bindings(value):
        if not isinstance(value, dict):
            return
        for key, child in value.items():
            if key == "use_sim_time": value[key] = True
            elif key == "odom_topic": value[key] = "/odometry/filtered_navsat"
            elif key == "topic" and child == "/navigation/obstacle_points": value[key] = "/configured/obstacle_points"
            else: clock_bindings(child)
    clock_bindings(nav2)
    if overlay_file:
        selected = nav2["controller_server"]["ros__parameters"]["FollowPath"]
        for key, value in yaml.safe_load(overlay_file.read_text())["FollowPath"].items():
            if isinstance(value, dict):
                selected.setdefault(key, {}).update(value)
            else:
                selected[key] = value
    tolerance = .25
    navigation_footprint = None
    if vehicle_file:
        vehicle = json.loads(vehicle_file.read_text())
        tolerance = float(vehicle["geometry"]["wheelbaseM"])/2
        nav2["controller_server"]["ros__parameters"]["goal_checker"]["xy_goal_tolerance"] = tolerance
        extents = vehicle["geometry"].get("bodyExtents")
        if extents:
            clearance = float(getattr(args, "simulation_clearance", .05))
            if not math.isfinite(clearance) or not 0 <= clearance <= .3:
                raise ValueError("Simulation body clearance must be between 0 and 0.3 m")
            front, rear = float(extents["frontM"])+clearance, float(extents["rearM"])+clearance
            left, right = float(extents["leftM"])+clearance, float(extents["rightM"])+clearance
            navigation_footprint = [[front, left], [front, -right], [-rear, -right], [-rear, left]]
            for name in ("global_costmap", "local_costmap"):
                settings = nav2[name][name]["ros__parameters"]
                settings["footprint"] = json.dumps(navigation_footprint)
                settings["footprint_padding"] = 0.
    nav2["planner_server"]["ros__parameters"]["GridBased"]["lattice_filepath"] = str(args.nav2_params.parent/"motion_primitives/diff_5cm.json")
    out.mkdir(parents=True)
    ET.indent(world, space="  ")
    world.write(out/"world.sdf", encoding="utf-8", xml_declaration=True)
    geometry = yaml.safe_load(base.with_suffix(".yaml").read_text())
    geometry["image"] = str(base.with_suffix(".pgm"))
    (out/"map.yaml").write_text(yaml.safe_dump(geometry))
    (out/"nav2_params.yaml").write_text(yaml.safe_dump(nav2, sort_keys=False))
    (out/"saved_route.json").write_text(json.dumps(route, ensure_ascii=False))
    (out/"waypoints.yaml").write_text(yaml.safe_dump({"frame_id": "map", "waypoints": [route["path"][i] for i in route["anchor_path_indices"]]}))
    manifest = {"schema_version": 1, "id": "farmland_real_104", "world": "world.sdf", "map": "map.yaml", "waypoints": "waypoints.yaml", "spawn": [first["x"], first["y"], first["yaw"]], "spawn_ground_z": ground_z, "manual_goals_only": True}
    (out/"scene.json").write_text(json.dumps(manifest, indent=2))
    corners = np.array([[x, y, 0] for x in (-100, 100) for y in (-150, 150)])
    exact = frame.projected(corners+metadata["offset"])[:, :2]
    approximation = corners[:, :2] @ rotation.T+translation
    report = {"source_scene": str(source), "map_base": str(base), "vehicle_id": "unity_car3", "route_id": route["id"], "path_count": len(route["path"]), "anchor_count": 104,
              "goal_tolerance_m": tolerance, "nav2_simulation_overlay": str(overlay_file) if overlay_file else None,
              "controller_plugin": nav2["controller_server"]["ros__parameters"]["FollowPath"]["plugin"],
              "navigation_footprint": navigation_footprint,
              "simulation_clearance_m": float(getattr(args, "simulation_clearance", .05)) if navigation_footprint else None,
              "allow_unknown": bool(nav2["planner_server"]["ros__parameters"]["GridBased"].get("allow_unknown", False)),
              "scene_yaw_rad": yaw, "scene_translation_xy": translation.tolist(), "rigid_projection_error_max_m": float(np.linalg.norm(exact-approximation, axis=1).max()),
              "spawn_ground_z": ground_z, "original_source_hashes": hashes,
              "scope": "Actual Car3 wheel/contact physics, full original Gaussian scene and real frozen route. Simulated GNSS/IMU into KF-GINS; no ground-truth navigation substitute. Hardware files unchanged."}
    if hashes != {str(p): sha256(Path(p)) for p in hashes}:
        raise ValueError("Source files changed during preparation")
    (out/"preparation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene_manifest", type=Path)
    parser.add_argument("map_base", type=Path)
    parser.add_argument("nav2_params", type=lambda p: Path(p).resolve())
    parser.add_argument("output", type=Path)
    parser.add_argument("--route-id", default="manual104_saved")
    parser.add_argument("--nav2-overlay", type=Path, default=Path(get_package_share_directory("agribot_differential_fastlivo_mppi"))/"config/scanned_saved_route_nav2_overlay.yaml")
    parser.add_argument("--vehicle-config", type=Path, default=Path(get_package_share_directory("agribot_vehicle_description"))/"generated/unity_car3/vehicle_config.json")
    parser.add_argument("--simulation-clearance", type=float, default=.05,
                        help="Extra margin around the actual Unity body, not a change to hardware parameters")
    prepare(parser.parse_args())


if __name__ == "__main__":
    main()
