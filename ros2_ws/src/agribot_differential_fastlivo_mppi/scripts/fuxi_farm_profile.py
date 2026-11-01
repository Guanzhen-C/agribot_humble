#!/usr/bin/env python3
"""Install and run the accepted Car3/104-point profile without changing its inputs."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml


PROFILE_ID = "fuxi_farm_104"
ALGORITHM = "stvl_off_navsat_smac_hybrid_rpp"
VEHICLE_ID = "unity_car3"
CONTROLLER = "agribot_vehicle_description::PositionOnlyRegulatedPurePursuitController"


def digest(path):
    with Path(path).open("rb") as stream:
        result = hashlib.sha256()
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            result.update(block)
        return result.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def remap_text(value, replacements):
    for old, new in sorted(replacements.items(), key=lambda item: -len(item[0])):
        if value == old or value.startswith(old + "/"):
            return new + value[len(old):]
    return value


def remap_tree(value, replacements):
    if isinstance(value, dict):
        return {key: remap_tree(item, replacements) for key, item in value.items()}
    if isinstance(value, list):
        return [remap_tree(item, replacements) for item in value]
    return remap_text(value, replacements) if isinstance(value, str) else value


def check_route(route):
    path, anchors = route["path"], route["anchor_path_indices"]
    if len(anchors) != 104 or len(path) != 11285:
        raise ValueError("The accepted route requires all 104 anchors and 11285 original poses")
    if not all(isinstance(i, int) and 0 <= i < len(path) for i in anchors):
        raise ValueError("Invalid original anchor indices")
    if anchors != sorted(set(anchors)):
        raise ValueError("Original anchors must remain in order")
    if not all(math.isfinite(p[k]) for p in path for k in ("x", "y", "yaw")):
        raise ValueError("Non-finite route pose")


def same_configuration(actual, expected, where="vehicle"):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys() != expected.keys():
            raise ValueError("Validated configuration differs at " + where)
        for key, value in expected.items():
            same_configuration(actual[key], value, where + "." + key)
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            raise ValueError("Validated configuration differs at " + where)
        for index, (value, reference) in enumerate(zip(actual, expected)):
            same_configuration(value, reference, f"{where}[{index}]")
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if (isinstance(actual, bool) or not isinstance(actual, (int, float))
                or not math.isclose(actual, expected, rel_tol=0., abs_tol=1e-5)):
            raise ValueError("Validated configuration differs at " + where)
    elif actual != expected:
        raise ValueError("Validated configuration differs at " + where)


def compare_route_poses(reference, planned):
    result = {"reference_pose_count": len(reference), "planned_pose_count": len(planned),
              "exactly_equal": planned == reference, "max_absolute_difference": None}
    if len(reference) == len(planned) and reference:
        result["max_absolute_difference"] = {
            key: max(abs(a[key] - b[key]) for a, b in zip(reference, planned))
            for key in ("x", "y", "yaw")}
    return result


def validate_vehicle(actual, expected):
    if actual.get("vehicleId") != VEHICLE_ID or actual.get("vehicleType") != "tracked":
        raise ValueError("伏羲农场固定路线仅使用已验证的Car3差速履带车")
    for key in ("coordinateConvention", "baseFrame", "geometry", "navigation", "chassis",
                "dynamics", "collision", "sensors"):
        same_configuration(actual.get(key), expected.get(key), key)
    if actual.get("implements"):
        raise ValueError("伏羲农场固定验证配置不挂载农机具")
    if actual.get("manipulators"):
        # The accepted export contains a dormant arm preset; launch disables it.
        same_configuration(actual["manipulators"], expected.get("manipulators", []), "manipulators")


def install_profile(bundle, destination, workspace, planning_parameters=None):
    bundle, destination, workspace = (Path(p).resolve() for p in (bundle, destination, workspace))
    report = read_json(bundle / "result.json")
    if not report.get("passed") or report.get("diagnostic_only") or report.get("controller_plugin") != CONTROLLER:
        raise ValueError("A complete accepted Car3/RPP result is required")
    check_route(read_json(bundle / "saved_route.json"))
    metadata = read_json(bundle / "preparation.json")
    for name, expected in metadata["original_source_hashes"].items():
        if digest(name) != expected:
            raise ValueError("Accepted source changed: " + name)
    if destination.exists():
        raise ValueError("Use a new profile directory; existing accepted assets are never overwritten")
    template = destination / "template"
    template.mkdir(parents=True)
    files = ["world.sdf", "scene.json", "map.yaml", "georeference.yaml", "waypoints.yaml",
             "saved_route.json", "nav2_params.yaml", "ground_support.stl", "ground_support_height.png"]
    files += [p.name for p in sorted(bundle.glob("collision_*_hard_contact.stl"))]
    for name in files:
        shutil.copy2(bundle / name, template / name)

    replacements = {str(bundle): str(template)}
    protected = {}
    for name, expected in metadata["original_source_hashes"].items():
        source = Path(name)
        if source.is_relative_to(Path(metadata["source_scene"])):
            target = source
        else:
            target = template / "reference" / source.name
            target.parent.mkdir(exist_ok=True)
            shutil.copy2(source, target)
            replacements[str(source)] = str(target)
        protected[str(target)] = expected
    source_workspace = Path(metadata["nav2_simulation_overlay"]).parents[3]
    replacements[str(source_workspace)] = str(workspace)
    tree = ET.parse(template / "world.sdf")
    for element in tree.iter():
        if element.text:
            prefix = "file://" if element.text.startswith("file://") else ""
            element.text = prefix + remap_text(element.text[len(prefix):], replacements)
        for key, value in element.attrib.items():
            element.set(key, remap_text(value, replacements))
    tree.write(template / "world.sdf", encoding="unicode", xml_declaration=True)
    for name in ("georeference.yaml", "nav2_params.yaml"):
        value = remap_tree(yaml.safe_load((template / name).read_text()), replacements)
        (template / name).write_text(yaml.safe_dump(value, sort_keys=False))
    original_yaml = Path(read_json(bundle / "preparation.json")["map_base"]).with_suffix(".yaml")
    original_map = yaml.safe_load(original_yaml.read_text())
    image_name = original_map["image"]
    if Path(image_name).name != image_name:
        raise ValueError("The accepted real map must use a colocated PGM filename")
    shutil.copy2(original_yaml, template / "map.yaml")
    shutil.copy2(original_yaml.parent / image_name, template / image_name)
    if planning_parameters is not None:
        parameters = Path(planning_parameters).resolve()
        shutil.copy2(parameters, template / "reference/planning_parameters.yaml")
        source_config = yaml.safe_load(parameters.read_text())
        planning = {key: source_config[key] for key in ("planner_server", "global_costmap")}
        planner = planning["planner_server"]["ros__parameters"]
        costmap = planning["global_costmap"]["global_costmap"]["ros__parameters"]
        planner["use_sim_time"] = costmap["use_sim_time"] = True
        real_config = Path(read_json(bundle / "preparation.json")["nav2_simulation_overlay"]).parents[2] / "agribot_hardware_bringup/differential/config"
        primitives = real_config / "motion_primitives/diff_5cm.json"
        shutil.copy2(primitives, template / "reference/diff_5cm.json")
        planner["GridBased"]["lattice_filepath"] = str(template / "reference/diff_5cm.json")
        # Match the empty-sensor static planning state of the real saved route.
        for source in ("lidar_mark", "lidar_clear"):
            costmap["stvl_layer"][source]["topic"] = "/fuxi_route/reference_only/" + source
        (template / "route_planner.yaml").write_text(yaml.safe_dump({"fuxi_route": planning}, sort_keys=False))
        metadata["route_planning"] = {"mode": "smac_preset_waypoints", "manual_point_count": 104,
            "map_yaml_sha256": digest(template / "map.yaml"), "map_pgm_sha256": digest(template / image_name),
            "source_parameters_sha256": digest(parameters), "primitives_sha256": digest(primitives),
            "planner_plugin": planner["GridBased"]["plugin"], "require_exact_reference_match": True}
        for file in ("route_planner.yaml", "reference/planning_parameters.yaml", "reference/diff_5cm.json"):
            protected[str(template / file)] = digest(template / file)
    scene = read_json(template / "scene.json")
    scene.update(id=PROFILE_ID, manual_goals_only=False)
    write_json(template / "scene.json", scene)
    for name in ("world.sdf", "scene.json", "map.yaml", image_name, "nav2_params.yaml", "saved_route.json"):
        protected[str(template / name)] = digest(template / name)
    metadata = remap_tree(metadata, replacements)
    metadata["original_source_hashes"] = protected
    metadata["baseline_validation"] = str(destination / "baseline_result.json")
    write_json(template / "preparation.json", metadata)
    shutil.copy2(bundle / "result.json", destination / "baseline_result.json")
    shutil.copy2(bundle / "independent_acceptance.json", destination / "baseline_independent_acceptance.json")
    generated = workspace / "src/agribot_vehicle_description/generated" / VEHICLE_ID
    vehicle = read_json(generated / "vehicle_config.json")
    validate_vehicle(vehicle, read_json(template / "reference/vehicle_config.json"))
    profile = {"schema_version": 1, "id": PROFILE_ID, "name": "伏羲农场", "vehicle_id": VEHICLE_ID,
               "algorithm_id": ALGORITHM, "template": "template", "baseline_bundle": str(bundle),
               "route_mode": "smac_preset_waypoints" if planning_parameters is not None else "saved_path",
               "route_sha256": digest(template / "saved_route.json"),
               "template_hashes": {str(p.relative_to(destination)): digest(p)
                                   for p in sorted(template.rglob("*")) if p.is_file()},
               "vehicle_hashes": {str(p.relative_to(generated)): digest(p)
                                  for p in sorted(generated.rglob("*")) if p.is_file()}}
    write_json(destination / "profile.json", profile)
    return destination / "profile.json"


def prepare_run(profile_file, export_config, workspace, runs_directory, algorithm_id=ALGORITHM):
    profile_file, workspace = Path(profile_file).resolve(), Path(workspace).resolve()
    profile = read_json(profile_file)
    if (profile.get("schema_version") != 1 or profile.get("id") != PROFILE_ID
            or profile.get("algorithm_id") != ALGORITHM or profile.get("vehicle_id") != VEHICLE_ID):
        raise ValueError("Unknown fixed-route simulation profile")
    root = profile_file.parent
    for relative, expected in profile["template_hashes"].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or digest(path) != expected:
            raise ValueError("Fixed simulation input changed: " + relative)
    template = root / profile["template"]
    route = read_json(template / "saved_route.json")
    check_route(route)
    if digest(template / "saved_route.json") != profile["route_sha256"]:
        raise ValueError("The original fixed route has changed")
    expected_vehicle = read_json(template / "reference/vehicle_config.json")
    validate_vehicle(read_json(export_config), expected_vehicle)
    generated = workspace / "src/agribot_vehicle_description/generated" / VEHICLE_ID
    for relative, expected in profile["vehicle_hashes"].items():
        if digest(generated / relative) != expected:
            raise ValueError("Validated Car3 model changed: " + relative)
    # A fresh report directory must never inherit a previous passed result.
    run = Path(runs_directory).resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True)
    for source in template.iterdir():
        if source.is_dir():
            continue
        if source.suffix in (".stl", ".png", ".pgm"):
            (run / source.name).symlink_to(source)
        else:
            shutil.copy2(source, run / source.name)
    if algorithm_id != ALGORITHM:
        points = route.get("manual_poses") or [route["path"][i] for i in route["anchor_path_indices"]]
        if len(points) != 104:
            raise ValueError("All 104 ordered scene waypoints are required")
        (run / "waypoints.yaml").write_text(yaml.safe_dump({"frame_id": "map", "waypoints": points}))
    write_json(run / "software_launch.json", {"profile": str(profile_file), "name": profile["name"],
               "algorithm_id": algorithm_id, "vehicle_id": VEHICLE_ID, "unity_export": str(Path(export_config).resolve()),
               "route_sha256": profile["route_sha256"], "source": "Unity simulation launcher"})
    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    install = commands.add_parser("install")
    install.add_argument("bundle", type=Path)
    install.add_argument("destination", type=Path)
    install.add_argument("--workspace", type=Path, required=True)
    install.add_argument("--planning-parameters", type=Path, help="Exact real Smac/global-costmap parameters used for the original 104-point plan")
    run = commands.add_parser("prepare-run")
    run.add_argument("profile", type=Path)
    run.add_argument("--export-config", type=Path, required=True)
    run.add_argument("--workspace", type=Path, required=True)
    run.add_argument("--runs-directory", type=Path, required=True)
    run.add_argument("--algorithm", default=ALGORITHM)
    args = parser.parse_args()
    if args.command == "install":
        print(install_profile(args.bundle, args.destination, args.workspace, args.planning_parameters))
    else:
        print(prepare_run(args.profile, args.export_config, args.workspace, args.runs_directory, args.algorithm))


if __name__ == "__main__":
    main()
