"""Review compatibility and run the existing isolated acceptance checks serially."""

import argparse
import copy
import hashlib
import importlib.util
import itertools
import json
import math
import re
from pathlib import Path
import subprocess
import sys

import yaml
from PIL import Image, ImageDraw

from agribot_vehicle_config_tools.model import effective_sensor_pose, footprint, load_and_validate
from agribot_vehicle_config_tools.implements import collision_boxes
from agribot_vehicle_config_tools.generator import generate_fastlivo_config
from agribot_vehicle_config_tools.manipulator import default_manipulator
from .scenarios import SCENARIO_IDS, build, route, vehicle_spawn


GROUP_ORDER = ("perception", "vision", "localization", "planner", "controller")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_catalog(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("agribot_farm_sim")) / "config/compatibility.json"
    data = json.loads(Path(path).read_text())
    if data.get("schemaVersion") != 1 or data.get("scope") != "simulation_only":
        raise ValueError("Unsupported compatibility catalog")
    if set(data["components"]) != set(GROUP_ORDER):
        raise ValueError("Component groups must match the production interface")
    for options in data["components"].values():
        if not options or len(set(options)) != len(options):
            raise ValueError("Component options must be nonempty and unique")
    for scenes in data["sceneGroups"].values():
        if not scenes or len(set(scenes)) != len(scenes) or not set(scenes) <= set(SCENARIO_IDS):
            raise ValueError("Unknown or duplicate scene in policy")
    for key in ("maxHorizontalRmseM", "maxHorizontalErrorM"):
        value = data.get("validationLimits", {}).get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("Expected positive, finite validation limits")
    cars = {item["car"]: item for item in data["vehicles"]}
    if len(cars) != 10 or set(cars) != {f"Car{i}" for i in range(10)}:
        raise ValueError("Expected the ten actual garage vehicles")
    keys = [(item["car"], item["implement"]) for item in data["bindings"]]
    if len(set(keys)) != len(keys):
        raise ValueError("Duplicate mounting binding")
    algorithms = {"_".join(case) for case in component_cases(data)}
    if data.get("defaultAlgorithm") not in algorithms:
        raise ValueError("Unknown default test algorithm")

    def validate_scene_choices(choices):
        if not isinstance(choices, dict) or not set(choices) <= set(SCENARIO_IDS):
            raise ValueError("Unknown scene in algorithm selection")
        for choice in choices.values():
            if not isinstance(choice, dict) or choice.get("algorithm") not in algorithms:
                raise ValueError("Unknown scene test algorithm")
            groups = choice.get("vehicleSceneGroups")
            if groups is not None and (not isinstance(groups, list) or not groups
                    or any(not isinstance(group, str) for group in groups)
                    or not set(groups) <= set(data["sceneGroups"])):
                raise ValueError("Unknown vehicle scene group in algorithm selection")
            if not isinstance(choice.get("reason"), str) or not choice["reason"].strip():
                raise ValueError("Scene algorithm selection needs a reason")
            conditions = choice.get("conditions")
            if not isinstance(conditions, list) or not conditions or any(
                    not isinstance(item, str) or not item.strip() for item in conditions):
                raise ValueError("Scene algorithm selection needs explicit conditions")
            allowed = choice.get("allowedLocalizationModes")
            if allowed is not None:
                selected = next(values[GROUP_ORDER.index("localization")] for values in component_cases(data)
                                if "_".join(values) == choice["algorithm"])
                if (not isinstance(allowed, list) or not allowed
                        or any(not isinstance(mode, str) for mode in allowed)
                        or len(set(allowed)) != len(allowed)
                        or not set(allowed) <= set(data["components"]["localization"]) or selected not in allowed):
                    raise ValueError("Invalid allowed localization modes in scene selection")

    validate_scene_choices(data.get("sceneAlgorithms", {}))
    for vehicle in data["vehicles"]:
        validate_scene_choices(vehicle.get("sceneAlgorithms", {}))
    for binding in data["bindings"]:
        if binding["car"] not in cars:
            raise ValueError("Unknown vehicle in mounting binding")
        if type(binding.get("navigationRecommendationAllowed", True)) is not bool:
            raise ValueError("Navigation recommendation policy must be boolean")
        if "testAlgorithm" in binding:
            raise ValueError("Mounting-wide testAlgorithm must be scoped with sceneAlgorithms")
        validate_scene_choices(binding.get("sceneAlgorithms", {}))
        if binding["implement"] == "lite6" and binding["car"] not in ("Car0", "Car3"):
            raise ValueError("Arm mount is not supported on this vehicle")
    return data


def binding_id(binding):
    return binding["car"] + "__" + binding["implement"]


def component_cases(catalog):
    return list(itertools.product(*(catalog["components"][key] for key in GROUP_ORDER)))


def select_algorithm(catalog, binding, scenario):
    """Select an offline test recipe, not a runtime switch or acceptance claim."""
    if scenario not in SCENARIO_IDS:
        raise ValueError("Unknown scene in algorithm selection")
    choice = binding.get("sceneAlgorithms", {}).get(scenario)
    source = "scene_binding" if choice else "physical_default"
    if not choice:
        car = next(item for item in catalog["vehicles"] if item["car"] == binding["car"])
        choice = car.get("sceneAlgorithms", {}).get(scenario)
        if choice:
            source = "scene_vehicle"
    if not choice:
        choice = catalog.get("sceneAlgorithms", {}).get(scenario)
        if choice and choice.get("vehicleSceneGroups"):
            car = next(item for item in catalog["vehicles"] if item["car"] == binding["car"])
            group = binding.get("sceneGroup", car["sceneGroup"])
            if group not in choice["vehicleSceneGroups"]:
                choice = None
        if choice:
            source = "scene_default"
    algorithm = choice["algorithm"] if choice else catalog["defaultAlgorithm"]
    components = next(dict(zip(GROUP_ORDER, values)) for values in component_cases(catalog)
                      if "_".join(values) == algorithm)
    allowed = catalog.get("sceneAlgorithms", {}).get(scenario, {}).get("allowedLocalizationModes")
    if choice and "allowedLocalizationModes" in choice:
        allowed = [mode for mode in choice["allowedLocalizationModes"] if allowed is None or mode in allowed]
    if allowed is not None and components["localization"] not in allowed:
        raise ValueError("Selected localization violates the scene localization policy")
    return {
        "algorithm": algorithm,
        "source": source,
        "components": components,
        "reason": choice["reason"] if choice else "优先使用已验证的真机默认算法组合；本场景仍需独立完成路线验证。",
        "conditions": list(choice["conditions"]) if choice else [
            "雷达、IMU、相机输入及安装参数有效；RTK可用时提供全局约束，不可用时需有效初始定位。",
            "沿用本车与机具组合的运动学、速度限制和完整缓冲外廓，不以缩小轮廓或删点通过测试。"],
        "claim": "test_recipe_not_runtime_acceptance",
        **({"allowedLocalizationModes": list(allowed)} if allowed is not None else {}),
    }


def configuration(binding, generated):
    path = Path(generated) / binding["preset"] / "vehicle_config.json"
    config, warnings = load_and_validate(path)
    if config["vehicleId"] != binding["preset"]:
        raise ValueError("Vehicle ID differs from the exported preset")
    wanted = [] if binding["implement"] in ("none", "lite6") else [binding["implement"]]
    if [item["implementType"] for item in config.get("implements", [])] != wanted:
        raise ValueError("Exported implement differs from the allowed mounting binding")
    config = copy.deepcopy(config)
    # Explicitly test the bare car even when the last Unity export had an arm.
    config["manipulators"] = (config.get("manipulators") or [default_manipulator(config)]) \
        if binding["implement"] == "lite6" else []
    return config, warnings, path


def installation_review(config):
    findings = []
    for sensor in config["sensors"]:
        if sensor["sensorType"] not in ("lidar", "camera"):
            continue
        point = effective_sensor_pose(sensor)["xyz"]
        for index, box in enumerate(collision_boxes(config)):
            if all(box[i] <= point[i] <= box[i + 3] for i in range(3)):
                findings.append({"sensor": sensor["id"], "implementProxy": index,
                                 "reason": "Measurement origin is inside an implement collision proxy; check physical clearance and sensor self-occlusion"})
    return findings


def pose_occupancy(image, metadata, polygon, pose):
    """Conservative raster footprint check, not a path-feasibility proof."""
    x, y, yaw = pose
    ox, oy, oyaw = metadata["origin"]
    resolution = metadata["resolution"]
    vertices = []
    for px, py in polygon:
        wx = x + math.cos(yaw) * px - math.sin(yaw) * py - ox
        wy = y + math.sin(yaw) * px + math.cos(yaw) * py - oy
        mx = (math.cos(oyaw) * wx + math.sin(oyaw) * wy) / resolution
        my = (-math.sin(oyaw) * wx + math.cos(oyaw) * wy) / resolution
        vertices.append((mx, image.height - 1 - my))
    if any(x < 0 or y < 0 or x >= image.width or y >= image.height for x, y in vertices):
        return "outside"
    left = math.floor(min(p[0] for p in vertices))
    top = math.floor(min(p[1] for p in vertices))
    right = math.ceil(max(p[0] for p in vertices)) + 1
    bottom = math.ceil(max(p[1] for p in vertices)) + 1
    mask = Image.new("L", (right - left, bottom - top))
    ImageDraw.Draw(mask).polygon([(x - left, y - top) for x, y in vertices], fill=255)
    histogram = image.crop((left, top, right, bottom)).histogram(mask)
    unknown = False
    for value, count in enumerate(histogram):
        if not count:
            continue
        occupancy = value / 255.0 if metadata.get("negate", 0) else 1 - value / 255.0
        if occupancy > metadata["occupied_thresh"]:
            return "collision"
        if occupancy >= metadata["free_thresh"]:
            unknown = True
    return "unknown" if unknown else "free"


def scenario_review(config, polygon, scenario, scene_entry, map_path, waypoints):
    metadata = yaml.safe_load(Path(map_path).read_text())
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = Path(map_path).parent / image_path
    with Image.open(image_path) as original:
        image = original.convert("L")
    spawn = vehicle_spawn(config, scenario, scene_entry["spawn"])
    checks = [("spawn", spawn), *[(f"waypoint_{i + 1}", p) for i, p in enumerate(waypoints)]]
    collisions = [{"pose": name, "status": status}
                  for name, position in checks
                  if (status := pose_occupancy(image, metadata, polygon, position)) != "free"]
    status = ("POSES_CLEAR" if not collisions else
              "UNKNOWN_SPACE_ONLY" if all(item["status"] == "unknown" for item in collisions) else
              "MISSION_REVIEW_REQUIRED")
    distance = sum(math.hypot(end[0] - start[0], end[1] - start[1])
                   for (_, start), (_, end) in zip(checks, checks[1:]))
    return {"status": status,
            "checkedPoses": len(checks), "conflicts": collisions,
            "routeDistanceLowerBoundM": distance,
            "claim": "Only authored pose footprints checked; no swept path, terrain, height or turning proof"}


def route_runtime_budget(distance, max_speed):
    """Bound wall-clock observation, without changing route or acceptance gates."""
    if (type(distance) not in (int, float) or not math.isfinite(distance) or distance < 0
            or type(max_speed) not in (int, float) or not math.isfinite(max_speed) or max_speed <= 0):
        raise ValueError("Runtime budget requires finite distance and a positive speed limit")
    seconds = max(2400, math.ceil(distance / max_speed * 2.5 + 180))
    if seconds > 14400:
        raise ValueError("Route observation exceeds four hours and needs an explicit execution review")
    return {"motionTimeoutS": seconds, "distanceLowerBoundM": distance,
            "maxSpeedMps": max_speed, "wallTimeFactor": 2.5, "startupAllowanceS": 180,
            "claim": "Observation budget only; original waypoints and acceptance gates are unchanged"}


def runtime_plan(catalog, suite="mountings"):
    """Separate mounting, component and scene coverage instead of implying Cartesian coverage."""
    default = catalog["defaultAlgorithm"]
    bindings = {binding_id(item): item for item in catalog["bindings"]}
    result, seen = [], set()

    def add(binding, scenario, algorithm, distance=4., lateral=0., selection=None):
        case_id = binding_id(binding) + "__" + scenario + "__" + algorithm
        if case_id in seen:
            return
        seen.add(case_id)
        result.append({"id": case_id, "binding": binding_id(binding), "vehicleId": binding["preset"],
                       "scenario": scenario, "algorithm": algorithm, "scope": "smoke",
                       "manipulator": "true" if binding["implement"] == "lite6" else "false",
                       "armDemo": "false", "distanceM": distance, "lateralM": lateral})
        if selection is not None:
            result[-1]["algorithmSelection"] = selection

    if suite in ("mountings", "baseline"):
        for binding in bindings.values():
            large = binding["car"] == "Car9" or binding["implement"] == "farm_trailer"
            scenario = binding.get("testScenario", "arable_field" if large else "tomato_greenhouse")
            selection = select_algorithm(catalog, binding, scenario)
            add(binding, scenario, selection["algorithm"],
                binding.get("testDistanceM", 6. if large else 4.),
                binding.get("testLateralM", .4 if binding["implement"] == "farm_trailer" else 0.), selection)
    if suite in ("components", "baseline"):
        groups = [catalog["components"][key] for key in GROUP_ORDER]
        base = [values[0] for values in groups]
        variants = [tuple(base)]
        for index, values in enumerate(groups):
            for value in values[1:]:
                variant = base.copy()
                variant[index] = value
                variants.append(tuple(variant))
        for car in ("Car0", "Car3"):
            for case in variants:
                add(bindings[car + "__none"], "tomato_greenhouse", "_".join(case))
    if suite in ("scenes", "baseline"):
        for scene in catalog["sceneGroups"]["compact"]:
            add(bindings["Car0__none"], scene, default)
    if suite == "matrix":
        cars = {v["car"]: v for v in catalog["vehicles"]}
        for binding in bindings.values():
            group = binding.get("sceneGroup", cars[binding["car"]]["sceneGroup"])
            for scene in catalog["sceneGroups"][group]:
                for case in component_cases(catalog):
                    add(binding, scene, "_".join(case), 6. if group == "harvester" else 4.)
    return result


def route_plan(catalog, static_audit, scenario, *, use_scene_hints=True):
    """Schedule original routes without known collisions, retaining unknown-space warnings."""
    records = {r["id"]: r for r in static_audit["bindings"]}
    cases, excluded = [], []
    for binding in catalog["bindings"]:
        key = binding_id(binding)
        if not binding.get("navigationRecommendationAllowed", True):
            excluded.append({"binding": key, "scenario": scenario,
                             "reason": binding.get("reviewReason", "Mounting-only review; navigation not qualified")})
            continue
        record = records.get(key, {})
        review = record.get("scenes", {}).get(scenario, {})
        selection = select_algorithm(catalog, binding, scenario)
        components = selection["components"]
        localization = "navsat" if components["localization"] == "navsat" else "fastlio"
        profile = "/".join((localization, components["perception"], components["controller"], components["planner"]))
        unknown_only = (review.get("status") == "UNKNOWN_SPACE_ONLY" and bool(review.get("conflicts"))
                        and all(item.get("status") == "unknown" for item in review["conflicts"])
                        and record.get("unknownTraversalPolicies", {}).get(profile) is True)
        if (record.get("status") != "PASS" or (review.get("status") != "POSES_CLEAR" and not unknown_only)
                or (use_scene_hints and review.get("policy") != "candidate")):
            excluded.append({"binding": key, "scenario": scenario,
                             "reason": "Authored route needs review; this is not proof that the vehicle cannot enter the scene",
                             "configurationStatus": record.get("status", "MISSING"),
                             "routeStatus": review.get("status", "MISSING"),
                             "conflicts": review.get("conflicts", []),
                             "sceneHintApplied": use_scene_hints})
            continue
        algorithm = selection["algorithm"]
        cases.append({"id": key + "__" + scenario + "__full_route__" + algorithm,
                      "binding": key, "vehicleId": binding["preset"], "scenario": scenario,
                      "algorithm": algorithm, "algorithmSelection": selection, "scope": "full-route",
                      "manipulator": "true" if binding["implement"] == "lite6" else "false",
                      "armDemo": "false"})
        if "routeDistanceLowerBoundM" in review and "maxSpeedMps" in record:
            budget = route_runtime_budget(review["routeDistanceLowerBoundM"], record["maxSpeedMps"])
            cases[-1].update(motionTimeoutS=budget["motionTimeoutS"], runtimeBudget=budget)
        if unknown_only:
            cases[-1]["routeWarnings"] = {
                "status": "UNKNOWN_SPACE_ONLY", "poses": copy.deepcopy(review["conflicts"]),
                "plannerProfile": profile, "allowUnknown": True,
                "claim": "Existing planner permits unknown traversal; runtime obstacle and full-route checks still required"}
    return cases, excluded


def validate_plan(cases, catalog):
    bindings = {binding_id(b): b for b in catalog["bindings"]}
    algorithms = {"_".join(c) for c in component_cases(catalog)}
    seen = set()
    if not cases:
        raise ValueError("Runtime plan is empty")
    for case in cases:
        try:
            name = case["id"]
            if not isinstance(name, str) or len(name) > 180 or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name) or name in seen:
                raise ValueError("Unsafe or duplicate case ID")
            seen.add(name)
            binding = bindings[case["binding"]]
            arm = "true" if binding["implement"] == "lite6" else "false"
            if case["vehicleId"] != binding["preset"] or case["manipulator"] != arm:
                raise ValueError("Test does not match its declared mounting binding")
            if case["scenario"] not in SCENARIO_IDS or case["algorithm"] not in algorithms:
                raise ValueError("Unknown scenario or algorithm")
            selection = case.get("algorithmSelection")
            if selection is not None and (selection != select_algorithm(catalog, binding, case["scenario"])
                                           or selection["algorithm"] != case["algorithm"]):
                raise ValueError("Algorithm selection differs from the current scene recipe")
            if case["scope"] not in ("smoke", "full-route") or case["armDemo"] != "false":
                raise ValueError("Unsupported validation scope or moving-arm demonstration")
            if "motionTimeoutS" in case:
                timeout = case["motionTimeoutS"]
                if case["scope"] != "full-route" or type(timeout) is not int or not 2400 <= timeout <= 14400:
                    raise ValueError("Invalid full-route observation budget")
            if case["scope"] == "smoke":
                distance, lateral = case["distanceM"], case["lateralM"]
                if type(distance) not in (int, float) or not math.isfinite(distance) or distance < 1.5:
                    raise ValueError("Invalid maneuver distance")
                if type(lateral) not in (int, float) or not math.isfinite(lateral):
                    raise ValueError("Invalid maneuver lateral offset")
        except (KeyError, TypeError) as error:
            raise ValueError("Malformed runtime plan case") from error


def audit(catalog, description, scene_share, destination):
    description, destination = Path(description), Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    scenes = {item["id"]: item for item in json.loads((Path(scene_share) / "config/scenarios.json").read_text())["scenarios"]}
    from ament_index_python.packages import get_package_share_directory
    orchard_share = Path(get_package_share_directory("agribot_ackermann_mppi"))
    scene_paths = {}
    for scene in scenes:
        if scene == "orchard":
            navigation = Path(get_package_share_directory("scout_navigation"))
            scene_paths[scene] = (navigation / "maps/orchard_v2_map6.yaml",
                yaml.safe_load((orchard_share / "config/orchard_waypoints_ackermann_smac.yaml").read_text())["waypoints"])
        else:
            directory = build(scene, destination / "scenes")
            scene_paths[scene] = (directory / "map.yaml", [dict(x=x, y=y, yaw=yaw) for x, y, yaw in route(scene)])
    spec = importlib.util.spec_from_file_location("compatibility_production_launch", description / "launch/configured_ackermann_sim.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    cars = {item["car"]: item for item in catalog["vehicles"]}
    report = {"schemaVersion": 1, "scope": "static_audit_not_runtime", "bindings": [],
              "algorithmCombinations": len(component_cases(catalog)), "limitations": catalog["limitations"]}
    for binding in catalog["bindings"]:
        record = dict(binding, id=binding_id(binding), status="PASS", scenes={}, profileChecks=0,
                      unknownTraversalPolicies={})
        try:
            config, warnings, config_path = configuration(binding, description / "generated")
            if config["vehicleType"] != cars[binding["car"]]["type"]:
                raise ValueError("Motion type differs from compatibility catalog")
            points = launch._body_navigation_footprint(config)
            extrinsics = yaml.safe_load((description / "generated" / binding["preset"] / "config/fastlivo_extrinsics.yaml").read_text())
            if extrinsics != generate_fastlivo_config(config):
                raise ValueError("Generated localization extrinsics differ from the canonical mounting poses")
            record.update(configSha256=digest(config_path), warnings=warnings, footprint=points,
                          installationReview=installation_review(config),
                          navigationFootprintPolicy="host_body_with_clearance_excluding_implements",
                          minTurningRadiusM=config["geometry"]["minTurningRadiusM"],
                          maxSpeedMps=config["navigation"]["maxLinearSpeedMps"])
            for loc in ("fastlio", "navsat"):
                source = description / "generated" / binding["preset"] / "config/simulation" / f"nav2_params_ackermann_{loc}_static.yaml"
                profiles = launch._write_nav2_profiles(source, description / "config",
                    "compat_" + record["id"] + "_" + loc, launch._uses_local_cad_profile(config),
                    navigation_footprint=points,
                    final_position_tolerance=launch._final_position_tolerance(config))
                for (perception, controller, planner), path in profiles.items():
                    params = yaml.safe_load(Path(path).read_text())
                    profile = "/".join((loc, perception, controller, planner))
                    record["unknownTraversalPolicies"][profile] = (
                        params["planner_server"]["ros__parameters"]["GridBased"].get("allow_unknown") is True)
                    for costmap in ("global_costmap", "local_costmap"):
                        actual = json.loads(params[costmap][costmap]["ros__parameters"]["footprint"])
                        if actual != points:
                            raise ValueError("A navigation profile lost the current host-body footprint")
                    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
                    speed_key = {"mppi": "vx_max", "rpp": "desired_linear_vel", "dwb": "max_vel_x"}[controller]
                    if not math.isclose(follow[speed_key], config["navigation"]["maxLinearSpeedMps"], abs_tol=1e-9):
                        raise ValueError("Controller lost the exported speed limit")
                    if controller == "mppi":
                        expected = "Ackermann" if config["vehicleType"] == "ackermann" else "DiffDrive"
                        if follow["motion_model"] != expected:
                            raise ValueError("Controller motion model does not match vehicle")
                    elif controller == "dwb":
                        expected = ("agribot_vehicle_description::AckermannTrajectoryGenerator"
                            if config["vehicleType"] == "ackermann" else "dwb_plugins::StandardTrajectoryGenerator")
                        if follow["trajectory_generator_name"] != expected:
                            raise ValueError("DWB trajectory generator does not match vehicle")
                    record["profileChecks"] += 1
            group = binding.get("sceneGroup", cars[binding["car"]]["sceneGroup"])
            for scene, entry in scenes.items():
                map_path, waypoints = scene_paths[scene]
                if scene != "orchard":
                    waypoints = [dict(x=x, y=y, yaw=yaw) for x, y, yaw in route(scene, config)]
                geometry = scenario_review(config, points, scene, entry, map_path,
                    [(p["x"], p["y"], p["yaw"]) for p in waypoints])
                geometry["policy"] = "candidate" if scene in catalog["sceneGroups"][group] else "not_recommended"
                record["scenes"][scene] = geometry
        except (ValueError, OSError, KeyError, StopIteration) as error:
            record.update(status="FAIL", error=str(error))
        report["bindings"].append(record)
    report["passed"] = sum(item["status"] == "PASS" for item in report["bindings"])
    (destination / "audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def evidence_record(summary, result, cache=None, verification_recheck=None):
    mismatches = []
    fingerprints = summary.get("file_sha256", {})
    if not fingerprints:
        mismatches.append("missing_fingerprints")
    for name, expected in fingerprints.items():
        path = Path(name)
        current = None
        if path.is_file():
            stat = path.stat()
            key = (str(path.resolve()), stat.st_size, stat.st_mtime_ns)
            if cache is None:
                current = digest(path)
            else:
                if key not in cache:
                    cache[key] = digest(path)
                current = cache[key]
        compatible_verifier = (verification_recheck and verification_recheck.get("compatibleFingerprint")
                              and name == verification_recheck["verifierPath"]
                              and expected == verification_recheck["legacyVerifierSha256"]
                              and current == verification_recheck["currentVerifierSha256"])
        if current != expected and not compatible_verifier:
            mismatches.append(name)
    complete = result.get("status") == "PASS" and not result.get("abnormal_process_exits")
    return {"status": result.get("status", "UNKNOWN"), "scope": result.get("scope", summary.get("scope")),
            "algorithm": result.get("id"), "vehicleId": summary.get("vehicle_id"),
            "scenario": summary.get("scenario"), "manipulator": summary.get("manipulator", "configured"),
            "currentInputsMatch": not mismatches, "changedInputs": mismatches,
            "verificationRecheck": verification_recheck,
            "fullRoute": complete and result.get("scope", summary.get("scope")) == "full-route" and result.get("full_route") == "PASS",
            "eligibleForSimulationRecommendation": complete and not mismatches and result.get("scope", summary.get("scope")) == "full-route" and result.get("full_route") == "PASS",
            "horizontalRmseM": result.get("horizontal_pose_rmse_m"),
            "horizontalMaxErrorM": result.get("horizontal_pose_max_error_m"), "error": result.get("error")}


def localization_quality_errors(result, limits):
    errors = []
    for field, limit_key in (("horizontal_pose_rmse_m", "maxHorizontalRmseM"),
                             ("horizontal_pose_max_error_m", "maxHorizontalErrorM")):
        value = result.get(field)
        if type(value) not in (int, float) or not math.isfinite(value):
            errors.append("Missing finite measurement: " + field)
        elif value > limits[limit_key]:
            errors.append(f"{field}={value:.4f} exceeds {limits[limit_key]}")
    return errors


def collect_evidence(catalog, roots, acceptance_policy=None):
    """Recheck inputs and keep failures; never transfer PASS across vehicles or scenes."""
    from .acceptance_rechecks import load_acceptance_policy, recheck_legacy_result
    policy = load_acceptance_policy(acceptance_policy)
    records, cache, seen = [], {}, set()
    algorithms = {"_".join(case): dict(zip(GROUP_ORDER, case)) for case in component_cases(catalog)}
    cars = {item["car"]: item for item in catalog["vehicles"]}
    for root in roots:
        if not Path(root).is_dir():
            raise ValueError("Runtime evidence root does not exist: " + str(root))
        for path in sorted(Path(root).rglob("summary.json")):
            if path.resolve() in seen:
                continue
            seen.add(path.resolve())
            summary = json.loads(path.read_text())
            if "vehicle_id" not in summary or "file_sha256" not in summary:
                continue
            for result in summary.get("results", []):
                record = evidence_record(summary, result, cache, recheck_legacy_result(path, summary, result, policy))
                quality_errors = localization_quality_errors(result, catalog["validationLimits"])
                record["qualityStatus"] = "FAIL" if quality_errors else "PASS"
                record["qualityErrors"] = quality_errors
                record["source"] = str(path)
                record["binding"] = None
                configured_arm = None
                if record["manipulator"] == "configured" and record["currentInputsMatch"]:
                    config_paths = [p for p in summary["file_sha256"] if p.endswith("/vehicle_config.json")]
                    if len(config_paths) == 1:
                        configured_arm = bool(json.loads(Path(config_paths[0]).read_text()).get("manipulators"))
                for binding in catalog["bindings"]:
                    has_arm = binding["implement"] == "lite6"
                    arm_matches = record["manipulator"] == str(has_arm).lower() or configured_arm == has_arm
                    if binding["preset"] == record["vehicleId"] and arm_matches:
                        record["binding"] = binding_id(binding)
                        group = binding.get("sceneGroup", cars[binding["car"]]["sceneGroup"])
                        record["sceneHint"] = record["scenario"] in catalog["sceneGroups"][group]
                        # A completed exact-input route is stronger evidence
                        # than the initial vehicle-size grouping heuristic.
                        record["candidateScene"] = record["scenario"] in SCENARIO_IDS
                        record["navigationPolicyAllowed"] = binding.get("navigationRecommendationAllowed", True)
                        record["policyReviewReason"] = binding.get("reviewReason")
                        break
                algorithm = algorithms.get(record["algorithm"], {})
                # The model may recommend a tested navigation combination, not
                # an unsafe direct baseline or untested Ackermann XY shortcut.
                record["eligibleForSimulationRecommendation"] &= bool(
                    record["binding"] and record.get("candidateScene") and record.get("navigationPolicyAllowed") and algorithm.get("planner") == "smac_hybrid" and not quality_errors)
                records.append(record)
    failed_full_routes = {(r["binding"], r["scenario"], r["algorithm"]) for r in records
                          if r["binding"] and r["currentInputsMatch"] and r["scope"] == "full-route"
                          and (not r["fullRoute"] or r["qualityStatus"] != "PASS")}
    for record in records:
        key = (record["binding"], record["scenario"], record["algorithm"])
        if record["eligibleForSimulationRecommendation"] and key in failed_full_routes:
            record["eligibleForSimulationRecommendation"] = False
            record["recommendationBlock"] = "Another full-route attempt with current inputs failed; a lucky pass is not repeatable acceptance"
    return records


def recommend_algorithm(catalog, binding, scenario, records):
    """Apply scene preference only among exact, current, successful full-route results."""
    recipe = select_algorithm(catalog, binding, scenario)
    matched = [r for r in records if r["binding"] == binding_id(binding) and r["scenario"] == scenario]
    current = [r for r in matched if r["currentInputsMatch"] and r["scope"] == "full-route"]
    failures = sorted({r["algorithm"] for r in current if not r["fullRoute"] or r["qualityStatus"] != "PASS"})
    algorithms = {"_".join(case): dict(zip(GROUP_ORDER, case)) for case in component_cases(catalog)}
    eligible = [r for r in current if r["eligibleForSimulationRecommendation"] and r["algorithm"] not in failures
                and (not recipe.get("allowedLocalizationModes") or
                     algorithms[r["algorithm"]]["localization"] in recipe["allowedLocalizationModes"])]
    default = algorithms[catalog["defaultAlgorithm"]]

    def priority(record):
        algorithm = record["algorithm"]
        return (algorithm != recipe["algorithm"], algorithm != catalog["defaultAlgorithm"],
                sum(value != default[key] for key, value in algorithms[algorithm].items()),
                record["horizontalRmseM"], record["horizontalMaxErrorM"], algorithm, record["source"])

    chosen = min(eligible, key=priority) if eligible else None
    policy_allowed = binding.get("navigationRecommendationAllowed", True)
    status = ("MOUNTING_REVIEW_REQUIRED" if not policy_allowed else
              "VALIDATED" if chosen else "REVIEW_REQUIRED" if failures else "PENDING_VALIDATION")
    if not policy_allowed:
        chosen = None
    selected = chosen["algorithm"] if chosen else None
    components = algorithms[selected] if selected else None
    source = (recipe["source"] if selected == recipe["algorithm"] else
              "physical_default" if selected == catalog["defaultAlgorithm"] else "tested_alternative") if chosen else None
    conditions = list(recipe["conditions"]) if selected == recipe["algorithm"] else []
    if chosen and not conditions:
        conditions = ["沿用该次已验证的车辆、机具、传感器安装、完整外廓、速度限制及原路线。"]
        if components["localization"] == "navsat":
            conditions.append("要求GNSS位置、IMU与初始航向有效；不能直接用于无GNSS的真实室内场景。")
        elif components["localization"] == "fastlivo_rtk":
            conditions.append("要求雷达、IMU、相机输入有效；RTK可用时约束漂移，不可用时需有效初始定位。")
    return {"binding": binding_id(binding), "vehicleId": binding["preset"], "scenario": scenario,
            "status": status, "algorithm": selected, "source": source, "components": components,
            "conditions": conditions, "testRecipe": recipe, "failedAlgorithms": failures,
            "historicalRecords": sum(not r["currentInputsMatch"] for r in matched),
            "evidence": [{key: r[key] for key in ("source", "horizontalRmseM", "horizontalMaxErrorM")}
                         for r in eligible if r["algorithm"] == selected],
            "reason": ("默认链路在该精确组合完成全路线且定位达标，优先沿用。" if source == "physical_default" else
                       "该场景优先配方在本精确组合有当前输入的全路线通过证据。" if source in ("scene_default", "scene_vehicle", "scene_binding") else
                       "优先配方尚无可用证据，采用已完成本精确组合全路线的替代链路。" if chosen else
                       binding.get("reviewReason", "挂载或作业策略尚待审查。") if not policy_allowed else
                       "已有全路线失败或定位超限记录，尚无可推荐的通过组合。" if failures else
                       "尚无该精确组合的当前输入全路线通过证据，仅提供待测试配方。"),
            "claim": "exact_simulation_route_not_global_optimum_or_physical_authorization"}


def evidence_report(catalog, roots, destination, acceptance_policy=None):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    records = collect_evidence(catalog, roots, acceptance_policy)
    candidates = {(r["binding"], r["scenario"], r["algorithm"]): r for r in records
                  if r["eligibleForSimulationRecommendation"]}
    selections = [recommend_algorithm(catalog, binding, scene, records)
                  for binding in catalog["bindings"] for scene in SCENARIO_IDS]
    report = {"schemaVersion": 1, "claim": "Evidence at exact recorded scope; NOT universal compatibility or physical work validation",
              "recommendationCandidates": list(candidates.values()), "algorithmSelections": selections,
              "validationLimits": catalog["validationLimits"], "records": records, "counts": {
                  "records": len(records),
                  "currentPass": sum(r["status"] == "PASS" and r["currentInputsMatch"] for r in records),
                  "currentFail": sum(r["status"] == "FAIL" and r["currentInputsMatch"] for r in records),
                  "currentQualityRejected": sum(r["status"] == "PASS" and r["qualityStatus"] != "PASS" and r["currentInputsMatch"] for r in records),
                  "historicalChangedInputs": sum(not r["currentInputsMatch"] for r in records),
                  "uniqueRecommendationCandidates": len(candidates),
                  "validatedSelections": sum(s["status"] == "VALIDATED" for s in selections),
                  "reviewRequiredSelections": sum(s["status"] == "REVIEW_REQUIRED" for s in selections),
                  "recommendableFullRoutes": sum(r["eligibleForSimulationRecommendation"] for r in records)}}
    (destination / "evidence.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    lines = ["# 农机组合验证记录", "", "仅按实际车型、机具、场景、算法及输入指纹统计；短程通过不等于全程通过。", "",
             "| 组合 | 短程链路通过 | 全程链路通过 | 运行失败 | 定位质量未达标 | 已变更输入记录 |", "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for binding in catalog["bindings"]:
        matched = [r for r in records if r["binding"] == binding_id(binding)]
        current = [r for r in matched if r["currentInputsMatch"]]
        short = sum(r["status"] == "PASS" and r["scope"] == "smoke" for r in current)
        full = sum(r["status"] == "PASS" and r["fullRoute"] for r in current)
        failed = sum(r["status"] == "FAIL" for r in current)
        quality_failed = sum(r["status"] == "PASS" and r["qualityStatus"] != "PASS" for r in current)
        lines.append(f"| {binding['car']} / {binding['name']} | {short} | {full} | {failed} | {quality_failed} | {len(matched) - len(current)} |")
    lines += ["", "## 当前全路线推荐候选", "",
              "仅列当前输入的路线与定位质量均通过，且没有同输入全程失败记录的组合；不代表其他场景或其他算法已通过。", "",
              "| 组合 | 场景 | 算法组合 | 验证记录 |",
              "| --- | --- | --- | --- |"]
    for record in candidates.values():
        lines.append(f"| {record['binding']} | {record['scenario']} | {record['algorithm']} | [记录]({record['source']}) |")
    lines += ["", "## 按场景、农机与机具选择", "",
              "仅从上面的有效全程证据中选择：优先真机默认，其次该场景配方，再次少量模块替换；不是全局最优或真车作业授权。",
              "所有组合及待验证状态保存在 evidence.json 的 algorithmSelections。", "",
              "| 组合 | 场景 | 选择状态 | 推荐算法 |", "| --- | --- | --- | --- |"]
    for selection in selections:
        if selection["status"] in ("VALIDATED", "REVIEW_REQUIRED"):
            lines.append(f"| {selection['binding']} | {selection['scenario']} | {selection['status']} | {selection['algorithm'] or '无，需解决失败'} |")
    lines += ["", "## 当前失败项", ""]
    for record in records:
        if record["currentInputsMatch"] and record["status"] == "FAIL":
            lines.append(f"- `{record['binding']}` / `{record['scenario']}` / `{record['algorithm']}`：{record['error']}；[记录]({record['source']})")
    lines += ["", "## 已完成但定位质量未达标", ""]
    for record in records:
        if record["currentInputsMatch"] and record["status"] == "PASS" and record["qualityErrors"]:
            lines.append(f"- `{record['binding']}` / `{record['scenario']}` / `{record['algorithm']}`：{'；'.join(record['qualityErrors'])}")
    lines += ["", "## 使用边界", ""] + ["- " + note for note in catalog["limitations"]]
    lines += ["", "定位质量另行核验：水平RMSE ≤ 0.30 m、最大误差 ≤ 0.75 m；这只是本次仿真推荐审查限值，不修改正式节点参数或原通过记录。", ""]
    (destination / "report.md").write_text("\n".join(lines) + "\n")
    return report


def run_plan(plan, output, runner, limit=None, validation_limits=None, *, gui=False, rviz=False, stall_timeout=0):
    import fcntl
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    lock_path = Path.home() / ".cache/agribot/farm-compatibility.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    failed = False
    with lock_path.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for index, case in enumerate(plan[:limit]):
            destination = output / case["id"]
            # Completed attempts are immutable; reruns need a new output root.
            if destination.exists() or (output / (case["id"] + ".runner.log")).exists() or (output / (case["id"] + ".attempt.json")).exists():
                raise FileExistsError("Existing attempt must not be overwritten: " + str(destination))
            command = [sys.executable, str(runner), "--vehicle-id", case["vehicleId"],
                       "--scenario", case["scenario"], "--case", case["algorithm"],
                       "--scope", case["scope"], "--manipulator", case["manipulator"],
                       "--arm-demo", case["armDemo"], "--output", str(destination),
                       "--domain", str(73 + index % 25), "--motion-timeout", "150"]
            if gui:
                command.append("--gui")
            if rviz:
                command.append("--rviz")
            if stall_timeout:
                command += ["--stall-timeout", str(stall_timeout)]
            if case["scope"] == "smoke":
                command += ["--local-maneuver", "--motion-distance", str(case["distanceM"]),
                            "--maneuver-lateral-offset", str(case["lateralM"])]
            else:
                timeout = case.get("motionTimeoutS", 2400)
                if type(timeout) is not int or not 2400 <= timeout <= 14400:
                    raise ValueError("Invalid full-route observation budget")
                command[command.index("--motion-timeout") + 1] = str(timeout)
            print(f"[{index + 1}/{min(len(plan), limit or len(plan))}] {case['id']}", flush=True)
            with (output / (case["id"] + ".runner.log")).open("w") as log:
                completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            quality = []
            summary = destination / "summary.json"
            if completed.returncode == 0 and validation_limits:
                if not summary.is_file():
                    quality.append("Missing runtime evidence")
                else:
                    results = json.loads(summary.read_text()).get("results", [])
                    if not results:
                        quality.append("Empty runtime evidence")
                    for result in results:
                        if result.get("status") != "PASS":
                            quality.append("Runtime evidence did not pass")
                        if case["scope"] == "full-route" and (
                                result.get("scope") != "full-route" or result.get("full_route") != "PASS"):
                            quality.append("Runtime evidence did not complete the requested full route")
                        quality.extend(localization_quality_errors(result, validation_limits))
            (output / (case["id"] + ".attempt.json")).write_text(json.dumps(
                {"case": case, "command": command, "returncode": completed.returncode,
                 "qualityErrors": quality}, indent=2) + "\n")
            quality_label = "FAIL: " + "; ".join(quality) if quality else (
                "PASS" if completed.returncode == 0 and validation_limits else "NOT_EVALUATED")
            print(f"  exit={completed.returncode}; quality={quality_label}", flush=True)
            failed = failed or completed.returncode != 0 or bool(quality)
            if completed.returncode in (130, -2):
                return 130
            if completed.returncode < 0:
                raise RuntimeError("Runner terminated by signal; refusing another simulation without verified cleanup")
            if summary.is_file() and any(item.get("cleanup_error") for item in json.loads(summary.read_text()).get("results", [])):
                raise RuntimeError("Cleanup failed; refusing to start another simulation")
    return int(failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path)
    commands = parser.add_subparsers(dest="command", required=True)
    select_cmd = commands.add_parser("select", help="Explain the recipe for an exact vehicle, implement and scene")
    select_cmd.add_argument("--car", required=True, choices=[f"Car{i}" for i in range(10)])
    select_cmd.add_argument("--implement", required=True)
    select_cmd.add_argument("--scenario", required=True, choices=SCENARIO_IDS)
    select_cmd.add_argument("--evidence-root", action="append", type=Path,
                            help="Recheck runtime records and select only a verified exact-combination recipe")
    audit_cmd = commands.add_parser("audit")
    audit_cmd.add_argument("--output", required=True, type=Path)
    plan_cmd = commands.add_parser("plan")
    plan_cmd.add_argument("--suite", choices=("mountings", "components", "scenes", "baseline", "matrix", "routes", "all-routes"), default="baseline")
    plan_cmd.add_argument("--audit", type=Path, help="Required for full-route plans")
    plan_cmd.add_argument("--scenario", choices=SCENARIO_IDS, default="tomato_greenhouse")
    plan_cmd.add_argument("--output", required=True, type=Path)
    run_cmd = commands.add_parser("run")
    run_cmd.add_argument("--plan", required=True, type=Path)
    run_cmd.add_argument("--output", required=True, type=Path)
    run_cmd.add_argument("--limit", type=int)
    run_cmd.add_argument("--gui", action="store_true", help="Show Gazebo for each serial runtime check")
    run_cmd.add_argument("--rviz", action="store_true", help="Show RViz for each serial runtime check")
    report_cmd = commands.add_parser("report")
    report_cmd.add_argument("--evidence-root", action="append", required=True, type=Path)
    report_cmd.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    catalog = read_catalog(args.catalog)
    if args.command == "select":
        binding = next((b for b in catalog["bindings"]
                        if b["car"] == args.car and b["implement"] == args.implement), None)
        if binding is None:
            parser.error("Vehicle and implement do not have a registered mounting binding")
        selection = (recommend_algorithm(catalog, binding, args.scenario, collect_evidence(catalog, args.evidence_root))
                     if args.evidence_root else
                     {"binding": binding_id(binding), "vehicleId": binding["preset"],
                      "scenario": args.scenario, **select_algorithm(catalog, binding, args.scenario)})
        print(json.dumps(selection, ensure_ascii=False, indent=2))
        return 0
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    if args.command == "plan":
        excluded = []
        if args.suite in ("routes", "all-routes"):
            if not args.audit:
                parser.error("full-route planning requires --audit")
            static = json.loads(args.audit.read_text())
            cases = []
            for scene in SCENARIO_IDS if args.suite == "all-routes" else (args.scenario,):
                planned, pending = route_plan(catalog, static, scene,
                                             use_scene_hints=args.suite != "all-routes")
                cases.extend(planned)
                excluded.extend(pending)
        else:
            cases = runtime_plan(catalog, args.suite)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            json.dump({"schemaVersion": 1, "suite": args.suite, "cases": cases, "excluded": excluded}, stream, ensure_ascii=False, indent=2)
        print(f"{len(cases)} sequential cases: {args.output}")
    elif args.command == "audit":
        report = audit(catalog, get_package_share_directory("agribot_vehicle_description"),
                       get_package_share_directory("agribot_farm_sim"), args.output)
        print(f"Static checks: {report['passed']}/{len(report['bindings'])}; NOT runtime acceptance")
        return int(report["passed"] != len(report["bindings"]))
    elif args.command == "run":
        plan = json.loads(args.plan.read_text())
        if plan.get("schemaVersion") != 1 or not plan.get("cases"):
            parser.error("Invalid or empty runtime plan")
        try:
            validate_plan(plan["cases"], catalog)
        except ValueError as error:
            parser.error(str(error))
        if args.limit is not None and args.limit <= 0:
            parser.error("--limit must be positive")
        runner = Path(get_package_prefix("agribot_vehicle_description")) / "lib/agribot_vehicle_description/verify_algorithm_runtime.py"
        return run_plan(plan["cases"], args.output, runner, args.limit, catalog["validationLimits"],
                        gui=args.gui, rviz=args.rviz)
    elif args.command == "report":
        report = evidence_report(catalog, args.evidence_root, args.output)
        print(json.dumps(report["counts"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
