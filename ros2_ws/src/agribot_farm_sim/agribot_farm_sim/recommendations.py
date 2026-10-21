"""Export exact, current full-route evidence for a future recommendation service."""

import argparse
import json
from pathlib import Path
import shlex
import time

import yaml

from .compatibility import binding_id, collect_evidence, digest, read_catalog, recommend_algorithm
from .scenarios import SCENARIO_IDS


def evidence_details(selection, binding, description):
    """Read recorded inputs, not a newly regenerated route or guessed GUI settings."""
    config_path = Path(description) / "generated" / binding["preset"] / "vehicle_config.json"
    for reference in selection["evidence"]:
        source = Path(reference["source"])
        summary = json.loads(source.read_text())
        result = next((item for item in summary["results"] if item["id"] == selection["algorithm"]
                       and item.get("status") == "PASS" and item.get("full_route") == "PASS"
                       and not item.get("abnormal_process_exits")), None)
        attempt_path = source.parent.parent / (source.parent.name + ".attempt.json")
        if result is None or not attempt_path.is_file():
            continue
        attempt = json.loads(attempt_path.read_text())
        case = attempt.get("case", {})
        if (attempt.get("returncode") != 0 or attempt.get("qualityErrors")
                or any(case.get(key) != selection[key] for key in ("binding", "vehicleId", "scenario", "algorithm"))):
            continue
        hashes = summary["file_sha256"]
        configs = [Path(name) for name in hashes if Path(name).name == "vehicle_config.json"]
        routes = [Path(name) for name in hashes
                  if Path(name).suffix in (".yaml", ".yml") and "waypoint" in Path(name).stem]
        if (len(configs) != 1 or configs[0].resolve() != config_path.resolve()
                or digest(config_path) != hashes[str(configs[0])] or len(routes) != 1):
            continue
        values = yaml.safe_load(routes[0].read_text())["waypoints"]
        if len(values) != result.get("checks", {}).get("driven_waypoints"):
            continue
        domain = result.get("ros_domain_id")
        rmw = summary.get("host", {}).get("rmw_implementation")
        if type(domain) is not int or not isinstance(rmw, str) or not rmw:
            continue
        environment = {"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": rmw}
        dds = [name for name in hashes if Path(name).name == "cyclonedds_simulation.xml"]
        if rmw == "rmw_cyclonedds_cpp" and len(dds) == 1:
            environment["CYCLONEDDS_URI"] = "file://" + dds[0]
        return {
            "config": json.loads(config_path.read_text()), "configPath": str(config_path),
            "configSha256": digest(config_path), "result": result,
            "waypoints": [{"x": point["x"], "y": point["y"], "yawRad": point["yaw"]} for point in values],
            "waypointFile": str(routes[0]), "waypointSha256": hashes[str(routes[0])],
            "validationCommand": attempt["command"], "validationAttempt": str(attempt_path),
            "environment": environment,
            "runtimePresentation": {"gazeboGui": "--gui" in attempt["command"], "rviz": "--rviz" in attempt["command"]},
        }
    raise ValueError("No matching successful attempt with recorded command, configuration and complete driven route")


def build_catalog(catalog, roots, description, scenes, acceptance_policy=None):
    records = collect_evidence(catalog, roots, acceptance_policy)
    cars = {car["car"]: car for car in catalog["vehicles"]}
    scene_names = {scene["id"]: scene["name"] for scene in scenes["scenarios"]}
    verified, selections = [], []
    for binding in catalog["bindings"]:
        for scene in SCENARIO_IDS:
            choice = recommend_algorithm(catalog, binding, scene, records)
            item = {**choice, "vehicleName": cars[binding["car"]]["name"],
                    "implementId": binding["implement"], "implementName": binding["name"],
                    "sceneName": scene_names[scene]}
            item["failures"] = [{key: record[key] for key in (
                "algorithm", "source", "error", "qualityErrors", "horizontalRmseM", "horizontalMaxErrorM")}
                for record in records if record["binding"] == binding_id(binding) and record["scenario"] == scene
                and record["currentInputsMatch"] and record["scope"] == "full-route"
                and (not record["fullRoute"] or record["qualityStatus"] != "PASS")]
            item["verificationRechecks"] = [record["verificationRecheck"] for record in records
                if record["binding"] == binding_id(binding) and record["scenario"] == scene and record.get("verificationRecheck")]
            selections.append(item)
            if choice["status"] != "VALIDATED":
                continue
            try:
                details = evidence_details(choice, binding, description)
            except ValueError as error:
                item.update(exportStatus="EVIDENCE_DETAIL_REQUIRED", exportError=str(error))
                continue
            config = details["config"]
            presentation = details["runtimePresentation"]
            launch = {
                "vehicle_id": binding["preset"], "scenario": scene,
                "manipulator": str(binding["implement"] == "lite6").lower(),
                "arm_demo": "false", "run_waypoints": "true",
                "rmw_implementation": details["environment"]["RMW_IMPLEMENTATION"],
                "gui": str(presentation["gazeboGui"]).lower(), "rviz": str(presentation["rviz"]).lower(),
                **{key + "_mode": value for key, value in choice["components"].items()},
            }
            command = ["env"] + [key + "=" + value for key, value in details["environment"].items()]
            command += ["ros2", "launch", "agribot_vehicle_description", "configured_ackermann_sim.launch.py"]
            command += [key + ":=" + value for key, value in launch.items()]
            result = details["result"]
            verified.append({
                **item, "scope": "exact_simulation_route", "vehicleType": config["vehicleType"],
                "verifiedOperations": ["transport_navigation"], "mountCapability": binding["capability"],
                "vehicleConfig": details["configPath"], "vehicleConfigSha256": details["configSha256"],
                "parameterDirectory": str(Path(details["configPath"]).parent / "config"),
                "bodyExtentsM": config["geometry"]["bodyExtents"],
                "minTurningRadiusM": config["geometry"]["minTurningRadiusM"],
                "maxLinearSpeedMps": config["navigation"]["maxLinearSpeedMps"],
                "finalPositionToleranceM": config["geometry"]["wheelbaseM"] / 2,
                "waypoints": details["waypoints"], "waypointFile": details["waypointFile"],
                "waypointSha256": details["waypointSha256"], "runtimePresentation": presentation,
                "environment": details["environment"],
                "launchArguments": launch, "simulationCommand": shlex.join(command),
                "validationCommand": details["validationCommand"], "validationAttempt": details["validationAttempt"],
                "metrics": {"horizontalRmseM": result["horizontal_pose_rmse_m"],
                            "horizontalMaxErrorM": result["horizontal_pose_max_error_m"],
                            "waypointCount": result["checks"]["driven_waypoints"],
                            "realTimeFactor": result.get("real_time_factor")},
            })
    return {
        "schemaVersion": 1, "scope": "simulation_only", "generatedAt": time.strftime("%FT%T%z"),
        "recommendations": verified, "allSelections": selections,
        "counts": {"verifiedCombinations": len(verified), "allCombinations": len(selections),
                   "reviewRequired": sum(item["status"] == "REVIEW_REQUIRED" for item in selections),
                   "pending": sum(item["status"] == "PENDING_VALIDATION" for item in selections),
                   "missingDetails": sum(item.get("exportStatus") == "EVIDENCE_DETAIL_REQUIRED" for item in selections)},
        "evidenceRoots": [str(Path(root).resolve()) for root in roots],
        "acceptanceMigrationPolicy": str(Path(acceptance_policy).resolve()) if acceptance_policy else None,
        "recommendationRules": [
            "只能从 recommendations 中选取精确车型、机具、场景和算法；不得把候选配方或未测试组合说成已通过。",
            "小型、紧凑型首先验证 FAST-LIVO2/RTK；大型车可按场景使用 NavSat。两种定位分别验收。",
            "只验证运输导航；挂载名称及场景名称不代表耕作、播种、喷洒、割草、采摘或收割过程已实现。",
            "只适用记录的完整路线、米制车体、传感器与参数；不得外推到所有路径、真实作业或其他车型。",
            "农机具不参与导航碰撞，本清单不提供机具防撞安全保证。",
            "simulationCommand 采用实际验收的显示开关；未开启 RViz 的验收不能作为 RViz 运行或退出稳定性证据。",
        ],
        "limitations": catalog["limitations"],
    }


def write_catalog(data, destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    (destination / "recommendations.json").write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    lines = ["# 农机、机具与场景可运行组合", "", "仅列当前输入下通过完整路线、定位质量及进程退出检查的组合。", "",
             f"已验收 {data['counts']['verifiedCombinations']} 项；全部 {data['counts']['allCombinations']} 项的状态见 recommendations.json。", "",
             "运行命令前须加载 ROS Humble 与本工作区 install/setup.bash；停止已有仿真再启动，避免同域进程重叠。", "",
             "| 农机 | 机具 | 场景 | 定位 | 规划 / 控制 | 途经点数 | 水平 RMSE / 最大误差 (m) |", "| --- | --- | --- | --- | --- | ---: | --- |"]
    for item in data["recommendations"]:
        components, metrics = item["components"], item["metrics"]
        lines.append(f"| {item['binding'].split('__')[0]} {item['vehicleName']} | {item['implementName']} | {item['sceneName']} | "
                     f"{components['localization']} | {components['planner']} / {components['controller']} | {metrics['waypointCount']} | "
                     f"{metrics['horizontalRmseM']:.4f} / {metrics['horizontalMaxErrorM']:.4f} |")
    lines += ["", "## 使用边界", ""] + ["- " + rule for rule in data["recommendationRules"]]
    lines += ["", "## 尚未通过的实测组合", ""]
    for item in data["allSelections"]:
        for failure in item["failures"]:
            reason = failure["error"] or "；".join(failure["qualityErrors"]) or "全路线未完成"
            lines.append(f"- {item['binding']} / {item['sceneName']} / {failure['algorithm']}：{reason}；[原始记录]({failure['source']})")
    lines += ["", "## 已验证路线", ""]
    for item in data["recommendations"]:
        label = item["binding"] + " / " + item["scenario"]
        lines += ["### " + label, "", "```bash", item["simulationCommand"], "```", "",
                  "途经点 (x, y, yawRad)：`" + json.dumps(item["waypoints"]) + "`", "",
                  "验收记录：[attempt](" + item["validationAttempt"] + ")", ""]
    (destination / "recommendations.md").write_text("\n".join(lines) + "\n")


def evidence_signature(roots, acceptance_policy=None):
    paths = {Path(__file__), Path(read_catalog.__code__.co_filename)}
    if acceptance_policy:
        policy = json.loads(Path(acceptance_policy).read_text())
        paths.update((Path(acceptance_policy), Path(policy["legacySourceArchive"]), Path(policy["verifierPath"])))
    for root in roots:
        for source in root.rglob("summary.json"):
            paths.add(source)
            paths.update(Path(name) for name in json.loads(source.read_text()).get("file_sha256", {}))
            paths.add(source.parent.parent / (source.parent.name + ".attempt.json"))
    signature = []
    for path in sorted(paths):
        try:
            stat = path.stat()
            signature.append((str(path), stat.st_mtime_ns, stat.st_size))
        except FileNotFoundError:
            signature.append((str(path), None, None))
    return signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watch", type=float, default=0, help="Poll evidence every N seconds; zero exports once")
    parser.add_argument("--acceptance-policy", type=Path, help="Explicit read-only verification-only migration proof")
    args = parser.parse_args()
    if args.watch < 0 or args.watch != args.watch or args.watch == float("inf"):
        parser.error("--watch must be finite and nonnegative")
    from ament_index_python.packages import get_package_share_directory
    description = Path(get_package_share_directory("agribot_vehicle_description"))
    scenes = json.loads((Path(get_package_share_directory("agribot_farm_sim")) / "config/scenarios.json").read_text())
    catalog = read_catalog()
    previous = None
    while True:
        try:
            signature = evidence_signature(args.evidence_root, args.acceptance_policy)
            if signature != previous:
                data = build_catalog(catalog, args.evidence_root, description, scenes, args.acceptance_policy)
                destination = args.output if not args.watch else args.output / str(time.time_ns())
                write_catalog(data, destination)
                if args.watch:
                    link = args.output / "latest.next"
                    link.unlink(missing_ok=True)
                    link.symlink_to(destination.resolve(), target_is_directory=True)
                    link.replace(args.output / "latest")
                previous = signature
                print(json.dumps(data["counts"]) + " " + str(destination), flush=True)
        except json.JSONDecodeError:
            if not args.watch:
                raise
        if not args.watch:
            return 0
        time.sleep(args.watch)


if __name__ == "__main__":
    raise SystemExit(main())
