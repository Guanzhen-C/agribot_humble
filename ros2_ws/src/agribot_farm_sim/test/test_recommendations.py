import json
from pathlib import Path

import pytest
import yaml

from agribot_farm_sim.compatibility import digest, read_catalog
from agribot_farm_sim.recommendations import build_catalog, evidence_signature, write_catalog


SHARE = Path(__file__).resolve().parents[1]


@pytest.fixture
def evidence(tmp_path):
    catalog = read_catalog(SHARE / "config/compatibility.json")
    scenes = json.loads((SHARE / "config/scenarios.json").read_text())
    description = tmp_path / "description"
    config = description / "generated/mower_front_cutter/vehicle_config.json"
    config.parent.mkdir(parents=True)
    config.write_text(json.dumps({"vehicleType": "differential",
        "geometry": {"wheelbaseM": 1.2, "minTurningRadiusM": .75,
                     "bodyExtents": {"frontM": 1.4, "rearM": .4, "leftM": .8, "rightM": .8}},
        "navigation": {"maxLinearSpeedMps": .3}}))
    waypoints = tmp_path / "mower_front_cutter_waypoints.yaml"
    waypoints.write_text(yaml.safe_dump({"waypoints": [dict(x=x, y=0, yaw=0) for x in (-3, 1, 5)]}))
    case = {"binding": "Car8__front_sickle_bar", "vehicleId": "mower_front_cutter",
            "scenario": "tomato_greenhouse", "algorithm": catalog["defaultAlgorithm"]}
    root = tmp_path / "runs"
    folder = root / "0001/Car8_front_cutter_case"
    folder.mkdir(parents=True)
    summary = folder / "summary.json"
    summary.write_text(json.dumps({"vehicle_id": case["vehicleId"], "scenario": case["scenario"],
        "manipulator": "false", "file_sha256": {str(p): digest(p) for p in (config, waypoints)},
        "host": {"rmw_implementation": "rmw_cyclonedds_cpp"},
        "results": [{"id": case["algorithm"], "status": "PASS", "scope": "full-route", "full_route": "PASS",
                     "ros_domain_id": 73,
                     "horizontal_pose_rmse_m": .015, "horizontal_pose_max_error_m": .04,
                     "checks": {"driven_waypoints": 3}}]}))
    attempt = folder.parent / (folder.name + ".attempt.json")
    attempt.write_text(json.dumps({"case": case, "returncode": 0, "qualityErrors": [],
                                  "command": ["/usr/bin/python3", "verify.py", "--gui"]}))
    return catalog, scenes, description, root, config, waypoints, summary, attempt


def export(evidence):
    catalog, scenes, description, root, *_ = evidence
    return build_catalog(catalog, [root], description, scenes)


def test_export_uses_recorded_complete_waypoints_and_actual_display_flags(evidence):
    result = export(evidence)
    assert result["counts"]["verifiedCombinations"] == 1
    assert result["counts"]["allCombinations"] == 549
    item = result["recommendations"][0]
    assert item["vehicleName"] == "骑乘式割草机" and item["sceneName"] == "番茄温室"
    assert item["components"]["localization"] == "fastlivo_rtk"
    assert [point["x"] for point in item["waypoints"]] == [-3, 1, 5]
    assert item["runtimePresentation"] == {"gazeboGui": True, "rviz": False}
    assert item["launchArguments"]["rviz"] == "false" and "rviz:=false" in item["simulationCommand"]
    assert item["finalPositionToleranceM"] == .6
    assert item["verifiedOperations"] == ["transport_navigation"]
    assert item["environment"]["ROS_DOMAIN_ID"] == "73"
    assert item["simulationCommand"].startswith("env ROS_DOMAIN_ID=73 ")


@pytest.mark.parametrize("change", ["failed", "bad_quality", "changed_inputs", "abnormal_exit"])
def test_export_never_recommends_failed_quality_stale_or_abnormal_evidence(evidence, change):
    *_, config, _, summary, _ = evidence
    data = json.loads(summary.read_text())
    result = data["results"][0]
    if change == "failed":
        result["status"] = "FAIL"
        result["error"] = "Vehicle moved before planning/control dispatch"
    elif change == "bad_quality":
        result["horizontal_pose_rmse_m"] = 1.2
    elif change == "abnormal_exit":
        result["abnormal_process_exits"] = [{"process": "rviz", "code": -11}]
    else:
        config.write_text(config.read_text() + "\n")
    summary.write_text(json.dumps(data))
    exported = export(evidence)
    assert not exported["recommendations"]
    choice = next(item for item in exported["allSelections"]
                  if item["binding"] == "Car8__front_sickle_bar" and item["scenario"] == "tomato_greenhouse")
    assert bool(choice["failures"]) == (change != "changed_inputs")
    if change == "failed":
        assert choice["failures"][0]["error"] == result["error"]


@pytest.mark.parametrize("change", ["missing", "wrong_binding", "nonzero_exit", "missing_waypoint"])
def test_export_requires_exact_successful_attempt_and_driven_waypoints(evidence, change):
    *_, summary, attempt = evidence
    if change == "missing":
        attempt.unlink()
    elif change == "missing_waypoint":
        data = json.loads(summary.read_text())
        data["results"][0]["checks"]["driven_waypoints"] = 2
        summary.write_text(json.dumps(data))
    else:
        data = json.loads(attempt.read_text())
        if change == "wrong_binding":
            data["case"]["binding"] = "Car0__none"
        else:
            data["returncode"] = 1
        attempt.write_text(json.dumps(data))
    result = export(evidence)
    assert not result["recommendations"] and result["counts"]["missingDetails"] == 1


def test_watcher_notices_changed_and_missing_inputs_without_summary_change(evidence):
    _, _, _, root, config, *_ = evidence
    first = evidence_signature([root])
    config.write_text(config.read_text() + "\n")
    second = evidence_signature([root])
    assert first != second
    config.unlink()
    assert second != evidence_signature([root])


def test_empty_evidence_is_pending_not_a_recommendation(evidence):
    *_, summary, _ = evidence
    data = json.loads(summary.read_text())
    data["results"] = []
    summary.write_text(json.dumps(data))
    result = export(evidence)
    assert result["counts"]["pending"] == 549 and not result["recommendations"]


def test_exports_are_immutable_and_contain_human_and_machine_readable_results(evidence, tmp_path):
    result = export(evidence)
    output = tmp_path / "export"
    write_catalog(result, output)
    assert json.loads((output / "recommendations.json").read_text())["counts"] == result["counts"]
    assert "番茄温室" in (output / "recommendations.md").read_text()
    with pytest.raises(FileExistsError):
        write_catalog(result, output)
