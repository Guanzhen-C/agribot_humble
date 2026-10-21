import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from agribot_farm_sim.compatibility import (
    binding_id, component_cases, configuration, digest, evidence_record,
    pose_occupancy, scenario_review, read_catalog, runtime_plan, route_plan, evidence_report, installation_review, validate_plan, run_plan,
    select_algorithm, collect_evidence, recommend_algorithm,
    route_runtime_budget,
)


CATALOG = Path(__file__).resolve().parents[1] / "config/compatibility.json"


def test_exact_garage_and_mounting_inventory():
    catalog = read_catalog(CATALOG)
    assert len(catalog["bindings"]) == 61
    assert len({binding_id(b) for b in catalog["bindings"]}) == 61
    actual = {v["car"]: {b["implement"] for b in catalog["bindings"] if b["car"] == v["car"]}
              for v in catalog["vehicles"]}
    assert actual["Car0"] == actual["Car3"] == {"none", "lite6"}
    rear = {"compact_chisel_plow", "mounted_disc_harrow", "groove_maker", "mounted_cultivator",
            "forest_chain_crusher", "inter_row_milling_cutter", "garlic_planter", "mulch_planter"}
    assert actual["Car6"] == {"none"} | (rear - {"forest_chain_crusher", "inter_row_milling_cutter"})
    assert actual["Car7"] == {"none", "farm_trailer"} | rear
    assert actual["Car8"] == {"none", "front_sickle_bar"}
    assert actual["Car9"] == {"none", "combine_header"}
    assert actual["Car1"] == actual["Car2"] == actual["Car4"] == actual["Car5"] == {"none"} | rear
    assert all(actual[car] - {"none"} for car in actual)
    assert all(b["capability"] != "harvesting" for b in catalog["bindings"])


def test_catalog_matches_actual_unity_farm_presets():
    source = Path.home() / "unity_projects/AgriculturalCar_Interior/Assets/AgribotConfig/Resources/farm_vehicle_presets.json"
    if not source.is_file():
        pytest.skip("Unity checkout is not present on this machine")
    presets = json.loads(source.read_text())["presets"]
    declared = {(b["preset"], b["implement"]) for b in read_catalog(CATALOG)["bindings"] if b["car"] not in ("Car0", "Car3")}
    assert declared == {(p["id"], p["implementId"]) for p in presets}


def test_component_product_is_the_existing_480_options():
    catalog = read_catalog(CATALOG)
    cases = component_cases(catalog)
    assert len(cases) == len(set(cases)) == 480
    assert catalog["defaultAlgorithm"] == "_".join(cases[0])


def test_registered_rear_mounts_match_the_design_matrix_and_reviews_are_retained():
    matrix = json.loads((CATALOG.parent / "rear_mount_matrix.json").read_text())
    reviews = {(r["car"], r["implement"]) for r in matrix["designReviews"]}
    assert reviews == {("Car6", "forest_chain_crusher"), ("Car6", "inter_row_milling_cutter")}
    assert all(r["reason"] for r in matrix["designReviews"])
    expected = {(h["car"], t["id"]) for h in matrix["hosts"] for t in matrix["implements"]} - reviews
    rear = {t["id"] for t in matrix["implements"]}
    actual = {(b["car"], b["implement"]) for b in read_catalog(CATALOG)["bindings"] if b["implement"] in rear}
    assert actual == expected
    assert len(actual) == 46


def test_json_key_order_does_not_change_algorithm_ids_or_plan():
    catalog = read_catalog(CATALOG)
    reordered = copy.deepcopy(catalog)
    reordered["components"] = dict(reversed(list(reordered["components"].items())))
    assert component_cases(reordered) == component_cases(catalog)
    assert runtime_plan(reordered, "baseline") == runtime_plan(catalog, "baseline")


@pytest.mark.parametrize("suite", ["mountings", "components", "scenes", "baseline", "matrix"])
def test_plans_have_explicit_arm_states_and_no_duplicate_cases(suite):
    plan = runtime_plan(read_catalog(CATALOG), suite)
    assert plan and len({p["id"] for p in plan}) == len(plan)
    assert all(p["scope"] == "smoke" for p in plan)
    for case in plan:
        assert case["manipulator"] == ("true" if case["binding"].endswith("__lite6") else "false")
        assert case["armDemo"] == "false"
    if suite == "mountings":
        assert len(plan) == 61
    if suite == "components":
        assert len(plan) == 28
    if suite == "scenes":
        assert len(plan) == 9
    if suite == "baseline":
        assert len(plan) == 95


def test_footprint_checks_unknown_collision_edges_and_rotated_origin():
    metadata = dict(origin=[0., 0., 0.], resolution=.1, negate=0,
                    occupied_thresh=.65, free_thresh=.196)
    image = Image.new("L", (100, 100), 254)
    polygon = [[-.2, -.2], [.2, -.2], [.2, .2], [-.2, .2]]
    assert pose_occupancy(image, metadata, polygon, [5., 5., 0.]) == "free"
    image.putpixel((50, 49), 0)
    assert pose_occupancy(image, metadata, polygon, [5., 5., 0.]) == "collision"
    image.putpixel((50, 49), 205)
    assert pose_occupancy(image, metadata, polygon, [5., 5., 0.]) == "unknown"
    assert pose_occupancy(image, metadata, polygon, [0., 0., 0.]) == "outside"
    metadata["origin"] = [0., 0., math.pi / 2]
    assert pose_occupancy(image, metadata, polygon, [-5., 5., 0.]) == "unknown"


def test_scenario_review_keeps_unknown_distinct_from_free_or_collision(tmp_path):
    image = Image.new("L", (100, 100), 205)
    image.save(tmp_path / "map.pgm")
    path = tmp_path / "map.yaml"
    path.write_text("image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    polygon = [[-.2, -.2], [.2, -.2], [.2, .2], [-.2, .2]]
    review = scenario_review({}, polygon, "orchard", {"spawn": [5, 5, 0]}, path, [[6, 5, 0]])
    assert review["status"] == "UNKNOWN_SPACE_ONLY" and len(review["conflicts"]) == 2
    assert review["routeDistanceLowerBoundM"] == 1.
    assert all(item["status"] == "unknown" for item in review["conflicts"])
    image.putpixel((50, 49), 0)
    image.save(tmp_path / "map.pgm")
    review = scenario_review({}, polygon, "orchard", {"spawn": [5, 5, 0]}, path, [[6, 5, 0]])
    assert review["status"] == "MISSION_REVIEW_REQUIRED"


def test_runtime_budget_accounts_for_route_length_without_changing_short_route_defaults():
    assert route_runtime_budget(12., .3)["motionTimeoutS"] == 2400
    budget = route_runtime_budget(478.162, .3)
    assert budget["motionTimeoutS"] == 4165
    assert budget["distanceLowerBoundM"] == 478.162
    assert budget["maxSpeedMps"] == .3


@pytest.mark.parametrize("distance,speed", [(True,.3),(-1,.3),(float('nan'),.3),
    (10,True),(10,0),(10,-.3),(10,float('inf')),(10000,.1)])
def test_runtime_budget_rejects_invalid_or_unreviewed_values(distance,speed):
    with pytest.raises(ValueError):
        route_runtime_budget(distance,speed)


def test_only_current_complete_route_evidence_is_eligible(tmp_path):
    source = tmp_path / "config.json"
    source.write_text("first")
    summary = {"vehicle_id": "test", "scenario": "tomato_greenhouse", "file_sha256": {str(source): digest(source)}}
    result = {"status": "PASS", "scope": "smoke", "full_route": "NOT_RUN"}
    assert not evidence_record(summary, result)["eligibleForSimulationRecommendation"]
    result.update(scope="full-route", full_route="PASS")
    assert evidence_record(summary, result)["eligibleForSimulationRecommendation"]
    source.write_text("new geometry")
    record = evidence_record(summary, result)
    assert not record["currentInputsMatch"] and not record["eligibleForSimulationRecommendation"]
    result["status"] = "FAIL"
    assert not evidence_record(summary, result)["fullRoute"]
    summary["file_sha256"] = {}
    assert not evidence_record(summary, result)["currentInputsMatch"]


def test_report_preserves_failure_and_does_not_recommend_direct_baseline(tmp_path):
    source = tmp_path / "config.json"
    source.write_text("unchanged")
    root = tmp_path / "runs"
    root.mkdir()
    catalog = read_catalog(CATALOG)
    summary = {"vehicle_id": "ackermann_current", "scenario": "tomato_greenhouse",
               "manipulator": "false", "file_sha256": {str(source): digest(source)},
               "results": [
                   {"id": catalog["defaultAlgorithm"], "status": "PASS", "scope": "full-route", "full_route": "PASS"},
                   {"id": "stvl_off_fastlivo_rtk_direct_mppi", "status": "PASS", "scope": "full-route", "full_route": "PASS"},
                   {"id": catalog["defaultAlgorithm"], "status": "FAIL", "scope": "smoke", "error": "controller failed"}]}
    for result in summary["results"]:
        result.update(horizontal_pose_rmse_m=.02, horizontal_pose_max_error_m=.1)
    (root / "summary.json").write_text(json.dumps(summary))
    report = evidence_report(catalog, [root, root], tmp_path / "report")
    assert len(report["records"]) == 3
    assert report["counts"]["currentFail"] == 1
    assert report["counts"]["recommendableFullRoutes"] == 1
    assert all(r["binding"] == "Car0__none" for r in report["records"])
    assert "controller failed" in (tmp_path / "report/report.md").read_text()
    summary["results"][0]["horizontal_pose_rmse_m"] = 4.5
    (root / "summary.json").write_text(json.dumps(summary))
    rejected = evidence_report(catalog, [root], tmp_path / "quality_report")
    assert rejected["counts"]["recommendableFullRoutes"] == 0
    assert rejected["counts"]["currentQualityRejected"] == 1
    summary["results"].append(dict(summary["results"][0], horizontal_pose_rmse_m=.01))
    (root / "summary.json").write_text(json.dumps(summary))
    mixed = evidence_report(catalog, [root], tmp_path / "mixed_report")
    assert not mixed["recommendationCandidates"]
    assert any("repeatable" in r.get("recommendationBlock", "") for r in mixed["records"])


@pytest.fixture
def recommendation_runs(tmp_path):
    catalog = read_catalog(CATALOG)
    binding = next(b for b in catalog["bindings"] if b["preset"] == "car5_crusher")
    source = tmp_path / "vehicle_config.json"
    source.write_text("current full-size vehicle and implement")
    root = tmp_path / "runs"
    root.mkdir()

    def collect(results, scene="strawberry_field"):
        summary = {"vehicle_id": binding["preset"], "scenario": scene, "manipulator": "false",
                   "file_sha256": {str(source): digest(source)}, "results": results}
        (root / "summary.json").write_text(json.dumps(summary))
        return collect_evidence(catalog, [root])

    return catalog, binding, source, root, collect


def complete_result(algorithm, **values):
    return {"id": algorithm, "status": "PASS", "scope": "full-route", "full_route": "PASS",
            "horizontal_pose_rmse_m": .02, "horizontal_pose_max_error_m": .08, **values}


def test_recommendation_prefers_physical_default_over_small_metric_gain(recommendation_runs):
    catalog, binding, _, _, collect = recommendation_runs
    records = collect([complete_result(catalog["defaultAlgorithm"]),
                       complete_result("stvl_off_fastlivo_rtk_smac_hybrid_rpp", horizontal_pose_rmse_m=.01)])
    choice = recommend_algorithm(catalog, binding, "strawberry_field", records)
    assert choice["status"] == "VALIDATED"
    assert choice["algorithm"] == catalog["defaultAlgorithm"]
    assert choice["source"] == "physical_default" and len(choice["evidence"]) == 1
    assert choice["conditions"] and choice["testRecipe"]["claim"] == "test_recipe_not_runtime_acceptance"


def test_open_field_preference_requires_exact_successful_evidence(recommendation_runs):
    catalog, _, _, _, collect = recommendation_runs
    binding = next(b for b in catalog["bindings"] if b["preset"] == "car1_crusher")
    navsat = "stvl_off_navsat_smac_hybrid_mppi"
    choice = recommend_algorithm(catalog, binding, "wide_headland", [])
    assert choice["status"] == "PENDING_VALIDATION" and choice["algorithm"] is None
    assert choice["testRecipe"]["algorithm"] == navsat
    records = collect([complete_result(catalog["defaultAlgorithm"], horizontal_pose_rmse_m=.01),
                       complete_result(navsat)], scene="wide_headland")
    records = [dict(r, binding=binding_id(binding)) for r in records]
    choice = recommend_algorithm(catalog, binding, "wide_headland", records)
    assert choice["algorithm"] == navsat and choice["source"] == "scene_default"
    assert "GNSS" in " ".join(choice["conditions"])
    assert recommend_algorithm(catalog, binding, "tomato_greenhouse", records)["algorithm"] is None
    records = collect([complete_result(catalog["defaultAlgorithm"]),
                       complete_result(navsat, status="FAIL", full_route="FAIL")], scene="wide_headland")
    records = [dict(r, binding=binding_id(binding)) for r in records]
    choice = recommend_algorithm(catalog, binding, "wide_headland", records)
    assert choice["algorithm"] is None
    assert choice["status"] == "REVIEW_REQUIRED" and choice["failedAlgorithms"] == [navsat]


def test_exact_controller_override_ranks_before_shared_scene_recipe(recommendation_runs):
    catalog, _, _, _, collect = recommendation_runs
    binding = next(b for b in catalog["bindings"] if b["preset"] == "harvester_header")
    preferred = "stvl_off_navsat_smac_hybrid_rpp"
    records = collect([complete_result(catalog["defaultAlgorithm"], horizontal_pose_rmse_m=.01),
                       complete_result("stvl_off_navsat_smac_hybrid_mppi", horizontal_pose_rmse_m=.01),
                       complete_result(preferred)], scene="wide_headland")
    records = [dict(r, binding=binding_id(binding)) for r in records]
    choice = recommend_algorithm(catalog, binding, "wide_headland", records)
    assert choice["status"] == "VALIDATED" and choice["algorithm"] == preferred
    assert choice["source"] == "scene_binding"


def test_recommendation_only_replaces_failed_module_for_exact_combination(recommendation_runs):
    catalog, binding, _, _, collect = recommendation_runs
    fallback = "stvl_off_fastlivo_rtk_smac_hybrid_rpp"
    records = collect([complete_result(catalog["defaultAlgorithm"], status="FAIL", full_route="FAIL"),
                       complete_result(fallback)])
    choice = recommend_algorithm(catalog, binding, "strawberry_field", records)
    assert choice["status"] == "VALIDATED" and choice["algorithm"] == fallback
    assert choice["source"] == "tested_alternative" and choice["conditions"]
    assert choice["failedAlgorithms"] == [catalog["defaultAlgorithm"]]
    assert recommend_algorithm(catalog, binding, "tomato_greenhouse", records)["algorithm"] is None
    another = next(b for b in catalog["bindings"] if b["preset"] == "car4_crusher")
    assert recommend_algorithm(catalog, another, "strawberry_field", records)["algorithm"] is None


@pytest.mark.parametrize("change", [
    {"status": "FAIL", "full_route": "FAIL"}, {"horizontal_pose_rmse_m": .31},
    {"horizontal_pose_max_error_m": .76}, {"abnormal_process_exits": ["rviz crashed"]}])
def test_failed_or_inconsistent_algorithm_is_not_recommended(recommendation_runs, change):
    catalog, binding, _, _, collect = recommendation_runs
    algorithm = catalog["defaultAlgorithm"]
    records = collect([complete_result(algorithm), complete_result(algorithm, **change)])
    choice = recommend_algorithm(catalog, binding, "strawberry_field", records)
    assert choice["status"] == "REVIEW_REQUIRED" and choice["algorithm"] is None
    assert not choice["evidence"]


def test_short_stale_or_unqualified_evidence_is_not_a_recommendation(recommendation_runs):
    catalog, binding, source, root, collect = recommendation_runs
    for result in (complete_result(catalog["defaultAlgorithm"], scope="smoke", full_route="NOT_RUN"),
                   complete_result("stvl_off_fastlivo_rtk_direct_mppi")):
        choice = recommend_algorithm(catalog, binding, "strawberry_field", collect([result]))
        assert choice["status"] == "PENDING_VALIDATION" and choice["algorithm"] is None
    collect([complete_result(catalog["defaultAlgorithm"])])
    source.write_text("changed implement geometry")
    records = collect_evidence(catalog, [root])
    choice = recommend_algorithm(catalog, binding, "strawberry_field", records)
    assert choice["status"] == "PENDING_VALIDATION" and choice["historicalRecords"] == 1
    assert choice["algorithm"] is None


def test_recommendation_report_lists_all_combinations_without_inventing_passes(recommendation_runs, tmp_path):
    catalog, binding, _, root, collect = recommendation_runs
    collect([complete_result(catalog["defaultAlgorithm"])])
    report = evidence_report(catalog, [root], tmp_path / "report")
    assert len(report["algorithmSelections"]) == len(catalog["bindings"]) * 9
    assert report["counts"]["validatedSelections"] == 1
    assert report["counts"]["reviewRequiredSelections"] == 0
    assert sum(s["algorithm"] is not None for s in report["algorithmSelections"]) == 1


def test_missing_evidence_directory_is_not_silently_treated_as_no_tests(tmp_path):
    with pytest.raises(ValueError, match="Runtime evidence root does not exist"):
        collect_evidence(read_catalog(CATALOG), [tmp_path / "misspelled"])


def test_full_route_plan_does_not_hide_excluded_mountings():
    catalog = read_catalog(CATALOG)
    audit = {"bindings": [{"id": "Car0__none", "status": "PASS", "scenes": {
        "tomato_greenhouse": {"status": "POSES_CLEAR", "policy": "candidate"}}}]}
    cases, excluded = route_plan(catalog, audit, "tomato_greenhouse")
    assert len(cases) == 1 and len(excluded) == 60
    assert cases[0]["scope"] == "full-route" and cases[0]["manipulator"] == "false"
    audit["bindings"][0]["scenes"]["tomato_greenhouse"]["policy"] = "not_recommended"
    assert not route_plan(catalog, audit, "tomato_greenhouse")[0]


def test_route_plan_carries_the_audited_observation_budget_without_changing_the_recipe():
    catalog = read_catalog(CATALOG)
    record = {"id": "Car0__none", "status": "PASS", "maxSpeedMps": .3,
              "scenes": {"orchard": {"status": "POSES_CLEAR", "policy": "candidate",
                                     "routeDistanceLowerBoundM": 478.162}}}
    cases, _ = route_plan(catalog, {"bindings": [record]}, "orchard")
    assert cases[0]["motionTimeoutS"] == 4165
    assert cases[0]["runtimeBudget"] == route_runtime_budget(478.162,.3)
    assert cases[0]["algorithm"] == catalog["defaultAlgorithm"]
    validate_plan(cases,catalog)


@pytest.mark.parametrize("timeout", [True,2399,14401,2400.,"4165",None])
def test_plan_validation_rejects_invalid_full_route_observation_budget(timeout):
    catalog = read_catalog(CATALOG)
    case = runtime_plan(catalog,"mountings")[0]
    case.update(scope="full-route",motionTimeoutS=timeout)
    with pytest.raises(ValueError,match="observation budget"):
        validate_plan([case],catalog)


def test_smoke_plan_cannot_override_full_route_observation_budget():
    catalog = read_catalog(CATALOG)
    case = runtime_plan(catalog,"mountings")[0]
    case["motionTimeoutS"] = 4165
    with pytest.raises(ValueError,match="observation budget"):
        validate_plan([case],catalog)


@pytest.mark.parametrize("vehicle, scenario, localization", [
    ("Car0", "orchard", "fastlio"),
    ("Car0", "wide_headland", "navsat"),
    ("Car1", "wide_headland", "navsat"),
])
@pytest.mark.parametrize("policy", [True, False, None, "true"])
def test_unknown_only_route_requires_the_selected_actual_planner_policy(vehicle, scenario, localization, policy):
    catalog = read_catalog(CATALOG)
    profile = localization + "/stvl/mppi/smac_hybrid"
    review = {"status": "UNKNOWN_SPACE_ONLY", "policy": "candidate",
              "conflicts": [{"pose": "waypoint_23", "status": "unknown"}]}
    record = {"id": vehicle + "__none", "status": "PASS", "scenes": {scenario: review},
              "unknownTraversalPolicies": {profile: policy}}
    cases, excluded = route_plan(catalog, {"bindings": [record]}, scenario)
    if policy is True:
        assert len(cases) == 1
        warning = cases[0]["routeWarnings"]
        assert warning["status"] == "UNKNOWN_SPACE_ONLY" and warning["plannerProfile"] == profile
        assert warning["poses"] == review["conflicts"]
        warning["poses"].clear()
        assert review["conflicts"]
        # Another profile's permission is not enough for this selection.
        record["unknownTraversalPolicies"] = {localization + "/stvl/rpp/smac_hybrid": True}
        assert not route_plan(catalog, {"bindings": [record]}, scenario)[0]
    else:
        assert not cases and any(item["binding"] == vehicle + "__none" for item in excluded)


@pytest.mark.parametrize("status", ["collision", "outside"])
def test_allow_unknown_never_bypasses_known_collision_or_outside_map(status):
    catalog = read_catalog(CATALOG)
    record = {"id": "Car0__none", "status": "PASS",
              "unknownTraversalPolicies": {"fastlio/stvl/mppi/smac_hybrid": True},
              "scenes": {"orchard": {"status": "UNKNOWN_SPACE_ONLY", "policy": "candidate",
                         "conflicts": [{"pose": "waypoint_23", "status": status}]}}}
    assert not route_plan(catalog, {"bindings": [record]}, "orchard")[0]


def test_mount_only_review_cannot_be_promoted_by_a_synthetic_success(tmp_path):
    catalog = read_catalog(CATALOG)
    binding = next(b for b in catalog["bindings"] if b["preset"] == "harvester_header")
    binding.update(navigationRecommendationAllowed=False, reviewReason="Explicit mounting-only test policy")
    source = tmp_path / "config.json"
    source.write_text("unchanged")
    root = tmp_path / "run"
    root.mkdir()
    (root / "summary.json").write_text(json.dumps({
        "vehicle_id": "harvester_header", "scenario": "farmyard", "manipulator": "false",
        "file_sha256": {str(source): digest(source)}, "results": [{
            "status": "PASS", "scope": "full-route", "full_route": "PASS", "id": catalog["defaultAlgorithm"],
            "horizontal_pose_rmse_m": .01, "horizontal_pose_max_error_m": .02}]}))
    report = evidence_report(catalog, [root], tmp_path / "report")
    assert not report["recommendationCandidates"]
    assert report["records"][0]["policyReviewReason"]
    static_audit = {"bindings": [{"id": "Car9__combine_header", "status": "PASS", "scenes": {
        "farmyard": {"status": "POSES_CLEAR", "policy": "candidate"}}}]}
    cases, excluded = route_plan(catalog, static_audit, "farmyard")
    assert not cases
    assert any(item["binding"] == "Car9__combine_header" and item["reason"] for item in excluded)


def test_all_scene_route_review_does_not_exclude_a_vehicle_by_size_group_alone():
    catalog = read_catalog(CATALOG)
    audit = {"bindings": [{"id": "Car1__mounted_cultivator", "status": "PASS", "scenes": {
        "vineyard": {"status": "POSES_CLEAR", "policy": "not_recommended"}}}]}
    assert not route_plan(catalog, audit, "vineyard")[0]
    cases, excluded = route_plan(catalog, audit, "vineyard", use_scene_hints=False)
    assert len(cases) == 1
    assert cases[0]["binding"] == "Car1__mounted_cultivator"
    assert cases[0]["scope"] == "full-route"
    review = audit["bindings"][0]["scenes"]["vineyard"]
    review.update(status="MISSION_REVIEW_REQUIRED", conflicts=[dict(pose="waypoint_2", status="collision")])
    cases, excluded = route_plan(catalog, audit, "vineyard", use_scene_hints=False)
    assert not cases
    item = next(item for item in excluded if item["binding"] == "Car1__mounted_cultivator")
    assert item["scenario"] == "vineyard" and item["conflicts"] == review["conflicts"]
    assert "not proof" in item["reason"]


def test_navigation_policy_does_not_accept_a_truthy_string(tmp_path):
    catalog = read_catalog(CATALOG)
    catalog["bindings"][0]["navigationRecommendationAllowed"] = "false"
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(catalog))
    with pytest.raises(ValueError, match="must be boolean"):
        read_catalog(path)


def test_exact_full_route_success_is_not_rejected_by_a_vehicle_size_scene_hint(tmp_path):
    catalog = read_catalog(CATALOG)
    source = tmp_path / "current.json"
    source.write_text("current model")
    root = tmp_path / "run"
    root.mkdir()
    summary = {"vehicle_id": "car1_cultivator", "scenario": "vineyard", "manipulator": "false",
               "file_sha256": {str(source): digest(source)}, "results": [{
                   "id": catalog["defaultAlgorithm"], "status": "PASS", "scope": "full-route", "full_route": "PASS",
                   "horizontal_pose_rmse_m": .02, "horizontal_pose_max_error_m": .05}]}
    (root / "summary.json").write_text(json.dumps(summary))
    report = evidence_report(catalog, [root], tmp_path / "report")
    assert report["records"][0]["sceneHint"] is False
    assert report["records"][0]["eligibleForSimulationRecommendation"] is True


def test_route_and_mount_plans_select_by_vehicle_implement_and_scene():
    catalog = read_catalog(CATALOG)
    for preset, scenario, algorithm in (
        ("car2_cultivator", "arable_field", "stvl_off_navsat_smac_hybrid_mppi"),
        ("car4_chisel", "tomato_greenhouse", catalog["defaultAlgorithm"]),
        ("car5_harrow", "tomato_greenhouse", catalog["defaultAlgorithm"]),
        ("car4_harrow", "wide_headland", "stvl_off_navsat_smac_hybrid_mppi"),
        ("cab_groove", "wide_headland", "stvl_off_navsat_smac_hybrid_mppi"),
        ("harvester_header", "wide_headland", "stvl_off_navsat_smac_hybrid_rpp"),
    ):
        binding = next(b for b in catalog["bindings"] if b["preset"] == preset)
        key = binding_id(binding)
        audit = {"bindings": [{"id": key, "status": "PASS", "scenes": {
            scenario: {"status": "POSES_CLEAR", "policy": "candidate"}}}]}
        cases, _ = route_plan(catalog, audit, scenario)
        assert len(cases) == 1 and cases[0]["algorithm"] == algorithm
        assert cases[0]["scope"] == "full-route"
        assert cases[0]["algorithmSelection"] == select_algorithm(catalog, binding, scenario)
        mounting = next(c for c in runtime_plan(catalog, "mountings") if c["binding"] == key)
        assert mounting["algorithm"] == select_algorithm(catalog, binding, mounting["scenario"])["algorithm"]


def test_open_field_preference_preserves_exact_overrides_and_other_scenes():
    catalog = read_catalog(CATALOG)
    for binding in catalog["bindings"]:
        for scene in ("arable_field", "wide_headland", "tomato_greenhouse", "farmyard", "orchard"):
            selection = select_algorithm(catalog, binding, scene)
            if scene == "wide_headland" and binding["preset"] in ("car4_harrow", "cab_groove", "harvester_header"):
                assert selection["source"] == "scene_binding"
                assert selection["components"]["localization"] == "navsat"
                assert selection["components"]["controller"] == ("rpp" if binding["preset"] == "harvester_header" else "mppi")
            elif scene == "arable_field" and binding["car"] == "Car4":
                assert selection["source"] == "scene_vehicle"
                assert selection["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
                assert selection["allowedLocalizationModes"] == ["navsat"]
            elif scene == "wide_headland" or (scene == "arable_field" and next(
                    v for v in catalog["vehicles"] if v["car"] == binding["car"])["sceneGroup"] in ("large_tractor", "harvester")):
                assert selection["source"] == "scene_default"
                assert selection["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
            else:
                assert selection["source"] == "physical_default"
                assert selection["algorithm"] == catalog["defaultAlgorithm"]
            assert selection["components"]["planner"] == "smac_hybrid"
            assert selection["claim"] == "test_recipe_not_runtime_acceptance"
            assert selection["reason"] and selection["conditions"]


@pytest.mark.parametrize("car", ["Car0", "Car3", "Car4", "Car5", "Car6", "Car8"])
@pytest.mark.parametrize("scene", ["arable_field", "wide_headland"])
def test_small_vehicles_default_fusion_except_user_confirmed_scene(car, scene):
    catalog = read_catalog(CATALOG)
    binding = next(b for b in catalog["bindings"] if b["car"] == car and b["implement"] == "none")
    expected = "stvl_off_navsat_smac_hybrid_mppi" if scene == "wide_headland" or (car == "Car4" and scene == "arable_field") else catalog["defaultAlgorithm"]
    assert select_algorithm(catalog, binding, scene)["algorithm"] == expected


def test_selection_does_not_mutate_catalog_or_share_conditions():
    catalog = read_catalog(CATALOG)
    original = copy.deepcopy(catalog)
    for preset in ("car4_harrow", "car4_chisel"):
        binding = next(b for b in catalog["bindings"] if b["preset"] == preset)
        selection = select_algorithm(catalog, binding, "wide_headland")
        selection["conditions"].clear()
        assert select_algorithm(catalog, binding, "wide_headland")["conditions"]
    assert catalog == original
    with pytest.raises(ValueError, match="Unknown scene"):
        select_algorithm(catalog, catalog["bindings"][0], "unregistered_scene")


@pytest.mark.parametrize("scope", ["scene", "vehicle", "binding"])
@pytest.mark.parametrize("choice, message", [
    ({"typo_scene": {}}, "Unknown scene"),
    ({"farmyard": {"algorithm": "unvalidated_algorithm"}}, "Unknown scene test algorithm"),
    ({"farmyard": {"algorithm": "stvl_off_fastlivo_rtk_smac_hybrid_mppi"}}, "needs a reason"),
    ({"farmyard": {"algorithm": "stvl_off_fastlivo_rtk_smac_hybrid_mppi", "reason": "tested"}}, "explicit conditions"),
    ({"farmyard": {"algorithm": "stvl_off_fastlivo_rtk_smac_hybrid_mppi", "reason": "tested", "conditions": [""]}}, "explicit conditions"),
])
def test_invalid_scene_selection_is_rejected(tmp_path, choice, message, scope):
    catalog = read_catalog(CATALOG)
    target = catalog if scope == "scene" else catalog["vehicles"][0] if scope == "vehicle" else catalog["bindings"][0]
    target["sceneAlgorithms"] = choice
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(catalog))
    with pytest.raises(ValueError, match=message):
        read_catalog(path)


def test_unscoped_mounting_algorithm_is_rejected(tmp_path):
    catalog = read_catalog(CATALOG)
    catalog["bindings"][0]["testAlgorithm"] = catalog["defaultAlgorithm"]
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(catalog))
    with pytest.raises(ValueError, match="must be scoped"):
        read_catalog(path)


def test_metric_transport_plan_keeps_all_ten_cars_with_an_implement():
    catalog = read_catalog(CATALOG)
    plan = json.loads((CATALOG.parent / "metric_transport_acceptance.json").read_text())
    validate_plan(plan["cases"], catalog)
    for case in plan["cases"]:
        binding = next(b for b in catalog["bindings"] if binding_id(b) == case["binding"])
        assert case["algorithm"] == select_algorithm(catalog, binding, case["scenario"])["algorithm"]
    assert {case["binding"].split("__")[0] for case in plan["cases"]} == {f"Car{i}" for i in range(10)}
    assert len(plan["cases"]) == 10
    assert all(case["scope"] == "full-route" and not case["binding"].endswith("__none") for case in plan["cases"])


@pytest.mark.parametrize("change", [
    {"id": "../../outside"}, {"vehicleId": "unity_car3"},
    {"manipulator": "true"}, {"algorithm": "unregistered_algorithm"},
    {"scenario": "unregistered_scene"}, {"scope": "real_chassis"},
    {"distanceM": float("nan")}, {"lateralM": True}, {"armDemo": "true"}])
def test_runtime_plan_rejects_unsafe_or_mismatched_selection(change):
    catalog = read_catalog(CATALOG)
    cases = runtime_plan(catalog, "mountings")
    validate_plan(cases, catalog)
    bad = copy.deepcopy(cases[:1])
    bad[0].update(change)
    with pytest.raises(ValueError):
        validate_plan(bad, catalog)


def test_runtime_plan_cannot_mislabel_a_manual_algorithm_as_the_scene_recipe():
    catalog = read_catalog(CATALOG)
    plan = runtime_plan(catalog, "mountings")[:1]
    plan[0]["algorithm"] = "stvl_off_navsat_smac_hybrid_mppi"
    with pytest.raises(ValueError, match="differs from the current scene recipe"):
        validate_plan(plan, catalog)
    # Explicit component experiments remain allowed without a recipe claim.
    plan[0].pop("algorithmSelection")
    validate_plan(plan, catalog)


def test_runner_returns_failure_for_localization_drift_even_when_motion_passes(tmp_path, monkeypatch):
    catalog = read_catalog(CATALOG)
    cases = runtime_plan(catalog, "mountings")[:1]
    monkeypatch.setattr(Path, "home", lambda: tmp_path)

    def fake_run(command, **kwargs):
        output = Path(command[command.index("--output") + 1])
        output.mkdir()
        (output / "summary.json").write_text(json.dumps({"results": [{
            "status": "PASS", "horizontal_pose_rmse_m": 1.25, "horizontal_pose_max_error_m": 2.}]}))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("agribot_farm_sim.compatibility.subprocess.run", fake_run)
    output = tmp_path / "output"
    assert run_plan(cases, output, tmp_path / "runner", validation_limits=catalog["validationLimits"]) == 1
    attempt = json.loads(next(output.glob("*.attempt.json")).read_text())
    assert attempt["returncode"] == 0 and len(attempt["qualityErrors"]) == 2


def test_runner_does_not_launch_next_case_after_cleanup_failure(tmp_path, monkeypatch):
    catalog = read_catalog(CATALOG)
    cases = runtime_plan(catalog, "mountings")[:2]
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        output = Path(command[command.index("--output") + 1])
        output.mkdir()
        (output / "summary.json").write_text(json.dumps({"results": [{"status": "FAIL", "cleanup_error": "still running"}]}))
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr("agribot_farm_sim.compatibility.subprocess.run", fake_run)
    with pytest.raises(RuntimeError, match="Cleanup failed"):
        run_plan(cases, tmp_path / "output", tmp_path / "runner", validation_limits=catalog["validationLimits"])
    assert len(calls) == 1


@pytest.mark.parametrize("gui, rviz", [(False, False), (True, False), (False, True), (True, True)])
@pytest.mark.parametrize("timeout", [None,4165])
def test_serial_runner_forwards_display_options_and_keeps_the_full_route(tmp_path, monkeypatch, gui, rviz,timeout):
    catalog = read_catalog(CATALOG)
    case = runtime_plan(catalog, "mountings")[0]
    case["scope"] = "full-route"
    if timeout is not None:
        case["motionTimeoutS"] = timeout
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        output = Path(command[command.index("--output") + 1])
        output.mkdir()
        result = complete_result(case["algorithm"])
        (output / "summary.json").write_text(json.dumps({"results": [result]}))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("agribot_farm_sim.compatibility.subprocess.run", fake_run)
    assert run_plan([case], tmp_path / "output", tmp_path / "runner",
                    validation_limits=catalog["validationLimits"], gui=gui, rviz=rviz) == 0
    command = calls[0]
    assert ("--gui" in command) == gui and ("--rviz" in command) == rviz
    assert command[command.index("--scope") + 1] == "full-route"
    assert command[command.index("--motion-timeout") + 1] == str(timeout or 2400)
    assert command[command.index("--domain") + 1] == "73"
    assert "--local-maneuver" not in command and "--motion-distance" not in command


def test_serial_runner_rejects_a_short_check_mislabeled_as_full_route(tmp_path, monkeypatch):
    catalog = read_catalog(CATALOG)
    case = runtime_plan(catalog, "mountings")[0]
    case["scope"] = "full-route"
    monkeypatch.setattr(Path, "home", lambda: tmp_path)

    def fake_run(command, **kwargs):
        output = Path(command[command.index("--output") + 1])
        output.mkdir()
        result = complete_result(case["algorithm"], scope="smoke", full_route="NOT_RUN")
        (output / "summary.json").write_text(json.dumps({"results": [result]}))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("agribot_farm_sim.compatibility.subprocess.run", fake_run)
    output = tmp_path / "output"
    assert run_plan([case], output, tmp_path / "runner", validation_limits=catalog["validationLimits"]) == 1
    attempt = json.loads(next(output.glob("*.attempt.json")).read_text())
    assert attempt["qualityErrors"] == ["Runtime evidence did not complete the requested full route"]


@pytest.mark.parametrize("binding", read_catalog(CATALOG)["bindings"], ids=binding_id)
def test_each_binding_resolves_the_current_canonical_config(binding):
    from ament_index_python.packages import get_package_share_directory
    generated = Path(get_package_share_directory("agribot_vehicle_description")) / "generated"
    config, _, _ = configuration(binding, generated)
    assert bool(config["manipulators"]) == (binding["implement"] == "lite6")
    if binding["implement"] not in ("none", "lite6"):
        bad = copy.deepcopy(binding)
        bad["implement"] = "unknown_implement"
        with pytest.raises(ValueError, match="implement differs"):
            configuration(bad, generated)


def test_sensor_proxy_review_detects_intrusion_and_clears_a_free_mount():
    from ament_index_python.packages import get_package_share_directory
    from agribot_vehicle_config_tools.implements import collision_boxes
    binding = next(b for b in read_catalog(CATALOG)["bindings"] if b["implement"] == "combine_header")
    config, _, _ = configuration(binding, Path(get_package_share_directory("agribot_vehicle_description")) / "generated")
    box = collision_boxes(config)[0]
    for sensor in config["sensors"]:
        if sensor["sensorType"] in ("lidar", "camera"):
            sensor["pose"]["xyz"] = [(box[i] + box[i + 3]) / 2 for i in range(3)]
            sensor.pop("measurementFramePose", None)
            sensor.pop("measurementFrameId", None)
    findings = installation_review(config)
    assert {f["sensor"] for f in findings} >= {"lidar", "right_camera"}
    for sensor in config["sensors"]:
        if sensor["sensorType"] in ("lidar", "camera"):
            sensor["pose"]["xyz"][2] = box[5] + 1.
    assert not installation_review(config)
