import copy
import json
from pathlib import Path
import re

import pytest

from agribot_farm_sim.compatibility import binding_id, read_catalog, recommend_algorithm, select_algorithm


CONFIG = Path(__file__).resolve().parents[1] / "config"


def test_car9_wide_headland_confirmation_is_scoped_to_navsat_rpp_header():
    labels = json.loads((CONFIG / "manual_confirmations_car9_wide_headland.json").read_text())
    catalog = read_catalog(CONFIG / "compatibility.json")
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] == "Car9" and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected == {"Car9__combine_header"}
    assert labels["vehicle"] == "Car9" and labels["scenario"] == "wide_headland"
    assert labels["algorithm"] == "stvl_off_navsat_smac_hybrid_rpp"
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/0078")


def test_wide_headland_user_group_excludes_harvester_and_individual_test_claims():
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / "manual_confirmations_wide_headland_non_harvesters.json").read_text())
    assert set(labels["vehicles"]) == {f"Car{i}" for i in range(9)}
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] in labels["vehicles"] and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected
    assert all(row["vehicle"] != "Car9" for row in labels["confirmations"])
    assert labels["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
    assert labels["scenario"] == "wide_headland"
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert [row["binding"] for row in labels["confirmations"] if row["individuallyRetestedInThisRound"]] == ["Car0__lite6"]


def test_wide_headland_all_bindings_use_navsat_and_preserve_controller_overrides():
    catalog = read_catalog(CONFIG / "compatibility.json")
    for binding in catalog["bindings"]:
        choice = select_algorithm(catalog, binding, "wide_headland")
        assert choice["components"]["localization"] == "navsat"
        assert choice["allowedLocalizationModes"] == ["navsat"]
        if binding["preset"] == "harvester_header":
            assert choice["components"]["controller"] == "rpp"
        if binding["car"] == "Car0":
            assert select_algorithm(catalog, binding, "vegetable_field")["algorithm"] == catalog["defaultAlgorithm"]


@pytest.mark.parametrize("car,trial,count", [("Car1", "0068", 8), ("Car2", "0069", 8), ("Car4", "0071", 8), ("Car5", "0072", 8), ("Car6", "0073", 6), ("Car7", "0074", 9)])
def test_vegetable_tractor_groups_cover_current_supported_implements_only(car, trial, count):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_vegetable_field.json").read_text())
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected
    assert len(labels["confirmations"]) == count
    assert labels["vehicle"] == car and labels["scenario"] == "vegetable_field"
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["coversAllCurrentSupportedImplements"] is True
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)
    if car == "Car7":
        trailer = next(row for row in labels["confirmations"] if row["binding"].endswith("__farm_trailer"))
        assert trailer["requiresIndividualDynamicsReview"] is True
        assert trailer["individualAutomaticRecommendationAllowed"] is False


@pytest.mark.parametrize("car,trial,implement", [("Car0", "0067", "lite6"), ("Car3", "0070", "lite6"), ("Car8", "0075", "front_sickle_bar")])
def test_vegetable_single_implement_groups_cover_current_supported_implements_only(car, trial, implement):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_vegetable_field.json").read_text())
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected == {car + "__" + implement}
    assert labels["vehicle"] == car and labels["scenario"] == "vegetable_field"
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["coversAllCurrentSupportedImplements"] is True
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)


def test_car3_vineyard_navsat_confirmation_supersedes_only_old_scene_recipe():
    labels = json.loads((CONFIG / "manual_confirmations_car3_vineyard_navsat.json").read_text())
    old = json.loads((CONFIG / "manual_confirmations_car3_vineyard.json").read_text())
    assert labels["supersedesConfirmationId"] == old["confirmationId"]
    assert labels["vehicle"] == "Car3" and labels["scenario"] == "vineyard"
    assert labels["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
    assert labels["confirmedLocalizationModes"] == ["navsat"]
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/0063")
    assert [row["binding"] for row in labels["confirmations"]] == ["Car3__lite6"]


@pytest.mark.parametrize("car,mode,trial,count", [("Car4", "navsat", "0061", 8), ("Car5", "fastlivo_rtk", "0062", 8), ("Car6", "navsat", "0065", 6)])
def test_vineyard_tractor_group_covers_current_supported_implements(car, mode, trial, count):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_vineyard.json").read_text())
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected
    assert len(labels["confirmations"]) == count
    assert labels["scenario"] == "vineyard" and labels["vehicle"] == car
    assert labels["algorithm"] == f"stvl_off_{mode}_smac_hybrid_mppi"
    assert labels["confirmedLocalizationModes"] == [mode]
    assert labels["coversAllCurrentSupportedImplements"] is True
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)


@pytest.mark.parametrize("car", ["Car3", "Car4", "Car6"])
def test_vineyard_selected_vehicles_use_navsat_only_without_changing_other_scenes(car):
    catalog = read_catalog(CONFIG / "compatibility.json")
    for binding in catalog["bindings"]:
        if binding["car"] != car:
            continue
        choice = select_algorithm(catalog, binding, "vineyard")
        assert choice["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
        assert choice["allowedLocalizationModes"] == ["navsat"]
        assert choice["source"] == "scene_vehicle"
        assert select_algorithm(catalog, binding, "farmyard")["algorithm"] == catalog["defaultAlgorithm"]
    for binding in catalog["bindings"]:
        if binding["car"] in {"Car0", "Car5", "Car8"}:
            assert select_algorithm(catalog, binding, "vineyard")["algorithm"] == catalog["defaultAlgorithm"]


@pytest.mark.parametrize("car,trial,implement", [("Car0", "0057", "lite6"), ("Car3", "0059", "lite6"), ("Car8", "0066", "front_sickle_bar")])
def test_vineyard_group_confirmations_cover_current_implements_only(car, trial, implement):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_vineyard.json").read_text())
    expected = {binding_id(row) for row in catalog["bindings"]
                if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected == {car + "__" + implement}
    assert labels["scenario"] == "vineyard" and labels["vehicle"] == car
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["coversAllCurrentSupportedImplements"] is True
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)


@pytest.mark.parametrize("car,binding,trial", [
    ("Car0", "Car0__lite6", "0048"),
    ("Car1", "Car1__compact_chisel_plow", "0049"),
    ("Car3", "Car3__lite6", "0051"),
    ("Car4", "Car4__compact_chisel_plow", "0052"),
    ("Car5", "Car5__compact_chisel_plow", "0053"),
    ("Car6", "Car6__compact_chisel_plow", "0054"),
    ("Car8", "Car8__front_sickle_bar", "0055"),
])
def test_strawberry_confirmations_are_limited_to_the_selected_combination(car, binding, trial):
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_strawberry_field.json").read_text())
    assert labels["vehicle"] == car and labels["scenario"] == "strawberry_field"
    assert labels["algorithm"] == read_catalog(CONFIG / "compatibility.json")["defaultAlgorithm"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)
    assert [row["binding"] for row in labels["confirmations"]] == [binding]
    assert "confirmedScenarios" not in labels


@pytest.mark.parametrize("car,binding,trial", [
    ("Car0", "Car0__lite6", "0039"),
    ("Car1", "Car1__compact_chisel_plow", "0040"),
    ("Car2", "Car2__compact_chisel_plow", "0041"),
    ("Car3", "Car3__lite6", "0042"),
    ("Car4", "Car4__compact_chisel_plow", "0043"),
    ("Car5", "Car5__compact_chisel_plow", "0044"),
    ("Car6", "Car6__compact_chisel_plow", "0045"),
    ("Car7", "Car7__compact_chisel_plow", "0046"),
    ("Car8", "Car8__front_sickle_bar", "0047"),
])
def test_tomato_confirmations_do_not_expand_to_other_scenes_or_implements(car, binding, trial):
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_tomato_greenhouse.json").read_text())
    assert labels["vehicle"] == car and labels["scenario"] == "tomato_greenhouse"
    assert labels["algorithm"] == read_catalog(CONFIG / "compatibility.json")["defaultAlgorithm"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["representativeInteractiveTrial"].endswith("/" + trial)
    assert [row["binding"] for row in labels["confirmations"]] == [binding]
    assert "confirmedScenarios" not in labels


def test_car3_farmyard_confirmation_is_scoped_to_the_adaptive_contact_trial():
    labels = json.loads((CONFIG / "manual_confirmations_car3_farmyard.json").read_text())
    assert labels["vehicle"] == "Car3" and labels["scenario"] == "farmyard"
    assert labels["algorithm"] == read_catalog(CONFIG / "compatibility.json")["defaultAlgorithm"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert "confirmedScenarios" not in labels
    assert labels["representativeInteractiveTrial"].endswith("/0031")
    assert len(labels["confirmations"]) == 1
    record = labels["confirmations"][0]
    assert record["binding"] == "Car3__lite6" and record["vehicleId"] == "unity_car3"
    assert record["vehicleConfigSha256"] == "c9f3043c031691504a15bcae8fd3c271e02e6c85132d37b1fbce0f522f396646"


@pytest.mark.parametrize("car,filename,count", [
    ("Car1", "manual_confirmations_car1_farmyard_all_implements.json", 8),
    ("Car2", "manual_confirmations_car2_farmyard.json", 8),
    ("Car4", "manual_confirmations_car4_farmyard.json", 8),
    ("Car5", "manual_confirmations_car5_farmyard.json", 8),
    ("Car6", "manual_confirmations_car6_farmyard.json", 6),
    ("Car8", "manual_confirmations_car8_farmyard.json", 1),
])
def test_farmyard_group_confirmations_cover_current_implements_only(car, filename, count):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / filename).read_text())
    bindings = {binding_id(row): row for row in catalog["bindings"]
                if row["car"] == car and row["implement"] != "none"}
    assert labels["vehicle"] == car and labels["scenario"] == "farmyard"
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["confirmedLocalizationModes"] == ["fastlivo_rtk"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert "confirmedScenarios" not in labels
    assert len(labels["confirmations"]) == len(bindings) == count
    assert {row["binding"] for row in labels["confirmations"]} == bindings.keys()
    for row in labels["confirmations"]:
        assert row["vehicleId"] == bindings[row["binding"]]["preset"]
        assert row["implementName"] == bindings[row["binding"]]["name"]
        assert re.fullmatch("[0-9a-f]{64}", row["vehicleConfigSha256"])
    if car == "Car1":
        assert labels["supersedesConfirmationId"] == "car1_farmyard_chisel_default_user_20261021"


def test_car0_farmyard_confirmation_is_limited_to_the_selected_arm_trial():
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / "manual_confirmations_car0_farmyard.json").read_text())
    binding = next(row for row in catalog["bindings"] if binding_id(row) == "Car0__lite6")
    assert labels["vehicle"] == "Car0" and labels["scenario"] == "farmyard"
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["userStatement"] == "很好，这个通过，下一个"
    assert "confirmedScenarios" not in labels
    assert len(labels["confirmations"]) == 1
    confirmation = labels["confirmations"][0]
    assert confirmation["binding"] == binding_id(binding)
    assert confirmation["vehicleId"] == binding["preset"]
    assert confirmation["implementName"] == binding["name"]
    assert re.fullmatch("[0-9a-f]{64}", confirmation["vehicleConfigSha256"])


@pytest.mark.parametrize("car", ["Car1", "Car2"])
def test_large_tractor_navsat_confirmations_cover_only_current_arable_implements(car):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_arable.json").read_text())
    bindings = {binding_id(row): row for row in catalog["bindings"] if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == bindings.keys()
    assert len(labels["confirmations"]) == len(bindings) == 8
    assert labels["vehicle"] == car and labels["scenario"] == "arable_field"
    assert labels["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
    assert labels["confirmedLocalizationModes"] == ["navsat"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    for row in labels["confirmations"]:
        assert row["vehicleId"] == bindings[row["binding"]]["preset"]
        assert row["implementName"] == bindings[row["binding"]]["name"]
        assert re.fullmatch("[0-9a-f]{64}", row["vehicleConfigSha256"])


@pytest.mark.parametrize("car", ["Car0", "Car3"])
def test_small_vehicle_confirmations_cover_only_arable_default_current_implements(car):
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / f"manual_confirmations_{car.lower()}_arable.json").read_text())
    expected = {binding_id(row) for row in catalog["bindings"] if row["car"] == car and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected == {car + "__lite6"}
    assert len(labels["confirmations"]) == 1
    assert labels["vehicle"] == car and labels["scenario"] == "arable_field"
    assert labels["algorithm"] == catalog["defaultAlgorithm"]
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert "confirmedScenarios" not in labels
    assert not (CONFIG / "manual_confirmations_car3_all_scenes.json").exists()
    if car == "Car3":
        assert labels["supersedesConfirmationId"] == "car3_all_current_scenes_default_user_20261021"


def test_car9_user_confirmation_covers_only_current_header():
    labels = json.loads((CONFIG / "manual_confirmations_car9_arable.json").read_text())
    expected = {binding_id(row) for row in read_catalog(CONFIG / "compatibility.json")["bindings"]
                if row["car"] == "Car9" and row["implement"] != "none"}
    assert {row["binding"] for row in labels["confirmations"]} == expected == {"Car9__combine_header"}
    assert labels["algorithm"] == "stvl_off_navsat_smac_hybrid_mppi"
    assert labels["evidenceType"] == "user_group_confirmation"
    assert labels["individualAutomaticFullRouteAcceptance"] is False


def test_manual_labels_cover_exactly_the_eight_supported_car4_implements():
    catalog = read_catalog(CONFIG / "compatibility.json")
    labels = json.loads((CONFIG / "manual_confirmations.json").read_text())
    bindings = {binding_id(row): row for row in catalog["bindings"]
                if row["car"] == "Car4" and row["implement"] != "none"}
    records = {row["binding"]: row for row in labels["confirmations"]}
    assert len(records) == len(labels["confirmations"]) == 8
    assert records.keys() == bindings.keys()
    assert labels["vehicle"] == "Car4" and labels["scenario"] == "arable_field"
    assert labels["evidenceType"] == "user_group_confirmation" and labels["status"] == "PASS"
    assert labels["individualAutomaticFullRouteAcceptance"] is False
    assert labels["routeSource"] == "user_selected_waypoints"
    assert labels["allowedLocalizationModes"] == ["navsat"]
    assert labels["rejectedLocalizationModes"] == ["fastlivo_rtk"]
    for key, row in records.items():
        assert row["status"] == "PASS" and row["vehicleId"] == bindings[key]["preset"]
        assert row["implementName"] == bindings[key]["name"]
        assert re.fullmatch("[0-9a-f]{64}", row["vehicleConfigSha256"])


def test_vehicle_rule_selects_navsat_for_every_car4_mount_only_in_arable():
    catalog = read_catalog(CONFIG / "compatibility.json")
    selected = [row for row in catalog["bindings"] if row["car"] == "Car4"]
    assert len(selected) == 9
    for row in selected:
        scene = select_algorithm(catalog, row, "arable_field")
        assert scene["components"]["localization"] == "navsat" and scene["source"] == "scene_vehicle"
        assert scene["allowedLocalizationModes"] == ["navsat"]
        assert select_algorithm(catalog, row, "tomato_greenhouse")["algorithm"] == catalog["defaultAlgorithm"]
    assert all(select_algorithm(catalog, row, "arable_field")["algorithm"] == catalog["defaultAlgorithm"]
               for row in catalog["bindings"] if row["car"] in {"Car0", "Car3", "Car5", "Car6", "Car8"})


def test_disallowed_livo_evidence_is_not_selected_and_manual_pass_is_not_auto_pass():
    catalog = read_catalog(CONFIG / "compatibility.json")
    binding = next(row for row in catalog["bindings"] if row["preset"] == "car4_mulch")
    record = {"binding": binding_id(binding), "scenario": "arable_field", "scope": "full-route",
              "currentInputsMatch": True, "fullRoute": True, "qualityStatus": "PASS",
              "eligibleForSimulationRecommendation": True, "horizontalRmseM": .01,
              "horizontalMaxErrorM": .05, "algorithm": catalog["defaultAlgorithm"], "source": "test"}
    assert recommend_algorithm(catalog, binding, "arable_field", [record])["algorithm"] is None
    assert recommend_algorithm(catalog, binding, "arable_field", [])["status"] == "PENDING_VALIDATION"
    navsat = {**record, "algorithm": "stvl_off_navsat_smac_hybrid_mppi"}
    choice = recommend_algorithm(catalog, binding, "arable_field", [record, navsat])
    assert choice["status"] == "VALIDATED" and choice["algorithm"] == navsat["algorithm"]
    assert choice["source"] == "scene_vehicle"


@pytest.mark.parametrize("allowed", [[], "navsat", ["navsat", "navsat"], ["bad"], ["fastlivo_rtk"], [True]])
def test_invalid_allowed_localization_policy_rejected(tmp_path, allowed):
    catalog = copy.deepcopy(read_catalog(CONFIG / "compatibility.json"))
    vehicle = next(row for row in catalog["vehicles"] if row["car"] == "Car4")
    vehicle["sceneAlgorithms"]["arable_field"]["allowedLocalizationModes"] = allowed
    path = tmp_path / "compatibility.json"
    path.write_text(json.dumps(catalog))
    with pytest.raises(ValueError, match="allowed localization modes"):
        read_catalog(path)
