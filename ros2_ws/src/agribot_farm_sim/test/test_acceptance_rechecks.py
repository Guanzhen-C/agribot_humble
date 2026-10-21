import hashlib
import json
from pathlib import Path

import pytest

from agribot_farm_sim.acceptance_rechecks import load_acceptance_policy
from agribot_farm_sim.compatibility import evidence_record


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def source_policy(tmp_path):
    legacy = tmp_path / "old.py"
    current = tmp_path / "new.py"
    old = ("def ordered_waypoint_distances(path, targets):\n    return []\n"
           "def run_case(driven, expected_waypoints, node, args):\n"
           "    return ordered_waypoint_distances(driven, expected_waypoints)\n")
    new = old.replace("return ordered_waypoint_distances(driven, expected_waypoints)",
                      "return driven_waypoint_distances(driven, expected_waypoints, node.terminal_observation, args.scope == 'full-route')")
    new += "def driven_waypoint_distances(path, targets, observation, full_route):\n    return []\n"
    legacy.write_text(old)
    current.write_text(new)
    policy = {"verifierPath": str(current), "legacySourceArchive": str(legacy),
              "legacySha256": digest(legacy), "currentSha256": digest(current), "recordInputs": {}}
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(policy))
    return path, policy


def test_acceptance_source_policy_is_opt_in():
    assert load_acceptance_policy(None) is None


def test_policy_allows_only_the_exact_helper_and_offline_call(source_policy):
    path, policy = source_policy
    loaded = load_acceptance_policy(path)
    assert loaded["currentSha256"] == policy["currentSha256"]
    assert loaded["legacySha256"] != loaded["currentSha256"]


def test_source_fingerprint_change_is_rejected(source_policy):
    path, policy = source_policy
    Path(policy["verifierPath"]).write_text("# replaced\n")
    with pytest.raises(ValueError, match="fingerprint"):
        load_acceptance_policy(path)


def test_a_change_outside_acceptance_is_rejected_even_with_updated_hash(source_policy):
    path, policy = source_policy
    current = Path(policy["verifierPath"])
    current.write_text(current.read_text() + "unsafe_runtime_change = True\n")
    policy["currentSha256"] = digest(current)
    path.write_text(json.dumps(policy))
    with pytest.raises(ValueError, match="more than"):
        load_acceptance_policy(path)


def test_wrong_acceptance_arguments_are_rejected(source_policy):
    path, policy = source_policy
    current = Path(policy["verifierPath"])
    current.write_text(current.read_text().replace("node.terminal_observation", "node.estimate"))
    policy["currentSha256"] = digest(current)
    path.write_text(json.dumps(policy))
    with pytest.raises(ValueError, match="call change"):
        load_acceptance_policy(path)


def test_verifier_exception_never_hides_an_algorithm_parameter_change(tmp_path):
    verifier, parameters = tmp_path / "verifier.py", tmp_path / "parameters.yaml"
    verifier.write_text("new verifier")
    parameters.write_text("changed physical parameters")
    result = {"status": "PASS", "scope": "full-route", "full_route": "PASS"}
    summary = {"file_sha256": {str(verifier): "old-verifier-hash", str(parameters): "old-parameter-hash"}}
    approved = {"compatibleFingerprint": True, "verifierPath": str(verifier),
                "legacyVerifierSha256": "old-verifier-hash", "currentVerifierSha256": digest(verifier)}
    record = evidence_record(summary, result, verification_recheck=approved)
    assert record["changedInputs"] == [str(parameters)]
    assert not record["eligibleForSimulationRecommendation"]
    assert str(verifier) in evidence_record(summary, result)["changedInputs"]
