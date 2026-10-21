"""Explicit, read-only migration of verification-only source changes."""

import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_acceptance_policy(path):
    if path is None:
        return None
    path = Path(path).resolve()
    policy = json.loads(path.read_text())
    legacy = Path(policy["legacySourceArchive"])
    current = Path(policy["verifierPath"])
    if sha256(legacy) != policy["legacySha256"] or sha256(current) != policy["currentSha256"]:
        raise ValueError("Acceptance migration source fingerprint changed")
    old_tree = ast.parse(legacy.read_text())
    new_tree = ast.parse(current.read_text())
    helpers = [node for node in new_tree.body if isinstance(node, ast.FunctionDef)
               and node.name == "driven_waypoint_distances"]
    if len(helpers) != 1:
        raise ValueError("Expected exactly one isolated driven-route helper")
    new_tree.body.remove(helpers[0])

    class RestoreCall(ast.NodeTransformer):
        count = 0

        def visit_Call(self, node):
            if isinstance(node.func, ast.Name) and node.func.id == "driven_waypoint_distances":
                expected = ast.parse("f(node.terminal_observation, args.scope == 'full-route')").body[0].value.args
                if (len(node.args) != 4 or node.keywords or
                        ast.dump(ast.Tuple(elts=node.args[2:]), include_attributes=False) !=
                        ast.dump(ast.Tuple(elts=expected), include_attributes=False)):
                    raise ValueError("Unexpected acceptance call change")
                self.count += 1
                node = copy.deepcopy(node)
                node.func.id = "ordered_waypoint_distances"
                node.args = node.args[:2]
            return self.generic_visit(node)

    restore = RestoreCall()
    new_tree = restore.visit(new_tree)
    if restore.count != 1 or ast.dump(old_tree, include_attributes=False) != ast.dump(new_tree, include_attributes=False):
        raise ValueError("Migration changes more than the isolated offline acceptance calculation")
    spec = importlib.util.spec_from_file_location("rechecked_runtime_acceptance", current)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {**policy, "path": str(path), "runtime": module}


def recheck_legacy_result(source, summary, result, policy):
    if policy is None or summary.get("file_sha256", {}).get(policy["verifierPath"]) != policy["legacySha256"]:
        return None
    source = Path(source).resolve()
    frozen = policy["recordInputs"].get(str(source))
    if not frozen or sha256(source) != frozen["summarySha256"]:
        return None
    review = {"policyFile": policy["path"], "source": str(source),
              "legacyVerifierSha256": policy["legacySha256"], "currentVerifierSha256": policy["currentSha256"],
              "verifierPath": policy["verifierPath"], "compatibleFingerprint": False}
    if result.get("status") not in ("PASS", "FAIL") or result.get("scope") != "full-route":
        return {**review, "status": "RESTART_REQUIRED"}
    affected_error = "RuntimeError: Vehicle missed an intermediate 0.75m neighborhood or final half-wheelbase tolerance"
    if result["status"] == "FAIL" and result.get("error") != affected_error:
        return {**review, "status": "FAIL_UNCHANGED", "compatibleFingerprint": True}
    runtime = policy["runtime"]
    try:
        trace_path = source.parent / (result["id"] + ".trace.json")
        if sha256(trace_path) != frozen["traceSha256"][result["id"]]:
            raise ValueError("Legacy trajectory fingerprint changed")
        trace = json.loads(trace_path.read_text())
        waypoint_paths = [Path(name) for name in summary["file_sha256"]
                          if Path(name).suffix in (".yaml", ".yml") and "waypoint" in Path(name).stem]
        if len(waypoint_paths) != 1:
            raise ValueError("Expected one recorded route")
        import yaml
        targets = [runtime.pose(point["x"], point["y"], point["yaw"])
                   for point in yaml.safe_load(waypoint_paths[0].read_text())["waypoints"]]
        checks = result["checks"]
        observation = checks["terminal_stop_observation"]
        radii = checks["driven_waypoint_acceptance_radii_m"]
        allowance = checks["final_sample_allowance_m"]
        if (checks["controller_action_status"] != 4 or observation["stationary_hold_s"] < 1 or
                observation["fresh_terminal_samples"] < 2 or
                not runtime.terminal_motion_stopped(observation["command"], observation["chassis_twist"]) or
                not targets or len(targets) != len(radii) or any(radius != .75 for radius in radii[:-1]) or allowance != .03 or
                checks["driven_waypoints"] != len(targets) or checks["planned_waypoints"] != len(targets)):
            raise ValueError("Native completion or original acceptance gates are missing")
        config_paths = [Path(name) for name in summary["file_sha256"] if name.endswith("/vehicle_config.json")]
        if len(config_paths) != 1 or not math.isclose(
                json.loads(config_paths[0].read_text())["geometry"]["wheelbaseM"] / 2, radii[-1]):
            raise ValueError("Final radius differs from half the recorded wheelbase")
        path = runtime.PathMessage()
        path.poses = [runtime.pose(row["truth"][0], row["truth"][1], 0) for row in trace]
        errors = runtime.driven_waypoint_distances(path, targets, observation, True)
        accepted = all(math.isfinite(error) and error <= radius + (allowance if i == len(errors)-1 else 0)
                       for i, (error, radius) in enumerate(zip(errors, radii)))
        review.update(recheckedErrorsM=errors, originalRadiiM=radii, originalSampleAllowanceM=allowance)
        if result["status"] == "FAIL":
            return {**review, "status": "RETEST_REQUIRED" if accepted else "FAIL_UNCHANGED",
                    "compatibleFingerprint": not accepted}
        if not accepted or result.get("full_route") != "PASS" or result.get("abnormal_process_exits") or result.get("cleanup_error"):
            raise ValueError("Legacy success does not satisfy the unchanged acceptance contract")
        return {**review, "status": "PASS_REVALIDATED", "compatibleFingerprint": True}
    except (KeyError, ValueError, OSError, TypeError, IndexError) as error:
        return {**review, "status": "LEGACY_REVIEW_REQUIRED", "error": str(error),
                "compatibleFingerprint": result["status"] == "FAIL"}
