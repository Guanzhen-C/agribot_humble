import csv
import importlib.util
import json
from pathlib import Path

import pytest


def module():
    path = Path(__file__).resolve().parents[1]/"scripts/display_scanned_route_result.py"
    spec = importlib.util.spec_from_file_location("scanned_result_display", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_result_display_preserves_actual_path_without_navigation(tmp_path):
    m = module()
    route = {"path": [{"x": 1., "y": 2., "yaw": .5}], "anchor_path_indices": [0]}
    (tmp_path/"result.json").write_text(json.dumps({"passed": True, "diagnostic_only": False}))
    (tmp_path/"saved_route.json").write_text(json.dumps(route))
    with (tmp_path/"actual_trajectory.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_time", "x", "y", "yaw"])
        writer.writerows([[1, 1.1, 2.2, .5], [2, 1.2, 2.3, .6]])
    planned, actual = m.read_result(tmp_path)
    assert planned == route and len(actual) == 2
    message = m.path_message(actual)
    assert message.header.frame_id == "map"
    assert message.poses[-1].pose.position.x == 1.2
    source = Path(m.__file__).read_text()
    for forbidden in ("ActionClient", "TransformBroadcaster", "cmd_vel", "SavedPathRunner"):
        assert forbidden not in source


@pytest.mark.parametrize("report", [{"passed": False}, {"passed": True, "diagnostic_only": True}])
def test_display_does_not_present_failed_or_partial_results_as_complete(tmp_path, report):
    (tmp_path/"result.json").write_text(json.dumps(report))
    with pytest.raises(ValueError, match="complete route"):
        module().read_result(tmp_path)
