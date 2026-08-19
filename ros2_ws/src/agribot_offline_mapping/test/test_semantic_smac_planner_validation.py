import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = (
    PACKAGE_ROOT / "launch" / "semantic_smac_planner_validation.launch.py"
)
SPEC = importlib.util.spec_from_file_location(
    "semantic_smac_planner_validation", LAUNCH_FILE
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_contracts(tmp_path):
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\nresolution: 0.05\n", encoding="utf-8")
    graph_path = tmp_path / "graph.json"
    graph = {"schema_version": 3, "frame_id": "map"}
    graph_path.write_text(json.dumps(graph), encoding="utf-8")
    route_path = tmp_path / "route.json"
    route = {
        "schema_version": 3,
        "frame_id": "map",
        "graph_sha256": hashlib.sha256(graph_path.read_bytes()).hexdigest(),
        "execution_policy": {
            "preview_only": True,
            "execution_authorized": False,
            "requires_nav2_path_planning": True,
        },
    }
    route_path.write_text(json.dumps(route), encoding="utf-8")
    return map_path, graph_path, route_path


def launch_context(map_path, graph_path, route_path):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "map": str(map_path),
            "navigation_graph": str(graph_path),
            "route_plan": str(route_path),
            "show_3d_map": "false",
            "pcd_map": "",
        }
    )
    return context


def test_unified_launch_rejects_route_from_another_graph(tmp_path):
    map_path, graph_path, route_path = write_contracts(tmp_path)
    context = launch_context(map_path, graph_path, route_path)

    assert MODULE._validate_inputs(context) == []

    route = json.loads(route_path.read_text(encoding="utf-8"))
    route["graph_sha256"] = "0" * 64
    route_path.write_text(json.dumps(route), encoding="utf-8")
    with pytest.raises(RuntimeError, match="different navigation graph"):
        MODULE._validate_inputs(context)


def test_unified_launch_combines_semantics_and_smac_without_motion_nodes():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "ackermann_smac_planner_validation.launch.py" in source
    assert '"route_plan": LaunchConfiguration("route_plan")' in source
    assert 'executable="publish_semantic_navigation_graph.py"' in source
    assert 'executable="publish_semantic_route.py"' in source
    assert 'executable="publish_semantic_route_costmap.py"' in source
    assert 'default_value="semantic_stops"' in source
    assert '"rviz": "false"' in source
    assert "GroupAction(" in source
    assert "scoped=True" in source
    for forbidden in (
        "nav2_controller",
        "bt_navigator",
        "ackermann_chassis",
        "cmd_vel",
    ):
        assert forbidden not in source


def test_unified_rviz_shows_both_routes_and_semantic_avoidance():
    rviz_path = (
        PACKAGE_ROOT / "rviz" / "semantic_smac_planner_validation.rviz"
    )
    document = rviz_path.read_text(encoding="utf-8")
    parsed = yaml.safe_load(document)

    assert parsed["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert "Value: /semantic_navigation/topology_markers" in document
    assert "Value: /semantic_navigation/route_preview" in document
    assert "Value: /semantic_navigation/route_preview_markers" in document
    assert "route_avoidance_zones: true" in document
    assert "Value: /planning_test/path" in document
    assert "Value: /planning_test/waypoints" in document
    assert "rviz_default_plugins/SetInitialPose" in document
    assert "rviz_default_plugins/SetGoal" in document
