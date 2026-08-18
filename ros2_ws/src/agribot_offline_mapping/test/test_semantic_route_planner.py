import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "plan_semantic_route.py"
SPEC = importlib.util.spec_from_file_location("plan_semantic_route", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def place(place_id, x, y, road=True):
    return {
        "id": place_id,
        "position": {"x": x, "y": y, "z": 0.0},
        "yaw": 0.0,
        "clearance_m": 0.8,
        "road_semantic_ids": [1] if road else [],
        "semantic_summary": [],
        "landmark_ids": [],
    }


def landmark(landmark_id, x, y, nearest_place, distance):
    return {
        "id": landmark_id,
        "position": {"x": x, "y": y, "z": 0.5},
        "caption": landmark_id.replace("_", " "),
        "category": "object",
        "num_detections": 20,
        "caption_consensus_ratio": 0.9,
        "nearest_place": nearest_place,
        "distance_to_place_m": distance,
    }


def drivable(connection_id, source, target, length, clearance=0.5, coverage=1.0):
    return {
        "id": connection_id,
        "kind": "drivable",
        "source": source,
        "target": target,
        "length_m": length,
        "minimum_clearance_m": clearance,
        "road_semantic_coverage_ratio": coverage,
        "bidirectional": True,
        "executable": True,
    }


def association(connection_id, place_id, landmark_id, distance):
    return {
        "id": connection_id,
        "kind": "semantic_association",
        "source": place_id,
        "target": landmark_id,
        "length_m": distance,
        "bidirectional": True,
        "executable": False,
    }


def graph_document():
    return {
        "schema_version": 3,
        "frame_id": "map",
        "parameters": {"minimum_edge_clearance_m": 0.2},
        "places": [
            place("place_start", 0.0, 0.0),
            place("place_via", 1.0, 0.0),
            place("place_goal", 2.0, 0.0),
            place("place_detour", 1.0, 2.0, road=False),
        ],
        "landmarks": [
            landmark("landmark_gate", 1.0, 0.2, "place_via", 0.2),
            landmark("landmark_building", 2.2, 0.0, "place_goal", 0.2),
        ],
        "connections": [
            drivable("connection_ab", "place_start", "place_via", 1.0),
            drivable("connection_bc", "place_via", "place_goal", 1.0),
            drivable(
                "connection_ad", "place_start", "place_detour", 2.3, coverage=0.5
            ),
            drivable(
                "connection_dc", "place_detour", "place_goal", 2.3, coverage=0.5
            ),
            association(
                "landmark_connection_gate", "place_via", "landmark_gate", 0.2
            ),
            association(
                "landmark_connection_building",
                "place_goal",
                "landmark_building",
                0.2,
            ),
        ],
    }


def test_plans_shortest_route_through_requested_places():
    result = MODULE.plan_route(
        graph_document(),
        ["place_start", "place_via", "place_goal"],
        None,
        "abc123",
    )

    assert result["request"]["via"] == ["place_via"]
    assert result["route"]["semantic_node_ids"] == [
        "place_start",
        "place_via",
        "place_goal",
    ]
    assert result["route"]["navigation_place_ids"] == [
        "place_start",
        "place_via",
        "place_goal",
    ]
    assert result["statistics"]["drivable_route_length_m"] == 2.0
    assert result["statistics"]["dijkstra_cost_m"] == 2.0
    assert result["statistics"]["minimum_route_clearance_m"] == 0.5
    assert result["statistics"]["road_semantic_coverage_ratio"] == 1.0
    assert result["execution_policy"]["preview_only"] is True
    assert result["execution_policy"]["execution_authorized"] is False


def test_dijkstra_uses_only_drivable_places_and_rejects_landmark_targets():
    graph = MODULE.SemanticRouteGraph(graph_document())

    assert set(graph.adjacency) == set(graph.places)
    assert all(
        connection_id.startswith("connection_")
        for neighbors in graph.adjacency.values()
        for _, _, connection_id, _, _ in neighbors
    )
    with pytest.raises(MODULE.RoutePlanningError, match="Dijkstra endpoints"):
        graph.shortest_path("place_start", "landmark_building", 0.0)
    with pytest.raises(MODULE.RoutePlanningError, match="drivable semantic places"):
        MODULE.plan_route(
            graph_document(),
            ["place_start", "landmark_building"],
            None,
            "abc123",
        )


def test_clearance_gate_changes_route_and_unknown_node_is_rejected():
    document = graph_document()
    document["connections"][1]["minimum_clearance_m"] = 0.1
    result = MODULE.plan_route(
        document, ["place_start", "place_goal"], 0.2, "abc123"
    )

    assert result["route"]["navigation_place_ids"] == [
        "place_start",
        "place_detour",
        "place_goal",
    ]
    assert result["statistics"]["drivable_route_length_m"] == 4.6
    assert result["statistics"]["road_semantic_coverage_ratio"] == 0.5

    with pytest.raises(MODULE.RoutePlanningError, match="unknown semantic node"):
        MODULE.plan_route(document, ["place_start", "missing"], 0.0)


def test_rejects_broken_or_legacy_graph_contract():
    document = graph_document()
    document["connections"][0]["target"] = "missing"
    with pytest.raises(MODULE.GraphValidationError, match="unknown semantic node"):
        MODULE.SemanticRouteGraph(document)

    document = graph_document()
    document["nodes"] = []
    with pytest.raises(MODULE.GraphValidationError, match="legacy route fields"):
        MODULE.SemanticRouteGraph(document)

    document = graph_document()
    document["places"][0]["route_node"] = "route_0000"
    with pytest.raises(MODULE.GraphValidationError, match="legacy route node"):
        MODULE.SemanticRouteGraph(document)


def test_each_landmark_requires_exactly_one_matching_nearest_place_connection():
    document = graph_document()
    document["connections"].pop()
    with pytest.raises(MODULE.GraphValidationError, match="exactly one"):
        MODULE.SemanticRouteGraph(document)

    document = graph_document()
    document["connections"][-1]["source"] = "place_via"
    with pytest.raises(MODULE.GraphValidationError, match="nearest-place policy"):
        MODULE.SemanticRouteGraph(document)


def test_route_output_round_trip_is_json_serializable(tmp_path):
    result = MODULE.plan_route(
        graph_document(), ["place_start", "place_goal"], None, "abc123"
    )
    output = tmp_path / "route.json"
    output.write_text(json.dumps(result), encoding="utf-8")
    loaded = json.loads(output.read_text(encoding="utf-8"))

    assert loaded["schema_version"] == 3
    assert loaded["route_id"] == result["route_id"]
    assert loaded["frame_id"] == "map"
    assert "node_ids" not in loaded["route"]
    assert "edge_ids" not in loaded["route"]


def test_nearest_start_uses_places_only_and_is_bounded():
    graph = MODULE.SemanticRouteGraph(graph_document())
    place_id, distance = graph.nearest_place(0.8, 0.1, 1.0)

    assert place_id == "place_via"
    assert distance == pytest.approx(0.2236067977)
    with pytest.raises(MODULE.RoutePlanningError, match="exceeding"):
        graph.nearest_place(20.0, 20.0, 1.0)
    with pytest.raises(MODULE.RoutePlanningError, match="drivable semantic places"):
        MODULE.plan_route(
            graph_document(), ["landmark_gate", "place_goal"], None, "abc123"
        )


def test_task_plan_accepts_places_only_and_rejects_stale_map():
    graph_digest = "a" * 64
    graph = MODULE.SemanticRouteGraph(graph_document())
    document = {
        "schema_version": 3,
        "task_id": "inspect-east-building",
        "graph_sha256": graph_digest,
        "destination_node_ids": ["place_via", "place_goal"],
    }
    validated = MODULE.validate_task_plan(document, graph_digest, graph)
    result = MODULE.plan_route(
        graph_document(),
        ["place_start"] + validated["destination_node_ids"],
        None,
        graph_digest,
        validated,
    )

    assert result["task_plan"]["task_id"] == "inspect-east-building"
    assert result["request"]["start"] == "place_start"
    assert result["request"]["goal"] == "place_goal"
    assert result["task_plan"]["avoid_node_ids"] == []
    assert result["route"]["semantic_node_ids"] == [
        "place_start",
        "place_via",
        "place_goal",
    ]

    document["graph_sha256"] = "b" * 64
    with pytest.raises(MODULE.RoutePlanningError, match="stale"):
        MODULE.validate_task_plan(document, graph_digest, graph)


def test_task_plan_cannot_inject_commands_or_unknown_ids():
    graph_digest = "a" * 64
    graph = MODULE.SemanticRouteGraph(graph_document())
    with pytest.raises(MODULE.RoutePlanningError, match="unsupported fields"):
        MODULE.validate_task_plan(
            {
                "schema_version": 3,
                "task_id": "unsafe",
                "graph_sha256": graph_digest,
                "destination_node_ids": ["place_goal"],
                "speed": 1.0,
            },
            graph_digest,
            graph,
        )
    with pytest.raises(MODULE.RoutePlanningError, match="destination place"):
        MODULE.validate_task_plan(
            {
                "schema_version": 3,
                "task_id": "wrong-id",
                "graph_sha256": graph_digest,
                "destination_node_ids": ["route_c"],
            },
            graph_digest,
            graph,
        )


def test_avoidance_zone_forces_alternate_route_and_is_auditable():
    result = MODULE.plan_route(
        graph_document(),
        ["place_start", "place_goal"],
        None,
        "abc123",
        None,
        ["place_via"],
        0.10,
    )

    assert result["route"]["navigation_place_ids"] == [
        "place_start",
        "place_detour",
        "place_goal",
    ]
    assert "place_via" not in result["route"]["semantic_node_ids"]
    assert result["request"]["avoid_node_ids"] == ["place_via"]
    assert result["avoidance_constraints"]["radius_m"] == 0.10
    assert "place_via" in result["avoidance_constraints"]["blocked_node_ids"]
    assert set(result["avoidance_constraints"]["blocked_connection_ids"]) == {
        "connection_ab",
        "connection_bc",
    }
    assert result["execution_policy"]["requires_nav2_keepout_enforcement"] is True


def test_avoidance_rejects_destination_or_start_inside_forbidden_zone():
    with pytest.raises(MODULE.RoutePlanningError, match="both destinations"):
        MODULE.plan_route(
            graph_document(),
            ["place_start", "place_goal"],
            None,
            "abc123",
            None,
            ["place_goal"],
            0.10,
        )

    with pytest.raises(MODULE.RoutePlanningError, match="inside.*avoidance"):
        MODULE.plan_route(
            graph_document(),
            ["place_start", "place_goal"],
            None,
            "abc123",
            None,
            ["place_via"],
            1.10,
        )


def test_task_plan_validates_avoidance_ids_and_disjointness():
    graph_digest = "a" * 64
    graph = MODULE.SemanticRouteGraph(graph_document())
    base = {
        "schema_version": 3,
        "task_id": "avoid-test",
        "graph_sha256": graph_digest,
        "destination_node_ids": ["place_goal"],
    }
    valid = dict(base, avoid_node_ids=["place_via"])
    assert MODULE.validate_task_plan(valid, graph_digest, graph)[
        "avoid_node_ids"
    ] == ["place_via"]

    with pytest.raises(MODULE.RoutePlanningError, match="avoidance place"):
        MODULE.validate_task_plan(
            dict(base, avoid_node_ids=["landmark_gate"]), graph_digest, graph
        )
    with pytest.raises(MODULE.RoutePlanningError, match="both destination"):
        MODULE.validate_task_plan(
            dict(base, avoid_node_ids=["place_goal"]), graph_digest, graph
        )
