import importlib.util
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "build_semantic_model_context.py"
SPEC = importlib.util.spec_from_file_location("build_semantic_model_context", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def graph_document():
    return {
        "schema_version": 3,
        "frame_id": "map",
        "places": [
            {
                "id": "place_000",
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 1.0,
                "road_semantic_ids": [10],
                "semantic_summary": ["a metal gate"],
                "landmark_ids": ["landmark_gate"],
            },
            {
                "id": "place_001",
                "position": {"x": 4.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 1.0,
                "road_semantic_ids": [11],
                "semantic_summary": ["a building"],
                "landmark_ids": [],
            },
        ],
        "landmarks": [
            {
                "id": "landmark_gate",
                "caption": "a metal gate",
                "category": "fence",
                "num_detections": 20,
                "caption_consensus_ratio": 0.8,
                "position": {"x": 0.0, "y": 1.2, "z": 1.0},
                "nearest_place": "place_000",
                "distance_to_place_m": 1.2,
            },
            {
                "id": "landmark_tree",
                "caption": "a tree",
                "category": "vegetation",
                "num_detections": 10,
                "caption_consensus_ratio": 0.7,
                "position": {"x": 4.0, "y": 2.0, "z": 1.0},
                "nearest_place": "place_001",
                "distance_to_place_m": 2.0,
            },
        ],
        "connections": [
            {
                "id": "connection_000",
                "kind": "drivable",
                "source": "place_000",
                "target": "place_001",
                "length_m": 4.0,
                "minimum_clearance_m": 0.8,
                "road_semantic_coverage_ratio": 1.0,
                "bidirectional": True,
                "executable": True,
            },
            {
                "id": "landmark_connection_gate",
                "kind": "semantic_association",
                "source": "place_000",
                "target": "landmark_gate",
                "length_m": 1.2,
                "bidirectional": True,
                "executable": False,
            },
            {
                "id": "landmark_connection_tree",
                "kind": "semantic_association",
                "source": "place_001",
                "target": "landmark_tree",
                "length_m": 2.0,
                "bidirectional": True,
                "executable": False,
            },
        ],
    }


def test_exports_all_places_landmarks_connections_and_contract():
    context = MODULE.build_model_context(graph_document(), "a" * 64)

    assert context["schema_version"] == 3
    assert context["graph_sha256"] == "a" * 64
    assert context["statistics"] == {
        "semantic_nodes": 4,
        "places": 2,
        "landmarks": 2,
        "connections": 3,
        "drivable_connections": 1,
        "semantic_associations": 2,
    }
    assert context["allowed_destination_node_ids"] == [
        "place_000",
        "place_001",
    ]
    assert [item["id"] for item in context["landmarks"]] == [
        "landmark_gate",
        "landmark_tree",
    ]
    assert context["landmarks"][0]["nearest_place"] == "place_000"
    assert context["landmarks"][0]["navigation_policy"] == (
        "model_must_output_nearest_place"
    )
    assert context["connections"][0]["road_semantic_coverage_ratio"] == 1.0
    assert context["connections"][1]["executable"] is False
    contract = context["planning_contract"]
    assert contract["allowed_output_node_types"] == ["place"]
    assert contract["landmarks_are_context_only"] is True
    assert contract["model_maps_landmarks_to_nearest_place"] is True
    assert contract["dijkstra_searches_drivable_places_only"] is True
    assert "user-requested visit order" in contract["destination_order"]
    assert context["planning_contract"]["direct_motion_commands_forbidden"] is True


def test_no_landmark_is_truncated_from_model_context():
    context = MODULE.build_model_context(graph_document(), "b" * 64)

    assert len(context["landmarks"]) == len(graph_document()["landmarks"])
    assert {item["id"] for item in context["landmarks"]} == {
        "landmark_gate",
        "landmark_tree",
    }
    assert not set(context["allowed_destination_node_ids"]).intersection(
        {"landmark_gate", "landmark_tree"}
    )
