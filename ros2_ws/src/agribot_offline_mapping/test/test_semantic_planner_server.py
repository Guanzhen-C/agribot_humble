import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "semantic_planner_server.py"
SPEC = importlib.util.spec_from_file_location("semantic_planner_server", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def graph_document():
    return {
        "schema_version": 3,
        "frame_id": "map",
        "places": [
            {
                "id": "place_000",
                "name": "起点",
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "semantic_summary": ["道路"],
            },
            {
                "id": "place_001",
                "name": "白色建筑",
                "position": {"x": 2.0, "y": 1.0, "z": 0.0},
                "yaw": 0.2,
                "semantic_summary": ["入口"],
            },
        ],
    }


def task_document(graph_digest, **updates):
    document = {
        "schema_version": 3,
        "task_id": "phone_test",
        "graph_sha256": graph_digest,
        "destination_node_ids": ["place_001"],
        "avoid_node_ids": [],
    }
    document.update(updates)
    return document


def write_graph(tmp_path):
    graph = tmp_path / "graph.json"
    graph.write_text(json.dumps(graph_document()), encoding="utf-8")
    return graph, hashlib.sha256(graph.read_bytes()).hexdigest()


def test_task_boundary_returns_only_targets_and_keepouts(tmp_path):
    graph, digest = write_graph(tmp_path)
    result = MODULE.validate_task_document(
        task_document(digest, avoid_node_ids=["place_000"]),
        graph,
        "map_test",
        2.0,
        0.5,
    )

    assert result["provider"] == "alibaba_cloud_bailian"
    assert result["model"] == "qwen3.7-flash"
    assert result["destination_poses"][0]["place_id"] == "place_001"
    assert result["destination_poses"][0]["yaw"] == 0.2
    assert result["avoidance_zones"][0]["selector"] == "place_000"
    assert result["avoidance_zones"][0]["influence_radius_m"] == 2.0
    assert result["avoidance_zones"][0]["decay_length_m"] == 0.5
    assert "route" not in result
    assert "route_centerline" not in result
    assert result["costmap_policy"] == {
        "semantic_route_preference_enabled": False,
        "semantic_avoidance_is_lethal": False,
        "semantic_proximity_cost_model": "exponential",
        "requires_nav2_proximity_layer": True,
    }
    assert result["statistics"]["path_planner"] == "nav2_smac_hybrid"

    with pytest.raises(MODULE.SemanticServiceError, match="过期图谱"):
        MODULE.validate_task_document(
            task_document("0" * 64), graph, "map_test", 2.0, 0.5
        )


def test_service_configuration_keeps_secrets_server_side(tmp_path, monkeypatch):
    graph, _ = write_graph(tmp_path)
    planner = tmp_path / "planner.py"
    planner.write_text("pass\n", encoding="utf-8")
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            {
                "planner_script": str(planner),
                "work_root": str(tmp_path / "tasks"),
                "model": "qwen3.7-flash",
                "embedding_model": "text-embedding-v4",
                "embedding_dimensions": 1024,
                "maps": {
                    "map_test": {
                        "graph": str(graph),
                        "neo4j_http_uri": "http://127.0.0.1:7476",
                        "neo4j_password_env": "TEST_NEO4J_PASSWORD",
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("TEST_NEO4J_PASSWORD", "server-only-secret")
    monkeypatch.setenv("DASHSCOPE_API_KEY", "server-only-api-key")
    service = MODULE.SemanticPlannerService(config)

    public = service.public_maps()
    assert public[0]["map_id"] == "map_test"
    assert "password" not in json.dumps(public).lower()
    assert "server-only-secret" not in json.dumps(public)


@pytest.mark.parametrize(
    "updates,error",
    [
        ({"destination_node_ids": []}, "目的地"),
        ({"destination_node_ids": ["place_999"]}, "目的地"),
        ({"avoid_node_ids": ["place_999"]}, "避让"),
        (
            {
                "destination_node_ids": ["place_001"],
                "avoid_node_ids": ["place_001"],
            },
            "同时作为",
        ),
        ({"extra": True}, "格式"),
    ],
)
def test_task_boundary_rejects_malformed_output(tmp_path, updates, error):
    graph, digest = write_graph(tmp_path)
    with pytest.raises(MODULE.SemanticServiceError, match=error):
        MODULE.validate_task_document(
            task_document(digest, **updates), graph, "map_test", 2.0, 0.5
        )
