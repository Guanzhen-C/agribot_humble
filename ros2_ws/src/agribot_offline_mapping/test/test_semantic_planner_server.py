import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "semantic_planner_server.py"
SPEC = importlib.util.spec_from_file_location("semantic_planner_server", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def route_document(graph_digest, provider="alibaba_cloud_bailian"):
    return {
        "schema_version": 3,
        "frame_id": "map",
        "graph_sha256": graph_digest,
        "execution_policy": {"preview_only": True},
        "route_id": "route_test",
        "task_plan": {
            "task_id": "phone_test",
            "avoid_node_ids": [],
        },
        "route": {
            "poses": [
                {"place_id": "place_000", "position": {"x": 0.0, "y": 0.0}, "yaw": 0.0},
                {"place_id": "place_001", "position": {"x": 2.0, "y": 1.0}, "yaw": 0.2},
            ]
        },
        "resolved_stops": [
            {"place_id": "place_000"},
            {"place_id": "place_001", "name": "白色建筑", "semantic_summary": ["入口"]},
        ],
        "statistics": {
            "route_navigation_places": 2,
            "drivable_route_length_m": 2.3,
            "search_algorithm": "astar_euclidean_admissible",
            "astar_cost_m": 2.3,
        },
        "model_provenance": {"provider": provider, "model": "qwen3.7-flash"},
    }


def test_route_boundary_accepts_only_bailian_and_current_graph(tmp_path):
    graph = tmp_path / "graph.json"
    graph.write_text('{"schema_version":3}', encoding="utf-8")
    digest = hashlib.sha256(graph.read_bytes()).hexdigest()

    result = MODULE.validate_route_document(route_document(digest), graph, "map_test")
    assert result["provider"] == "alibaba_cloud_bailian"
    assert result["model"] == "qwen3.7-flash"
    assert result["route"][1]["place_id"] == "place_001"
    assert result["execution_allowed"] is True
    assert result["statistics"]["search_algorithm"] == "astar_euclidean_admissible"

    with pytest.raises(MODULE.SemanticServiceError, match="阿里百炼"):
        MODULE.validate_route_document(
            route_document(digest, provider="ollama_local"), graph, "map_test"
        )
    with pytest.raises(MODULE.SemanticServiceError, match="过期图谱"):
        MODULE.validate_route_document(route_document("0" * 64), graph, "map_test")


def test_service_configuration_keeps_secrets_server_side(tmp_path, monkeypatch):
    graph = tmp_path / "graph.json"
    graph.write_text('{"schema_version":3}', encoding="utf-8")
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
    "field,value,error",
    [
        (
            "statistics",
            {"route_navigation_places": 2.5, "drivable_route_length_m": 2.3},
            "拓扑点数量",
        ),
        (
            "statistics",
            {"route_navigation_places": 2, "drivable_route_length_m": -1.0},
            "路线长度",
        ),
        (
            "resolved_stops",
            [
                {"place_id": "place_000"},
                {
                    "place_id": "place_001",
                    "semantic_summary": "不是列表",
                },
            ],
            "目的地描述",
        ),
    ],
)
def test_route_boundary_rejects_malformed_server_output(tmp_path, field, value, error):
    graph = tmp_path / "graph.json"
    graph.write_text('{"schema_version":3}', encoding="utf-8")
    digest = hashlib.sha256(graph.read_bytes()).hexdigest()
    document = route_document(digest)
    document[field] = value

    with pytest.raises(MODULE.SemanticServiceError, match=error):
        MODULE.validate_route_document(document, graph, "map_test")
