import importlib.util
import json
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "plan_semantic_task_bailian.py"
SPEC = importlib.util.spec_from_file_location("plan_semantic_task_bailian", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


GRAPH_DIGEST = "a" * 64


def graph_document():
    return {
        "schema_version": 3,
        "frame_id": "map",
        "landmarks": [
            {
                "id": "landmark_gate",
                "caption": "a metal gate",
                "category": "fence",
                "num_detections": 20,
                "caption_consensus_ratio": 0.8,
                "distance_to_route_m": 1.2,
                "position": {"x": 0.0, "y": 1.2, "z": 1.0},
                "nearest_place": "place_000",
                "distance_to_place_m": 1.2,
            }
        ],
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
        ],
    }


def intent_plan(**updates):
    plan = {
        "schema_version": 1,
        "task_id": "inspection_001",
        "graph_sha256": GRAPH_DIGEST,
        "destination_queries": ["白色建筑"],
        "avoid_queries": [],
    }
    plan.update(updates)
    return plan


def task_plan(**updates):
    plan = {
        "schema_version": 3,
        "task_id": "inspection_001",
        "graph_sha256": GRAPH_DIGEST,
        "destination_node_ids": ["place_001"],
        "avoid_node_ids": [],
    }
    plan.update(updates)
    return plan


def api_response(document=None):
    return {
        "id": "chatcmpl-test",
        "choices": [
            {
                "index": 0,
                "message": {
                    "role": "assistant",
                    "content": json.dumps(document or task_plan()),
                },
                "finish_reason": "stop",
            }
        ],
        "usage": {
            "prompt_tokens": 100,
            "completion_tokens": 20,
            "total_tokens": 120,
        },
    }


def candidate(place_id, caption, landmark_id):
    return {
        "place_id": place_id,
        "hybrid_score": 0.032,
        "semantic_evidence": [
            {
                "landmark_id": landmark_id,
                "caption": caption,
                "category": "object",
                "distance_to_place_m": 1.0,
                "retrieval_sources": ["vector", "fulltext"],
            }
        ],
    }


def retrieval_context(destination_requests=None, avoidance_requests=None):
    if destination_requests is None:
        destination_requests = [
            {
                "query_index": 0,
                "semantic_query": "白色建筑",
                "candidate_places": [
                    candidate("place_001", "白色建筑", "landmark_building"),
                    candidate("place_000", "金属门", "landmark_gate"),
                ],
            }
        ]
    return {
        "schema_version": 1,
        "context_type": "agribot_neo4j_retrieved_place_candidates",
        "map_id": "test_map",
        "graph_sha256": GRAPH_DIGEST,
        "retrieval_policy": {
            "source": "neo4j_hybrid_landmark_retrieval",
            "landmark_anchor_relationship": "NEAREST_PLACE",
            "candidate_limit_per_query": 5,
            "destination_order_matches_query_index": True,
            "contains_full_graph": False,
        },
        "destination_requests": destination_requests,
        "avoidance_requests": avoidance_requests or [],
    }


def context_and_start():
    document = graph_document()
    graph = MODULE.SemanticRouteGraph(document)
    selector, start = MODULE.resolve_start(graph, "place_000", None, 5.0)
    return document, graph, selector, start


def test_intent_request_contains_no_map_and_preserves_order_contract():
    request = MODULE.build_intent_request(
        MODULE.DEFAULT_MODEL,
        "先去白楼，再去金属门",
        "inspection_001",
        GRAPH_DIGEST,
    )

    serialized = json.dumps(request, ensure_ascii=False)
    assert request["response_format"] == {"type": "json_object"}
    assert request["enable_thinking"] is False
    assert request["temperature"] == 0.0
    assert "严格保留用户要求的访问顺序" in request["messages"][0]["content"]
    assert "简洁中文语义检索短语" in request["messages"][0]["content"]
    assert "不得翻译成英文" in request["messages"][0]["content"]
    assert "destination_queries" in serialized
    assert "place_000" not in serialized
    assert "landmark_gate" not in serialized


def test_intent_plan_is_strictly_validated():
    assert MODULE.validate_intent_plan(
        intent_plan(), "inspection_001", GRAPH_DIGEST
    )["destination_queries"] == ["白色建筑"]

    with pytest.raises(MODULE.BailianPlanningError, match="unsupported fields"):
        MODULE.validate_intent_plan(
            intent_plan(speed_mps=0.5), "inspection_001", GRAPH_DIGEST
        )
    with pytest.raises(MODULE.BailianPlanningError, match="changed the requested"):
        MODULE.validate_intent_plan(
            intent_plan(task_id="other"), "inspection_001", GRAPH_DIGEST
        )


def test_retrieval_context_contains_only_top_candidates(monkeypatch):
    calls = []

    def fake_retrieve(client, map_id, query, embedding, top_k):
        calls.append((client, map_id, query, embedding, top_k))
        return [
            {
                "place_id": "place_001",
                "position": {"x": 4.0, "y": 0.0},
                "hybrid_score": 0.032786885,
                "evidence_landmarks": [
                    {
                        "landmark_id": "landmark_building",
                        "caption": "白色建筑",
                        "category": "建筑",
                        "distance_to_place_m": 0.75,
                        "retrieval_sources": [
                            {"source": "vector", "rank": 1, "source_score": 0.9}
                        ],
                    }
                ],
            }
        ]

    monkeypatch.setattr(MODULE, "retrieve_place_candidates", fake_retrieve)
    context = MODULE.build_retrieval_context(
        object(), "test_map", intent_plan(), [[0.1, 0.2]], [], 3
    )

    assert calls[0][1:] == ("test_map", "白色建筑", [0.1, 0.2], 3)
    assert context["retrieval_policy"]["contains_full_graph"] is False
    assert context["destination_requests"][0]["candidate_places"][0][
        "place_id"
    ] == "place_001"
    serialized = json.dumps(context)
    assert '"places"' not in serialized
    assert '"connections"' not in serialized
    assert '"position"' not in serialized


def test_selection_request_uses_only_per_query_neo4j_candidates():
    _, _, _, start = context_and_start()
    request = MODULE.build_bailian_request(
        MODULE.DEFAULT_MODEL,
        retrieval_context(),
        "去白色建筑物附近巡检",
        "inspection_001",
        start,
    )

    serialized = request["messages"][1]["content"]
    assert request["response_format"] == {"type": "json_object"}
    assert "每个语义查询恰好选择一个" in request["messages"][0]["content"]
    assert "同索引candidate_places" in request["messages"][0]["content"]
    assert "Neo4j" in request["messages"][0]["content"]
    assert "place_001" in serialized
    assert "landmark_building" in serialized
    assert '"connections"' not in serialized
    assert '"places"' not in serialized


def test_validated_bailian_plan_creates_preview_only_route():
    document, graph, selector, start = context_and_start()
    context = retrieval_context()
    request = MODULE.build_bailian_request(
        MODULE.DEFAULT_MODEL,
        context,
        "去白色建筑物附近巡检",
        "inspection_001",
        start,
    )
    response = api_response()
    plan = MODULE.validated_model_plan(
        response, GRAPH_DIGEST, graph, "inspection_001", context
    )
    route = MODULE.create_route_preview(
        document,
        GRAPH_DIGEST,
        plan,
        selector,
        start,
        0.2,
        MODULE.DEFAULT_MODEL,
        request,
        MODULE.response_usage(response),
        map_id="test_map",
    )

    assert route["request"]["goal"] == "place_001"
    assert route["statistics"]["drivable_route_length_m"] == 4.0
    assert route["execution_policy"]["preview_only"] is True
    assert route["execution_policy"]["execution_authorized"] is False
    assert route["model_provenance"]["provider"] == "alibaba_cloud_bailian"
    assert route["model_provenance"]["model"] == "qwen3.7-flash"
    assert route["model_provenance"]["selection_usage"]["total_tokens"] == 120
    assert route["model_provenance"]["neo4j_map_id"] == "test_map"


def test_destination_order_is_enforced_per_retrieval_query():
    _, graph, _, _ = context_and_start()
    context = retrieval_context(
        [
            {
                "query_index": 0,
                "semantic_query": "building",
                "candidate_places": [
                    candidate("place_001", "a building", "landmark_building")
                ],
            },
            {
                "query_index": 1,
                "semantic_query": "gate",
                "candidate_places": [
                    candidate("place_000", "a gate", "landmark_gate")
                ],
            },
        ]
    )
    swapped = task_plan(destination_node_ids=["place_000", "place_001"])

    with pytest.raises(MODULE.BailianPlanningError, match="semantic query 0"):
        MODULE.validated_model_plan(
            api_response(swapped), GRAPH_DIGEST, graph, "inspection_001", context
        )


def test_every_avoidance_query_requires_a_retrieved_candidate():
    context = retrieval_context(
        avoidance_requests=[
            {
                "query_index": 0,
                "semantic_query": "blue bicycle",
                "candidate_places": [
                    candidate("place_000", "a blue bicycle", "landmark_bicycle")
                ],
            }
        ]
    )
    with pytest.raises(MODULE.BailianPlanningError, match="did not select"):
        MODULE.validate_plan_against_retrieval(task_plan(), context)

    selected = task_plan(avoid_node_ids=["place_000"])
    assert MODULE.validate_plan_against_retrieval(selected, context) is selected


@pytest.mark.parametrize(
    "invalid_plan",
    [
        task_plan(speed_mps=0.5),
        task_plan(graph_sha256="b" * 64),
        task_plan(destination_node_ids=["landmark_gate"]),
        task_plan(destination_node_ids=["place_999"]),
        task_plan(destination_node_ids=["place_001", "place_001"]),
        task_plan(avoid_node_ids=["landmark_gate"]),
        task_plan(avoid_node_ids=["place_999"]),
        task_plan(avoid_node_ids=["place_001"]),
    ],
)
def test_model_output_must_pass_existing_task_plan_contract(invalid_plan):
    _, graph, _, _ = context_and_start()

    with pytest.raises(MODULE.RoutePlanningError):
        MODULE.validated_model_plan(
            api_response(invalid_plan), GRAPH_DIGEST, graph, "inspection_001"
        )


def test_model_cannot_change_task_id():
    _, graph, _, _ = context_and_start()

    with pytest.raises(MODULE.BailianPlanningError, match="changed the requested"):
        MODULE.validated_model_plan(
            api_response(task_plan(task_id="other")),
            GRAPH_DIGEST,
            graph,
            "inspection_001",
        )


def test_markdown_or_duplicate_json_keys_are_rejected():
    fenced = api_response()
    fenced["choices"][0]["message"]["content"] = "```json\n{}\n```"
    duplicate = api_response()
    duplicate["choices"][0]["message"]["content"] = (
        '{"schema_version":3,"schema_version":3}'
    )

    with pytest.raises(MODULE.BailianPlanningError):
        MODULE.extract_task_plan(fenced)
    with pytest.raises(MODULE.BailianPlanningError, match="duplicate JSON key"):
        MODULE.extract_task_plan(duplicate)


def test_api_key_is_required_and_not_accepted_in_base_url():
    with pytest.raises(MODULE.BailianPlanningError, match="DASHSCOPE_API_KEY"):
        MODULE.call_bailian({}, "", MODULE.DEFAULT_BASE_URL, 1.0)
    with pytest.raises(MODULE.BailianPlanningError, match="without credentials"):
        MODULE.validate_base_url("https://secret@example.com/v1")


def test_start_position_is_bounded_by_semantic_place_graph():
    graph = MODULE.SemanticRouteGraph(graph_document())
    selector, start = MODULE.resolve_start(graph, None, (0.1, 0.0), 0.5)

    assert selector == "place_000"
    assert start["source"] == "nearest_semantic_place"
    assert start["distance_m"] == pytest.approx(0.1)
    with pytest.raises(MODULE.RoutePlanningError, match="exceeding"):
        MODULE.resolve_start(graph, None, (20.0, 20.0), 1.0)
