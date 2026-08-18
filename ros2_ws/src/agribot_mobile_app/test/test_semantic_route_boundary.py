import threading

import pytest

from agribot_mobile_app.gateway_node import ApiError, MobileGateway


def semantic_document(**updates):
    document = {
        "map_id": "map_lio_sam_0811",
        "provider": "ollama_local",
        "model": "qwen3.8:27b",
        "graph_sha256": "e" * 64,
        "instruction": "先去白色建筑，再到北门",
        "route": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "place_id": "place_000"},
            {"x": 2.0, "y": 1.0, "yaw": 0.2, "place_id": "place_001"},
        ],
        "destinations": [
            {"place_id": "place_001", "name": "北门", "semantic_summary": ["白色门柱"]}
        ],
        "avoid_node_ids": [],
        "execution_allowed": True,
        "statistics": {"route_navigation_places": 2, "drivable_route_length_m": 2.3},
    }
    document.update(updates)
    return document


def gateway_stub():
    gateway = object.__new__(MobileGateway)
    gateway._lock = threading.RLock()
    gateway._state = {
        "active_runtime": {"map_id": "map_lio_sam_0811"},
        "semantic": {},
    }
    gateway.semantic_map_ids = {"map_lio_sam_0811"}
    gateway._assert_navigation_ready = lambda: None
    gateway._touch = lambda: None
    return gateway


def test_rdk_accepts_server_route_without_contacting_the_model():
    gateway = gateway_stub()
    result = gateway.receive_semantic_route(
        {"request_id": "phone_01", "semantic": semantic_document()}
    )
    assert result["semantic"]["route"][1]["place_id"] == "place_001"
    assert result["semantic"]["provider"] == "ollama_local"
    assert result["semantic"]["request_id"] == "phone_01"


@pytest.mark.parametrize(
    "semantic",
    [
        semantic_document(provider="alibaba_cloud_bailian"),
        semantic_document(model="qwen3.7-flash"),
        semantic_document(map_id="other_map"),
        semantic_document(graph_sha256="invalid"),
        semantic_document(route=[{"x": 0.0, "y": 0.0, "yaw": 0.0}]),
        semantic_document(
            destinations=[
                {
                    "place_id": "place_001",
                    "name": "北门",
                    "semantic_summary": "不是列表",
                }
            ]
        ),
        semantic_document(
            statistics={"route_navigation_places": 2.5, "drivable_route_length_m": 2.3}
        ),
        semantic_document(
            statistics={"route_navigation_places": 2, "drivable_route_length_m": -1.0}
        ),
    ],
)
def test_rdk_rejects_untrusted_or_stale_semantic_routes(semantic):
    with pytest.raises(ApiError):
        gateway_stub().receive_semantic_route({"semantic": semantic})
