import threading

import numpy as np
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.gateway_node import ApiError, MobileGateway


def semantic_document(**updates):
    document = {
        "map_id": "map_lio_sam_0811",
        "provider": "alibaba_cloud_bailian",
        "model": "qwen3.7-flash",
        "graph_sha256": "e" * 64,
        "instruction": "先去白色建筑，再到北门",
        "route": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "place_id": "place_000"},
            {"x": 2.0, "y": 1.0, "yaw": 0.2, "place_id": "place_001"},
        ],
        "route_centerline": [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 1.0}],
        "destination_poses": [
            {"x": 2.0, "y": 1.0, "yaw": 0.2, "place_id": "place_001"}
        ],
        "destinations": [
            {"place_id": "place_001", "name": "北门", "semantic_summary": ["白色门柱"]}
        ],
        "avoid_node_ids": [],
        "avoidance_zones": [],
        "execution_allowed": True,
        "costmap_policy": {
            "semantic_route_preference_enabled": True,
            "route_corridor_model": "wide_free_core_additive_outside",
            "route_corridor_half_width_m": 2.0,
            "route_corridor_transition_width_m": 1.0,
            "route_corridor_outside_cost": 100,
            "semantic_avoidance_is_lethal": False,
            "semantic_proximity_cost_model": "exponential_additive",
            "requires_nav2_proximity_layer": True,
        },
        "statistics": {
            "destination_count": 1,
            "avoidance_zone_count": 0,
            "path_planner": "nav2_smac_hybrid",
            "route_navigation_places": 2,
            "drivable_route_length_m": 2.2,
            "search_algorithm": "astar_euclidean_admissible",
            "astar_cost_m": 2.2,
        },
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
    gateway._grids = {
        "map": GridData(
            width=10,
            height=10,
            resolution=1.0,
            origin_x=-5.0,
            origin_y=-5.0,
            origin_yaw=0.0,
            data=bytes(100),
        )
    }
    gateway._semantic_costmap_source = None
    gateway.semantic_map_ids = {"map_lio_sam_0811"}
    gateway._assert_navigation_ready = lambda: None
    gateway._publish_semantic_costmap = lambda *_args: np.full(
        (10, 10), 100, dtype=np.uint8
    )
    gateway._touch = lambda: None
    return gateway


def test_rdk_accepts_astar_corridor_but_keeps_model_destinations():
    gateway = gateway_stub()
    result = gateway.receive_semantic_route(
        {"request_id": "phone_01", "semantic": semantic_document()}
    )

    semantic = result["semantic"]
    assert semantic["destination_poses"][0]["place_id"] == "place_001"
    assert semantic["provider"] == "alibaba_cloud_bailian"
    assert semantic["request_id"] == "phone_01"
    assert semantic["costmap_ready"] is False
    assert len(semantic["route"]) == 2
    assert len(semantic["route_centerline"]) == 2
    assert gateway._semantic_costmap_source["corridor_policy"].half_width_m == 2.0


def test_semantic_execution_sends_only_ordered_model_destinations():
    gateway = gateway_stub()
    gateway.receive_semantic_route({"semantic": semantic_document()})
    gateway._state["semantic"]["costmap_ready"] = True
    sent = []
    gateway._send_navigation_route = lambda route, kind: sent.append(
        (route, kind)
    ) or {"poses": route}

    gateway.execute_semantic_navigation({})

    assert sent[0][1] == "semantic"
    assert len(sent[0][0]) == 1
    assert sent[0][0][0]["x"] == 2.0


def test_semantic_proximity_task_waits_for_the_global_costmap():
    gateway = gateway_stub()
    result = gateway.receive_semantic_route(
        {
            "semantic": semantic_document(
                avoid_node_ids=["place_009"],
                avoidance_zones=[
                    {
                        "selector": "place_009",
                        "x": 4.0,
                        "y": 5.0,
                        "influence_radius_m": 2.0,
                        "decay_length_m": 0.5,
                    }
                ],
                costmap_policy={
                    "semantic_route_preference_enabled": True,
                    "route_corridor_model": "wide_free_core_additive_outside",
                    "route_corridor_half_width_m": 2.0,
                    "route_corridor_transition_width_m": 1.0,
                    "route_corridor_outside_cost": 100,
                    "semantic_avoidance_is_lethal": False,
                    "semantic_proximity_cost_model": "exponential_additive",
                    "requires_nav2_proximity_layer": True,
                },
                statistics={
                    "destination_count": 1,
                    "avoidance_zone_count": 1,
                    "path_planner": "nav2_smac_hybrid",
                    "route_navigation_places": 2,
                    "drivable_route_length_m": 2.2,
                    "search_algorithm": "astar_euclidean_admissible",
                    "astar_cost_m": 2.2,
                },
            )
        }
    )

    assert result["semantic"]["costmap_ready"] is False
    assert gateway._semantic_costmap_source["avoidance_zones"][0]["x"] == 4.0
    assert gateway._semantic_costmap_source["verification_points"][0][
        "minimum_cost"
    ] == 180


@pytest.mark.parametrize(
    "semantic",
    [
        semantic_document(provider="ollama_local"),
        semantic_document(model="qwen3.8:27b"),
        semantic_document(map_id="other_map"),
        semantic_document(graph_sha256="invalid"),
        semantic_document(destination_poses=[]),
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
            statistics={
                "destination_count": 2,
                "avoidance_zone_count": 0,
                "path_planner": "nav2_smac_hybrid",
            }
        ),
        semantic_document(
            statistics={
                "destination_count": 1,
                "avoidance_zone_count": 0,
                "path_planner": "dijkstra",
            }
        ),
    ],
)
def test_rdk_rejects_untrusted_or_stale_semantic_tasks(semantic):
    with pytest.raises(ApiError):
        gateway_stub().receive_semantic_route({"semantic": semantic})
