import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "refine_semantic_navigation_graph.py"
)
SPEC = importlib.util.spec_from_file_location(
    "refine_semantic_navigation_graph", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_map(tmp_path):
    image = np.full((200, 200), 255, dtype=np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    image_path = tmp_path / "map.pgm"
    assert cv2.imwrite(str(image_path), image)
    map_path = tmp_path / "map.yaml"
    map_path.write_text(
        yaml.safe_dump(
            {
                "image": image_path.name,
                "mode": "trinary",
                "resolution": 0.1,
                "origin": [-10.0, -10.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        encoding="utf-8",
    )
    return map_path


def write_contracts(tmp_path):
    map_path = write_map(tmp_path)
    metadata_path = tmp_path / "semantics.json"
    metadata_path.write_text(
        json.dumps(
            {
                "frame_id": "map",
                "objects": [
                    {
                        "id": 1,
                        "caption": "road",
                        "legacy_semantickitti_tag": "road",
                        "center": [0.0, 0.0, 0.0],
                        "extent": [20.0, 20.0, 0.1],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    graph_path = tmp_path / "recorded_graph.json"
    graph = {
        "schema_version": 3,
        "frame_id": "map",
        "source": {
            "map_yaml": str(map_path),
            "map_image": str(tmp_path / "map.pgm"),
            "semantic_metadata": str(metadata_path),
            "sha256": {
                "map_yaml": sha256(map_path),
                "map_image": sha256(tmp_path / "map.pgm"),
                "semantic_metadata": sha256(metadata_path),
            },
        },
        "parameters": {
            "minimum_edge_clearance_m": 0.2,
            "road_support_distance_m": 2.0,
            "drivable_semantic_tags": ["road", "parking"],
        },
        "statistics": {},
        "places": [
            {
                "id": "place_000",
                "position": {"x": -4.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 2.0,
                "road_semantic_ids": [1],
                "landmark_ids": [],
            },
            {
                "id": "place_001",
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 2.0,
                "road_semantic_ids": [1],
                "landmark_ids": ["landmark_0001"],
            },
            {
                "id": "place_002",
                "position": {"x": 4.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 2.0,
                "road_semantic_ids": [1],
                "landmark_ids": [],
            },
        ],
        "landmarks": [
            {
                "id": "landmark_0001",
                "position": {"x": 0.0, "y": 2.0, "z": 1.0},
                "caption": "building",
                "num_detections": 20,
                "nearest_place": "place_001",
                "distance_to_place_m": 2.0,
            }
        ],
        "connections": [
            {
                "id": "connection_000",
                "kind": "drivable",
                "source": "place_000",
                "target": "place_001",
                "length_m": 4.0,
                "minimum_clearance_m": 2.0,
                "road_semantic_coverage_ratio": 1.0,
                "bidirectional": True,
                "executable": True,
                "evidence": "recorded_vehicle_trajectory",
            },
            {
                "id": "connection_001",
                "kind": "drivable",
                "source": "place_001",
                "target": "place_002",
                "length_m": 4.0,
                "minimum_clearance_m": 2.0,
                "road_semantic_coverage_ratio": 1.0,
                "bidirectional": True,
                "executable": True,
                "evidence": "recorded_vehicle_trajectory",
            },
            {
                "id": "landmark_connection_0001",
                "kind": "semantic_association",
                "source": "place_001",
                "target": "landmark_0001",
                "length_m": 2.0,
                "bidirectional": True,
                "executable": False,
                "evidence": "nearest_place_semantic_association",
            },
        ],
    }
    graph_path.write_text(json.dumps(graph), encoding="utf-8")
    route_path = tmp_path / "anchor_route.json"
    route_path.write_text(
        json.dumps(
            {
                "schema_version": 3,
                "graph_sha256": sha256(graph_path),
            }
        ),
        encoding="utf-8",
    )
    reference_path = tmp_path / "smac_reference.json"
    x_values = np.linspace(-4.0, 4.0, 17)
    y_values = np.sin(np.linspace(0.0, math.pi, 17))
    reference_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "frame_id": "map",
                "planner_id": "GridBased",
                "route_id": "reference_test",
                "route_waypoint_mode": "requested_stops",
                "source": {
                    "map_yaml": str(map_path),
                    "map_yaml_sha256": sha256(map_path),
                    "semantic_route": str(route_path),
                    "semantic_route_sha256": sha256(route_path),
                },
                "poses": [
                    {
                        "index": index,
                        "position": {"x": float(x), "y": float(y), "z": 0.0},
                        "yaw": 0.0,
                    }
                    for index, (x, y) in enumerate(zip(x_values, y_values))
                ],
            }
        ),
        encoding="utf-8",
    )
    return map_path, graph_path, reference_path


def test_refines_recorded_places_onto_planner_certified_path(tmp_path):
    map_path, graph_path, reference_path = write_contracts(tmp_path)
    arguments = SimpleNamespace(
        graph=graph_path,
        map_yaml=map_path,
        reference_path=reference_path,
        anchors=["place_000", "place_002"],
        output=tmp_path / "refined_graph.json",
        maximum_anchor_fit_distance=0.25,
        minimum_topology_clearance=0.5,
        place_landmark_radius=8.0,
        maximum_place_landmarks=8,
    )

    refined = MODULE.refine_graph(arguments)

    middle = next(place for place in refined["places"] if place["id"] == "place_001")
    assert middle["position"]["x"] == 0.0
    assert middle["position"]["y"] == 1.0
    assert middle["topology_evidence"] == "smac_planner_certified_reference"
    edges = [
        edge for edge in refined["connections"] if edge["kind"] == "drivable"
    ]
    assert all(edge["planner_certified"] is True for edge in edges)
    assert all(edge["evidence"] == "smac_planner_certified_reference" for edge in edges)
    assert refined["statistics"]["planner_certified_places"] == 3
    assert refined["planner_certification"]["corridor_places"] == [
        "place_000",
        "place_001",
        "place_002",
    ]
    landmark = refined["landmarks"][0]
    assert landmark["nearest_place"] == "place_001"
    assert landmark["distance_to_place_m"] == 1.0
