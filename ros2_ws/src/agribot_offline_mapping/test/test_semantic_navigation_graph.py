import importlib.util
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "build_semantic_navigation_graph.py"
)
SPEC = importlib.util.spec_from_file_location("build_semantic_navigation_graph", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_map(tmp_path):
    image = np.full((200, 200), 255, dtype=np.uint8)
    image[10:30, 10:30] = 0
    image_path = tmp_path / "map.pgm"
    assert cv2.imwrite(str(image_path), image)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
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
    return yaml_path


def write_poses(path, points):
    rows = []
    for x, y in points:
        transform = np.eye(4, dtype=np.float64)[:3]
        transform[0, 3] = x
        transform[1, 3] = y
        rows.append(transform.reshape(-1))
    np.savetxt(path, np.asarray(rows))


def square_trajectory():
    segments = [
        np.column_stack((np.linspace(-5, 5, 101), np.full(101, -5.0))),
        np.column_stack((np.full(101, 5.0), np.linspace(-5, 5, 101))),
        np.column_stack((np.linspace(5, -5, 101), np.full(101, 5.0))),
        np.column_stack((np.full(101, -5.0), np.linspace(5, -5, 101))),
    ]
    return np.concatenate((segments[0], *(segment[1:] for segment in segments[1:])))


def write_semantics(path):
    document = {
        "frame_id": "map",
        "objects": [
            {
                "id": 1,
                "caption": "a paved road",
                "legacy_semantickitti_tag": "road",
                "num_detections": 50,
                "center": [0.0, 0.0, 0.0],
                "extent": [20.0, 20.0, 0.1],
            },
            {
                "id": 2,
                "caption": "a white building",
                "legacy_semantickitti_tag": "building",
                "num_detections": 20,
                "caption_consensus_ratio": 0.8,
                "center": [4.0, 3.5, 1.0],
                "extent": [2.0, 2.0, 3.0],
            },
        ],
    }
    path.write_text(json.dumps(document), encoding="utf-8")


def arguments(map_yaml, semantics, poses, output):
    return SimpleNamespace(
        map_yaml=map_yaml,
        semantic_metadata=semantics,
        trajectory_poses=poses,
        output=output,
        sample_spacing=1.0,
        merge_radius=0.4,
        minimum_clearance=0.3,
        minimum_edge_clearance=0.2,
        maximum_snap_distance=1.0,
        road_support_distance=1.0,
        drivable_semantic_tags=["road", "parking"],
        minimum_landmark_detections=10,
        landmark_attach_radius=10.0,
        place_spacing=5.0,
        place_landmark_radius=8.0,
        maximum_place_landmarks=4,
    )


def test_map_coordinate_round_trip_and_clearance_snap(tmp_path):
    occupancy_map = MODULE.OccupancyMap(write_map(tmp_path))
    point = np.asarray([2.35, -1.65])
    row, column = occupancy_map.world_to_pixel(point)
    recovered = occupancy_map.pixel_to_world(row, column)

    assert np.linalg.norm(recovered - point) <= occupancy_map.resolution
    snapped, clearance, distance = occupancy_map.snap_to_clearance(
        np.asarray([-8.5, 8.5]), 0.3, 3.0
    )
    assert occupancy_map.clearance_at(snapped) >= 0.3
    assert clearance >= 0.3
    assert distance > 0.0


def test_resampling_and_spatial_loop_merge():
    line = np.asarray([[0.0, 0.0], [5.0, 0.0]])
    sampled, distance = MODULE.resample_polyline(line, 1.0)
    assert len(sampled) == 6
    assert distance[-1] == 5.0

    loop_samples = np.asarray([[0.0, 0.0], [2.0, 0.0], [0.1, 0.0]])
    sample_to_node, clusters = MODULE.merge_nearby_samples(loop_samples, 0.2)
    assert len(clusters) == 2
    assert sample_to_node[0] == sample_to_node[2]


def test_builds_connected_semantic_route_graph(tmp_path):
    map_yaml = write_map(tmp_path)
    poses = tmp_path / "poses.txt"
    semantics = tmp_path / "semantics.json"
    output = tmp_path / "graph.json"
    write_poses(poses, square_trajectory())
    write_semantics(semantics)

    graph = MODULE.build_navigation_graph(
        arguments(map_yaml, semantics, poses, output)
    )

    stats = graph["statistics"]
    assert graph["schema_version"] == 3
    assert stats["unsafe_connections"] == 0
    assert stats["road_semantic_coverage_ratio"] == 1.0
    assert stats["landmarks"] == 1
    assert stats["places"] >= 4
    assert stats["connections"] == len(graph["connections"])
    assert stats["semantic_nodes"] == stats["places"] + stats["landmarks"]
    assert stats["drivable_connections"] == stats["places"]
    assert stats["semantic_associations"] == stats["landmarks"]
    assert stats["connections"] == stats["places"] + stats["landmarks"]
    assert "nodes" not in graph
    assert "edges" not in graph
    assert "place_edges" not in graph
    place_ids = {place["id"] for place in graph["places"]}
    assert all("route_node" not in place for place in graph["places"])
    assert all(
        connection["source"] in place_ids and connection["target"] in place_ids
        for connection in graph["connections"]
        if connection["kind"] == "drivable"
    )
    assert all(
        connection["evidence"] == "contracted_recorded_vehicle_trajectory"
        for connection in graph["connections"]
        if connection["kind"] == "drivable"
    )
    assert all(
        "route_node_ids" not in connection and "route_edge_ids" not in connection
        for connection in graph["connections"]
    )
    assert all("route_node" not in landmark for landmark in graph["landmarks"])
    associations = [
        connection
        for connection in graph["connections"]
        if connection["kind"] == "semantic_association"
    ]
    assert len(associations) == len(graph["landmarks"])
    assert all(connection["executable"] is False for connection in associations)
    assert all(
        sum(landmark["id"] in (edge["source"], edge["target"]) for edge in associations)
        == 1
        for landmark in graph["landmarks"]
    )


def test_applies_digest_bound_chinese_landmark_localization(tmp_path):
    map_yaml = write_map(tmp_path)
    poses = tmp_path / "poses.txt"
    semantics = tmp_path / "semantics.json"
    localization = tmp_path / "landmarks_zh.json"
    output = tmp_path / "graph.json"
    write_poses(poses, square_trajectory())
    write_semantics(semantics)
    localization.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "semantic_metadata_sha256": MODULE.file_sha256(semantics),
                "translations": [
                    {
                        "source_caption": "a white building",
                        "source_category": "building",
                        "caption_zh": "白色建筑物",
                        "category_zh": "建筑",
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    options = arguments(map_yaml, semantics, poses, output)
    options.landmark_localization = localization

    graph = MODULE.build_navigation_graph(options)

    assert graph["language"] == "zh-CN"
    assert graph["landmarks"][0]["caption"] == "白色建筑物"
    assert graph["landmarks"][0]["category"] == "建筑"
    assert graph["landmarks"][0]["source_caption_en"] == "a white building"
    assert graph["source"]["sha256"]["landmark_localization"] == MODULE.file_sha256(
        localization
    )


def test_uses_audited_bailian_chinese_instances_and_preserves_embedding(tmp_path):
    map_yaml = write_map(tmp_path)
    poses = tmp_path / "poses.txt"
    semantics = tmp_path / "semantics_zh.json"
    output = tmp_path / "graph.json"
    write_poses(poses, square_trajectory())
    search_text = "带蓝色门牌的固定入口；类别：建筑入口"
    vector = [1.0] + [0.0] * 63
    semantics.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "frame_id": "map",
                "language": "zh-CN",
                "objects": [
                    {
                        "id": 1,
                        "caption": "铺装道路",
                        "caption_zh": "铺装道路",
                        "category_zh": "道路",
                        "is_drivable_surface": True,
                        "landmark_usable": False,
                        "is_static": True,
                        "semantic_confidence": 0.99,
                        "num_detections": 50,
                        "center": [0.0, 0.0, 0.0],
                        "extent": [20.0, 20.0, 0.1],
                    },
                    {
                        "id": 2,
                        "caption": "带蓝色门牌的固定入口",
                        "caption_zh": "带蓝色门牌的固定入口",
                        "category_zh": "建筑入口",
                        "source_caption": "a fixed entrance with a blue sign",
                        "source_category": "building",
                        "landmark_usable": True,
                        "is_static": True,
                        "is_drivable_surface": False,
                        "semantic_confidence": 0.94,
                        "semantic_source": "qwen3.7-flash",
                        "visible_evidence": ["蓝色门牌", "固定门框"],
                        "num_detections": 20,
                        "caption_consensus_ratio": 0.8,
                        "center": [4.0, 3.5, 1.0],
                        "extent": [2.0, 2.0, 3.0],
                        "semantic_embedding": {
                            "provider": "alibaba_cloud_bailian",
                            "model": "text-embedding-v4",
                            "dimensions": 64,
                            "text_sha256": hashlib.sha256(
                                search_text.encode("utf-8")
                            ).hexdigest(),
                            "vector": vector,
                        },
                    },
                    {
                        "id": 3,
                        "caption": "行人",
                        "caption_zh": "行人",
                        "category_zh": "人员",
                        "landmark_usable": False,
                        "is_static": False,
                        "is_drivable_surface": False,
                        "semantic_confidence": 0.98,
                        "num_detections": 30,
                        "center": [-4.0, 3.5, 1.0],
                        "extent": [1.0, 1.0, 2.0],
                    },
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    graph = MODULE.build_navigation_graph(
        arguments(map_yaml, semantics, poses, output)
    )

    assert graph["language"] == "zh-CN"
    assert graph["parameters"]["semantic_mode"] == "bailian_chinese_instances"
    assert graph["statistics"]["landmarks"] == 1
    assert graph["statistics"]["road_semantic_objects"] == 1
    landmark = graph["landmarks"][0]
    assert landmark["caption"] == "带蓝色门牌的固定入口"
    assert landmark["category"] == "建筑入口"
    assert landmark["semantic_confidence"] == 0.94
    assert landmark["semantic_embedding"]["vector"] == vector
    assert "landmark_localization" not in graph["source"]
