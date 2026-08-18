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
    / "build_map_semantic_navigation_graph.py"
)
SPEC = importlib.util.spec_from_file_location(
    "build_map_semantic_navigation_graph", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_map(tmp_path, name, image):
    image_path = tmp_path / "{}.pgm".format(name)
    yaml_path = tmp_path / "{}.yaml".format(name)
    assert cv2.imwrite(str(image_path), image)
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": image_path.name,
                "resolution": 0.1,
                "origin": [-20.0, -20.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
                "mode": "trinary",
            }
        ),
        encoding="utf-8",
    )
    return yaml_path


def arguments(map_yaml, boundary_yaml, semantics, localization, output):
    return SimpleNamespace(
        map_yaml=map_yaml,
        road_boundary_map_yaml=boundary_yaml,
        semantic_metadata=semantics,
        landmark_localization=localization,
        output=output,
        start_position=[0.0, -11.5],
        start_yaw_deg=0.0,
        place_spacing=5.0,
        boundary_sample_spacing=0.2,
        boundary_smoothing=0.8,
        centerline_sample_spacing=0.2,
        centerline_smoothing=0.5,
        safety_smoothing=0.2,
        safety_iterations=2,
        middle_band_pixels=1.5,
        minimum_boundary_pixels=100,
        minimum_centerline_clearance=0.5,
        safety_clearance_margin=0.15,
        maximum_centerline_snap=1.0,
        minimum_landmark_detections=10,
        landmark_attach_radius=20.0,
        road_support_distance=2.0,
        drivable_semantic_tags=["road", "parking"],
        maximum_place_summaries=5,
    )


def test_map_boundaries_create_smooth_uniform_chinese_topology(tmp_path):
    image = np.full((400, 400), 254, dtype=np.uint8)
    cv2.circle(image, (200, 200), 150, 0, 1)
    cv2.circle(image, (200, 200), 80, 0, 1)
    map_yaml = write_map(tmp_path, "map", image)
    boundary_yaml = write_map(tmp_path, "boundaries", image)

    semantics = tmp_path / "semantics.json"
    semantics.write_text(
        json.dumps(
            {
                "frame_id": "map",
                "objects": [
                    {
                        "id": 1,
                        "caption": "a paved road",
                        "legacy_semantickitti_tag": "road",
                        "num_detections": 50,
                        "center": [0.0, 0.0, 0.0],
                        "extent": [40.0, 40.0, 0.1],
                    },
                    {
                        "id": 2,
                        "caption": "a white building",
                        "legacy_semantickitti_tag": "building",
                        "num_detections": 30,
                        "caption_consensus_ratio": 0.9,
                        "center": [11.0, 0.0, 1.0],
                        "extent": [2.0, 2.0, 2.0],
                    },
                    {
                        "id": 3,
                        "caption": "a blue bicycle",
                        "legacy_semantickitti_tag": "bicycle",
                        "num_detections": 20,
                        "caption_consensus_ratio": 0.8,
                        "center": [-11.0, 0.0, 0.5],
                        "extent": [1.0, 1.0, 1.0],
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    localization = tmp_path / "landmarks_zh.json"
    localization.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "language": "zh-CN",
                "semantic_metadata_sha256": MODULE.file_sha256(semantics),
                "translations": [
                    {
                        "source_caption": "a white building",
                        "source_category": "building",
                        "caption_zh": "白色建筑",
                        "category_zh": "建筑",
                    },
                    {
                        "source_caption": "a blue bicycle",
                        "source_category": "bicycle",
                        "caption_zh": "蓝色自行车",
                        "category_zh": "自行车",
                    },
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    graph = MODULE.build_graph(
        arguments(map_yaml, boundary_yaml, semantics, localization, tmp_path / "graph.json")
    )

    assert graph["language"] == "zh-CN"
    assert graph["source"]["topology_source"] == "two_dimensional_map_road_centerline"
    assert "trajectory_poses" not in graph["source"]
    assert graph["statistics"]["unsafe_connections"] == 0
    assert graph["statistics"]["places"] >= 12
    assert graph["statistics"]["landmarks"] == 2
    assert graph["statistics"]["drivable_connections"] == len(graph["places"])
    assert graph["statistics"]["semantic_associations"] == len(graph["landmarks"])
    assert all(place["name"].startswith("道路地点") for place in graph["places"])
    assert all(MODULE.contains_chinese(item["caption"]) for item in graph["landmarks"])
    assert all(MODULE.contains_chinese(item["category"]) for item in graph["landmarks"])

    drivable = [
        item for item in graph["connections"] if item["kind"] == "drivable"
    ]
    assert all(item["evidence"] == "two_dimensional_map_road_centerline" for item in drivable)
    assert all(len(item["centerline"]) > 2 for item in drivable)
    assert np.std([item["length_m"] for item in drivable]) < 1e-9
    assert drivable[-1]["target"] == "place_000"

    place_positions = {
        item["id"]: np.asarray(
            [item["position"]["x"], item["position"]["y"]]
        )
        for item in graph["places"]
    }
    for landmark in graph["landmarks"]:
        position = np.asarray(
            [landmark["position"]["x"], landmark["position"]["y"]]
        )
        expected = min(
            place_positions,
            key=lambda place_id: np.linalg.norm(place_positions[place_id] - position),
        )
        assert landmark["nearest_place"] == expected
        associations = [
            item
            for item in graph["connections"]
            if item["kind"] == "semantic_association"
            and item["target"] == landmark["id"]
        ]
        assert len(associations) == 1
        assert associations[0]["source"] == expected


def test_map_topology_accepts_local_model_instances_without_second_translation(tmp_path):
    image = np.full((400, 400), 254, dtype=np.uint8)
    cv2.circle(image, (200, 200), 150, 0, 1)
    cv2.circle(image, (200, 200), 80, 0, 1)
    map_yaml = write_map(tmp_path, "map", image)
    boundary_yaml = write_map(tmp_path, "boundaries", image)
    semantics = tmp_path / "semantics_zh.json"
    vector = [1.0] + [0.0] * 63
    search_text = "白色建筑入口；类别：建筑入口"
    semantics.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "frame_id": "map",
                "language": "zh-CN",
                "objects": [
                    {
                        "id": 1,
                        "caption_zh": "环形道路",
                        "category_zh": "道路",
                        "is_drivable_surface": True,
                        "landmark_usable": False,
                        "is_static": True,
                        "semantic_confidence": 0.99,
                        "num_detections": 50,
                        "center": [0.0, 0.0, 0.0],
                        "extent": [40.0, 40.0, 0.1],
                    },
                    {
                        "id": 2,
                        "caption_zh": "白色建筑入口",
                        "category_zh": "建筑入口",
                        "source_caption": "white entrance",
                        "source_category": "building",
                        "is_drivable_surface": False,
                        "landmark_usable": True,
                        "is_static": True,
                        "semantic_confidence": 0.93,
                        "semantic_source": "qwen3.8:27b",
                        "visible_evidence": ["白色门框"],
                        "num_detections": 30,
                        "center": [11.0, 0.0, 1.0],
                        "extent": [2.0, 2.0, 2.0],
                        "semantic_embedding": {
                            "provider": "ollama_local",
                            "model": "qwen3-embedding:8b",
                            "dimensions": 64,
                            "text_sha256": hashlib.sha256(
                                search_text.encode("utf-8")
                            ).hexdigest(),
                            "vector": vector,
                        },
                    },
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    options = arguments(
        map_yaml, boundary_yaml, semantics, None, tmp_path / "graph.json"
    )

    graph = MODULE.build_graph(options)

    assert graph["parameters"]["semantic_mode"] == "ollama_chinese_instances"
    assert graph["statistics"]["road_semantic_objects"] == 1
    assert graph["statistics"]["landmarks"] == 1
    assert graph["landmarks"][0]["caption"] == "白色建筑入口"
    assert graph["landmarks"][0]["semantic_embedding"]["vector"] == vector
    assert "landmark_localization" not in graph["source"]
