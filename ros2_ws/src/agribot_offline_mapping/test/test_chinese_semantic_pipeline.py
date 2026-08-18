import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


def load_script(name):
    path = SCRIPTS / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LOCALIZE = load_script("localize_semantic_landmarks_zh.py")
UPSERT = load_script("upsert_semantic_landmarks_neo4j.py")


def test_landmark_localization_is_chinese_ordered_and_resumable():
    semantics = {
        "objects": [
            {
                "caption": "a paved road",
                "legacy_semantickitti_tag": "road",
                "num_detections": 100,
            },
            {
                "caption": "a white building",
                "legacy_semantickitti_tag": "building",
                "num_detections": 20,
            },
            {
                "caption": "a white building",
                "legacy_semantickitti_tag": "building",
                "num_detections": 30,
            },
            {
                "caption": "an uncertain object",
                "legacy_semantickitti_tag": "other",
                "num_detections": 2,
            },
        ]
    }
    pairs = LOCALIZE.collect_pairs(semantics, 10, {"road", "parking"})
    assert pairs == [("a white building", "building")]
    response = {
        "translations": [
            {
                "source_caption": "a white building",
                "source_category": "building",
                "caption_zh": "白色建筑",
                "category_zh": "建筑",
            }
        ]
    }
    assert LOCALIZE.validate_translations(response, pairs) == response["translations"]
    request = LOCALIZE.translation_request("qwen3.7-flash", pairs)
    prompt = request["messages"][0]["content"]
    assert "可检索的中文" in prompt
    assert request["response_format"] == {"type": "json_object"}

    invalid = json.loads(json.dumps(response, ensure_ascii=False))
    invalid["translations"][0]["caption_zh"] = "white building"
    with pytest.raises(LOCALIZE.LandmarkLocalizationError, match="must be Chinese"):
        LOCALIZE.validate_translations(invalid, pairs)


def test_incremental_chinese_landmark_is_attached_to_nearest_place(tmp_path):
    source = tmp_path / "new_landmarks.json"
    source.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "language": "zh-CN",
                "landmarks": [
                    {
                        "id": "landmark_zh_north_gate",
                        "caption": "园区北门",
                        "category": "入口",
                        "position": {"x": 9.0, "y": 0.0, "z": 1.0},
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    graph = {
        "places": [
            {"id": "place_000", "position": {"x": 0.0, "y": 0.0}},
            {"id": "place_001", "position": {"x": 10.0, "y": 0.0}},
        ]
    }
    resolved = UPSERT.load_incremental_landmarks(source, graph, 5.0)
    assert resolved[0]["nearest_place"] == "place_001"
    assert resolved[0]["distance_to_place_m"] == 1.0
    assert resolved[0]["caption"] == "园区北门"

    document = json.loads(source.read_text(encoding="utf-8"))
    document["landmarks"][0]["caption"] = "north gate"
    source.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(UPSERT.Neo4jGraphError, match="must be Chinese"):
        UPSERT.load_incremental_landmarks(source, graph, 5.0)
