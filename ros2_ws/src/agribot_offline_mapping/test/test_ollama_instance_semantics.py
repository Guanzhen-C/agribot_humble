import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import cv2


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


def load_script(name):
    path = SCRIPTS / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CLIENT = load_script("ollama_semantic_client")
DESCRIBE = load_script("describe_opengraph_instances_ollama")


def test_embedding_client_deduplicates_and_reuses_persistent_cache(tmp_path):
    client = CLIENT.OllamaSemanticClient(cache_path=tmp_path / "cache.sqlite3")
    calls = []

    def request(endpoint, document, _description):
        calls.append((endpoint, document))
        vectors = []
        for index, _text in enumerate(document["input"]):
            vector = [0.0] * 64
            vector[index] = 1.0
            vectors.append(vector)
        return {"embeddings": vectors}

    client._request = request
    first = client.embed_texts(["蓝色入口", "蓝色入口", "白色立柱"], dimensions=64)
    assert len(calls) == 1
    assert calls[0][1]["input"] == ["蓝色入口", "白色立柱"]
    assert first[0] == first[1]

    client._request = lambda *_args: (_ for _ in ()).throw(
        AssertionError("cached text must not call the API")
    )
    second = client.embed_texts(["白色立柱", "蓝色入口"], dimensions=64)
    assert second == [first[2], first[0]]
    client.database.close()


def test_multimodal_response_is_locally_audited_before_landmark_promotion():
    records = [
        {"object_id": 1, "views": [{"crop": "one.jpg"}]},
        {"object_id": 2, "views": [{"crop": "two.jpg"}]},
    ]
    response = {
        "objects": [
            {
                "object_id": 1,
                "caption_zh": "带蓝色门牌的固定入口",
                "category_zh": "建筑入口",
                "landmark_usable": True,
                "is_static": True,
                "is_drivable_surface": False,
                "confidence": 0.93,
                "visible_evidence": ["蓝色门牌", "固定门框"],
                "rejection_reason": "",
            },
            {
                "object_id": 2,
                "caption_zh": "穿深色衣服的行人",
                "category_zh": "人员",
                "landmark_usable": True,
                "is_static": False,
                "is_drivable_surface": False,
                "confidence": 0.96,
                "visible_evidence": ["人体轮廓"],
                "rejection_reason": "可移动目标",
            },
        ]
    }

    result = DESCRIBE.validate_response(response, records, 0.78)

    assert result[0]["landmark_usable"] is True
    assert result[1]["model_landmark_usable"] is True
    assert result[1]["landmark_usable"] is False


def test_movable_category_is_never_promoted_even_when_model_calls_it_static():
    records = [{"object_id": 9, "views": [{"crop": "motorcycle.jpg"}]}]
    response = {
        "objects": [
            {
                "object_id": 9,
                "caption_zh": "路边停放的黑色摩托车",
                "category_zh": "摩托车",
                "landmark_usable": True,
                "is_static": True,
                "is_drivable_surface": False,
                "confidence": 0.95,
                "visible_evidence": ["可见车轮和车把"],
                "rejection_reason": "",
            }
        ]
    }

    result = DESCRIBE.validate_response(response, records, 0.78)

    assert result[0]["model_landmark_usable"] is True
    assert result[0]["landmark_usable"] is False
    assert "可移动目标" in result[0]["rejection_reason"]


def test_movable_word_in_building_caption_does_not_reject_building_landmark():
    description = {
        "caption_zh": "门前停放车辆的浅色建筑立面",
        "category_zh": "建筑立面",
        "landmark_usable": True,
    }

    result = DESCRIBE.apply_local_landmark_policy(description)

    assert result["landmark_usable"] is True


def test_multimodal_request_uses_bounded_image_resolution(tmp_path):
    image = np.zeros((16, 16, 3), dtype=np.uint8)
    path = tmp_path / "crop.jpg"
    assert cv2.imwrite(str(path), image)
    _model, content = DESCRIBE.build_request(
        "qwen3.8:27b",
        [{"object_id": 3, "views": [{"crop": str(path)}]}],
    )
    image_items = [item for item in content if item["type"] == "image_url"]

    assert len(image_items) == 1
    assert image_items[0]["min_pixels"] == 65536
    assert image_items[0]["max_pixels"] >= image_items[0]["min_pixels"]


def test_invalid_batch_is_retried_then_split_into_individual_records(monkeypatch):
    records = [
        {"object_id": 1, "views": [{"crop": "one.jpg"}]},
        {"object_id": 2, "views": [{"crop": "two.jpg"}]},
    ]
    responses = {
        1: {
            "objects": [
                {
                    "object_id": 1,
                    "caption_zh": "蓝色门牌入口",
                    "category_zh": "建筑入口",
                    "landmark_usable": True,
                    "is_static": True,
                    "is_drivable_surface": False,
                    "confidence": 0.9,
                    "visible_evidence": ["蓝色门牌"],
                    "rejection_reason": "",
                }
            ]
        },
        2: {
            "objects": [
                {
                    "object_id": 2,
                    "caption_zh": "白色立柱",
                    "category_zh": "立柱",
                    "landmark_usable": True,
                    "is_static": True,
                    "is_drivable_surface": False,
                    "confidence": 0.9,
                    "visible_evidence": ["白色柱体"],
                    "rejection_reason": "",
                }
            ]
        },
    }

    class Client:
        def chat_json(self, _model, content):
            ids = [
                int(item["text"].split("=", 1)[1].split("，", 1)[0])
                for item in content
                if item["type"] == "text" and item["text"].startswith("object_id=")
            ]
            if len(ids) > 1:
                return {"objects": "invalid"}, {}
            return responses[ids[0]], {}

    monkeypatch.setattr(DESCRIBE, "image_data_url", lambda _path: "data:image/jpeg;base64,AA==")
    result = DESCRIBE.describe_records(Client(), "qwen3.8:27b", records, 0.78, 2)

    assert [item["object_id"] for item in result] == [1, 2]
    assert all(item["landmark_usable"] for item in result)


def test_persistently_invalid_single_instance_is_rejected_and_pipeline_continues(
    monkeypatch,
):
    records = [{"object_id": 7, "views": [{"crop": "bad.jpg"}]}]

    class Client:
        def chat_json(self, _model, _content):
            return {"objects": "invalid"}, {}

    monkeypatch.setattr(
        DESCRIBE, "image_data_url", lambda _path: "data:image/jpeg;base64,AA=="
    )
    result = DESCRIBE.describe_records(
        Client(), "qwen3.8:27b", records, 0.78, 2
    )

    assert result[0]["object_id"] == 7
    assert result[0]["semantic_status"] == "model_response_rejected"
    assert result[0]["landmark_usable"] is False
    assert result[0]["confidence"] == 0.0
    assert "连续2次" in result[0]["rejection_reason"]


def test_projection_and_mask_overlap_select_the_instance_pixels():
    projection = np.asarray(
        [[20.0, 0.0, 10.0, 0.0], [0.0, 20.0, 10.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
    )
    points = np.asarray(
        [[-0.1, -0.1, 2.0], [0.1, -0.1, 2.0], [-0.1, 0.1, 2.0], [0.1, 0.1, 2.0]]
    )
    pixels = DESCRIBE.project_points(points, np.eye(4), projection, 20, 20)
    matching = np.zeros((20, 20), dtype=bool)
    matching[8:13, 8:13] = True
    unrelated = np.zeros((20, 20), dtype=bool)
    unrelated[:3, :3] = True

    selected, count, ratio = DESCRIBE.select_mask(
        pixels, [unrelated, matching]
    )

    assert selected is matching
    assert count == 4
    assert ratio == 1.0


def test_crop_metadata_is_standard_json_serializable(tmp_path):
    image = np.full((20, 20, 3), 127, dtype=np.uint8)
    pixels = np.asarray([[8.2, 8.1], [11.7, 8.2], [8.3, 11.8], [11.6, 11.7]])
    metadata = DESCRIBE.crop_from_observation(
        image, pixels, None, tmp_path / "crop.jpg"
    )

    json.dumps(metadata)
    assert all(type(value) is int for value in metadata["crop_bbox"])
    assert all(type(value) is int for value in metadata["target_bbox"])
    assert cv2.imread(str(tmp_path / "crop.jpg")) is not None


def test_final_metadata_preserves_every_observation_and_embeds_only_landmarks(tmp_path):
    pickle_path = tmp_path / "full_pcd.pkl.gz"
    metadata_path = tmp_path / "semantic_instances.json"
    dataset_root = tmp_path / "dataset"
    dataset_root.mkdir()
    pickle_path.write_bytes(b"source-pickle")
    metadata_path.write_text("{}", encoding="utf-8")
    (dataset_root / "poses.txt").write_text("pose", encoding="utf-8")
    (dataset_root / "calib.txt").write_text("calibration", encoding="utf-8")
    metadata = {
        "frame_id": "map",
        "objects": [
            {
                "id": 0,
                "caption": "a doorway",
                "legacy_semantickitti_tag": "building",
                "point_count": 100,
            },
            {
                "id": 1,
                "caption": "a person",
                "legacy_semantickitti_tag": "person",
                "point_count": 50,
            },
        ],
    }
    descriptions = {
        "0": {
            "object_id": 0,
            "caption_zh": "蓝色门牌入口",
            "category_zh": "建筑入口",
            "model_landmark_usable": True,
            "landmark_usable": True,
            "is_static": True,
            "is_drivable_surface": False,
            "confidence": 0.92,
            "visible_evidence": ["蓝色门牌"],
            "rejection_reason": "",
        }
    }
    vector = [1.0] + [0.0] * 63

    output = DESCRIBE.build_output(
        metadata,
        descriptions,
        {"0": [{"frame_index": 10}]},
        pickle_path,
        metadata_path,
        dataset_root,
        "qwen3.8:27b",
        "qwen3-embedding:8b",
        64,
        0.78,
        {"0": vector},
    )

    assert output["object_count"] == 2
    assert output["landmark_count"] == 1
    assert output["rejected_model_response_count"] == 0
    assert output["objects"][0]["caption"] == "蓝色门牌入口"
    assert output["objects"][0]["source_caption"] == "a doorway"
    assert output["objects"][0]["semantic_embedding"]["vector"] == vector
    assert output["objects"][1]["landmark_usable"] is False
    assert "semantic_embedding" not in output["objects"][1]
