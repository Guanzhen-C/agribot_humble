import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "upsert_semantic_landmarks_neo4j.py"
SPEC = importlib.util.spec_from_file_location(
    "upsert_semantic_landmarks_neo4j", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def graph():
    return {
        "places": [
            {"id": "place_000", "position": {"x": 0.0, "y": 0.0}},
            {"id": "place_001", "position": {"x": 10.0, "y": 0.0}},
        ]
    }


def test_loads_manual_editor_yaml_and_attaches_nearest_place(tmp_path):
    path = tmp_path / "manual.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "outdoor",
                "frame_id": "map",
                "landmarks": [
                    {
                        "id": "landmark_manual_0001",
                        "name": "篮球场",
                        "category": "体育场馆",
                        "position": {"x": 8.0, "y": 1.0, "z": 0.0},
                        "source": "manual",
                    }
                ],
            },
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    landmarks = MODULE.load_incremental_landmarks(
        path, graph(), 20.0, "outdoor"
    )

    assert landmarks[0]["caption"] == "篮球场"
    assert landmarks[0]["nearest_place"] == "place_001"
    assert landmarks[0]["distance_to_place_m"] == pytest.approx(5 ** 0.5)


def test_rejects_manual_landmarks_from_another_map(tmp_path):
    path = tmp_path / "manual.yaml"
    path.write_text(
        "schema_version: 1\nmap_id: indoor\nframe_id: map\nlandmarks: []\n",
        encoding="utf-8",
    )
    with pytest.raises(MODULE.Neo4jGraphError, match="another map"):
        MODULE.load_incremental_landmarks(path, graph(), 20.0, "outdoor")
