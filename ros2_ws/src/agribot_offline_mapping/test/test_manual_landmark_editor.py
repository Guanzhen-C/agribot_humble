import importlib.util
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
import yaml
from PyQt5.QtWidgets import QApplication, QDialog


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "manual_landmark_editor.py"
SPEC = importlib.util.spec_from_file_location("manual_landmark_editor", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_store_creates_a_map_specific_draft_without_graph_fields(tmp_path):
    output = tmp_path / "manual_landmarks.yaml"
    store = MODULE.ManualLandmarkStore(output, "map_test")

    first = store.add("东侧充电站入口", "充电设施", 12.4, -3.8, 0.0)
    second = store.add("灰色平开门", "建筑入口", 8.1, 2.3, 0.0)

    assert first["id"] == "landmark_manual_0001"
    assert second["id"] == "landmark_manual_0002"
    document = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert document["schema_version"] == 1
    assert document["map_id"] == "map_test"
    assert document["frame_id"] == "map"
    assert document["landmarks"][0] == {
        "id": "landmark_manual_0001",
        "name": "东侧充电站入口",
        "category": "充电设施",
        "position": {"x": 12.4, "y": -3.8, "z": 0.0},
        "source": "manual",
    }
    serialized = output.read_text(encoding="utf-8")
    for forbidden in (
        "semantic_embedding",
        "nearest_place",
        "graph_sha256",
        "neo4j",
    ):
        assert forbidden not in serialized


def test_store_reloads_existing_entries_and_keeps_identifiers_stable(tmp_path):
    output = tmp_path / "manual_landmarks.yaml"
    MODULE.ManualLandmarkStore(output, "map_test").add(
        "第一个地标", "建筑", 1.0, 2.0
    )

    reloaded = MODULE.ManualLandmarkStore(output, "map_test")
    landmark = reloaded.add("第二个地标", "道路设施", 3.0, 4.0)

    assert landmark["id"] == "landmark_manual_0002"
    assert [item["name"] for item in reloaded.landmarks()] == [
        "第一个地标",
        "第二个地标",
    ]


def test_store_rejects_a_different_map_and_duplicate_click(tmp_path):
    output = tmp_path / "manual_landmarks.yaml"
    store = MODULE.ManualLandmarkStore(output, "map_one")
    store.add("充电位置", "充电设施", 1.0, 2.0)

    with pytest.raises(MODULE.ManualLandmarkError, match="另一张地图"):
        MODULE.ManualLandmarkStore(output, "map_two").landmarks()
    with pytest.raises(MODULE.ManualLandmarkError, match="相同位置"):
        store.add("充电位置", "其他类别", 1.01, 2.01)


def test_store_rejects_malformed_or_unexpected_content(tmp_path):
    output = tmp_path / "manual_landmarks.yaml"
    output.write_text(
        "schema_version: 1\nmap_id: map_test\nframe_id: map\n"
        "landmarks: []\ngraph_sha256: forbidden\n",
        encoding="utf-8",
    )
    with pytest.raises(MODULE.ManualLandmarkError, match="未支持字段"):
        MODULE.ManualLandmarkStore(output, "map_test").landmarks()


def test_dialog_accepts_name_and_category_without_editing_coordinates():
    application = QApplication.instance() or QApplication([])
    dialog = MODULE.LandmarkDialog(12.4, -3.8, 0.0)
    dialog.name_edit.setText("东侧充电站入口")
    dialog.category_edit.setText("充电设施")

    dialog._accept_if_valid()

    assert dialog.result() == QDialog.Accepted
    assert dialog.name_edit.text() == "东侧充电站入口"
    assert dialog.category_edit.text() == "充电设施"
    dialog.deleteLater()
    application.processEvents()


def test_editor_application_stays_alive_after_dialog_closes():
    application = QApplication.instance() or QApplication([])
    application.setQuitOnLastWindowClosed(False)

    dialog = MODULE.LandmarkDialog(1.0, 2.0, 0.0)
    dialog.show()
    application.processEvents()
    dialog.reject()
    application.processEvents()

    assert not application.quitOnLastWindowClosed()
    assert not dialog.isVisible()
    dialog.deleteLater()
