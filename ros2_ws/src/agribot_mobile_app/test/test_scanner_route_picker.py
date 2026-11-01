import importlib.util
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location("scanner_route_picker", Path(__file__).parents[1] / "tools/select_scanner_route_points.py")
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def make_store(tmp_path):
    base = tmp_path / "map"
    base.with_suffix(".yaml").write_text("resolution: 0.05\norigin: [0, 0, 0]\n")
    (tmp_path / "map_georeference.yaml").write_text("map: {id: map}\n")
    reference = tmp_path / "reference.csv"
    reference.write_text("capture_timestamp_s,map_x_m,map_y_m,map_z_m\n1,0,0,2\n2,1,0,3\n3,1,1,4\n4,0,1,5\n")
    return module.SelectionStore(base, reference, tmp_path / "manual.json")


def test_manual_snap_uses_exact_reference_and_keeps_user_heading(tmp_path):
    store = make_store(tmp_path)
    before = store.reference.copy()
    index, _ = store.append(0.9, 0.05, -1.2)
    assert index == 1
    assert store.poses == [{"x": 1.0, "y": 0.0, "yaw": -1.2}]
    assert (store.reference == before).all()
    assert len(store.reference) == 4
    reloaded = module.SelectionStore(store.base, store.reference_csv, store.output)
    assert reloaded.poses == store.poses


def test_reject_far_click_and_preserve_state_on_rejection(tmp_path):
    store = make_store(tmp_path)
    with pytest.raises(ValueError, match="closer"):
        store.append(20, 20, 0)
    assert store.poses == []
    assert not store.output.exists()


def test_undo_clear_and_original_endpoint_is_not_moved(tmp_path):
    store = make_store(tmp_path)
    store.append(0, 0, 0)
    store.append(0, 1, 1.57)
    assert store.poses[-1]["x"] == 0 and store.poses[-1]["y"] == 1
    store.undo()
    assert len(store.poses) == 1
    store.clear()
    assert store.poses == []
    assert store.output.exists()


def test_changed_reference_does_not_silently_reuse_picks(tmp_path):
    store = make_store(tmp_path)
    store.append(0, 0, 0)
    store.reference_csv.write_text(store.reference_csv.read_text().replace("4,0,1,5", "4,0,2,5"))
    with pytest.raises(ValueError, match="another map/reference"):
        module.SelectionStore(store.base, store.reference_csv, store.output)


def test_extend_beyond_110_preserves_existing_100_points(tmp_path):
    store = make_store(tmp_path)
    for index in range(100):
        store.append(index % 2, 0, 0.1)
    before = [dict(pose) for pose in store.poses]
    store = module.SelectionStore(store.base, store.reference_csv, store.output)
    for index in range(100, 110):
        store.append(index % 2, 0, 0.2)
    assert store.poses[:100] == before
    assert len(store.poses) == 110
    store.append(0, 0, 0)
    assert len(store.poses) == 111
    reloaded = module.SelectionStore(store.base, store.reference_csv, store.output)
    assert reloaded.poses == store.poses
