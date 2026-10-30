import importlib.util
from pathlib import Path

import numpy as np
import pytest


SPEC = importlib.util.spec_from_file_location(
    "clean_grid", Path(__file__).parents[1]/"scripts/clean_navigation_grid_regions.py")
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def scene():
    x, y = np.meshgrid(np.arange(1, 9, 0.1), np.arange(1, 9, 0.1))
    points = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    cells = np.full((100, 100), 205, dtype=np.uint8)
    cells[10:90, 10:90] = 254
    region = {"name": "reviewed", "kind": "polygon", "coordinates": [[1,1],[9,1],[9,9],[1,9]]}
    return points, cells, region


def test_scattered_road_noise_is_cleared_but_outside_is_unchanged():
    points, cells, region = scene()
    cells[50, 50] = 0
    cells[1, 1] = 0
    result, changed, _, reports = module.clean_grid(cells, points, np.array([0,0]), 0.1, [region])
    assert result[50, 50] == 254
    assert result[1, 1] == 0
    assert np.array_equal(result[~changed], cells[~changed])
    assert "skipped" not in reports[0]


def test_supported_vertical_pole_is_preserved():
    points, cells, region = scene()
    pole = np.column_stack([np.full(20, 5.05), np.full(20, 5.05), np.linspace(.5, 2, 20)])
    points = np.vstack([points, pole])
    cells[50, 50] = 0
    result, _, protected, _ = module.clean_grid(cells, points, np.array([0,0]), 0.1, [region])
    assert protected[50, 50]
    assert result[50, 50] == 0


def test_lower_drainage_layer_is_not_erased():
    points, cells, region = scene()
    low = ((points[:,0]>4)&(points[:,0]<6)&(points[:,1]>4)&(points[:,1]<6))
    points[low, 2] = -1.0
    cells[40:61,40:61] = 0
    result, _, protected, _ = module.clean_grid(cells, points, np.array([0,0]), 0.1, [region])
    assert protected[50, 50]
    assert result[50, 50] == 0


def test_no_unobserved_hole_is_implicitly_filled():
    points, cells, region = scene()
    points = points[np.linalg.norm(points[:,:2]-[5,5],axis=1)>1]
    cells[48:53,48:53] = 205
    result, _, _, _ = module.clean_grid(cells, points, np.array([0,0]), 0.1, [region])
    assert result[50,50] == 205


def test_narrow_or_unobserved_region_fails_closed():
    points, cells, _ = scene()
    region = {"name": "empty", "kind": "polygon", "coordinates": [[30,30],[31,30],[31,31],[30,31]]}
    result, _, _, reports = module.clean_grid(cells, points, np.array([0,0]), 0.1, [region])
    np.testing.assert_array_equal(result,cells)
    assert reports[0]["skipped"] == "outside map"


def test_corridor_rasterization_uses_map_coordinates():
    region = {"kind": "corridor", "coordinates": [[12,22],[16,22]], "width_m": 1.0}
    mask = module.region_mask((100,100),np.array([10,20]),.1,region)
    assert mask[20,40]
    assert not mask[40,20]


def test_inconsistent_ground_levels_fail_closed():
    points, _, _ = scene()
    points[:,2] = np.sin(points[:,0])*0.65
    with pytest.raises(ValueError, match="surface"):
        module.fit_floor(points)


def test_reprocessing_does_not_replace_later_manual_edits(tmp_path):
    import json
    base = tmp_path/"map"
    for suffix in [".pcd", ".yaml", "_georeference.yaml"]:
        Path(str(base)+suffix).write_bytes(b"unchanged")
    current = Path(str(base)+".pgm")
    current.write_bytes(b"user manual edits")
    Path(str(base)+"_before_road_cleanup.pgm").write_bytes(b"original")
    Path(str(base)+"_road_cleanup.json").write_text(json.dumps({"output_pgm_sha256": "different"}))
    annotations = tmp_path/"regions.json"
    annotations.write_text(json.dumps({"map_id": "map"}))
    with pytest.raises(ValueError, match="edited since cleanup"):
        module.process(base,annotations,from_backup=True)
    assert current.read_bytes() == b"user manual edits"
