import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("capture_route_preset", SCRIPTS / "create_capture_route_preset.py")
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)
sys.path.pop(0)


def test_keypoints_preserve_hairpins_start_end_and_order():
    path = np.array([[0, 0], [0, 3], [0, 8], [1, 8], [1, 3], [1, 0], [2, 0], [2, 8], [0, 8], [0, 0]], dtype=float)
    selected = module.simplify_indices(path, 0.1)
    assert selected == [0, 2, 3, 5, 6, 7, 8, 9]


def test_straight_section_reduction_and_invalid_tolerances():
    path = np.column_stack([np.arange(100), np.zeros(100)])
    assert module.simplify_indices(path, 0.25) == [0, 99]
    for tolerance in [0, -1, float("nan"), float("inf")]:
        with pytest.raises(ValueError):
            module.simplify_indices(path, tolerance)
    with pytest.raises(ValueError):
        module.simplify_indices(np.array([[0, 0], [1, np.nan]]), 0.2)


def test_different_enu_origins_and_map_rotation_use_ecef_not_translation_only():
    pytest.importorskip("pyproj")
    from pyproj import Transformer
    source = module.EnuFrame("EPSG:32650", [115.908, 38.968, 2.0])
    target_ref = {"longitude_deg": 115.907, "latitude_deg": 38.967, "altitude_m": 1.0}
    target = module.EnuFrame("EPSG:32650", [115.907, 38.967, 1.0])
    llh = np.array([[115.908, 38.968, 2.0], [115.909, 38.969, 3.0]])
    positions = source.geodetic(llh)
    converted = module.to_map(positions, source, target_ref, {"rpy": [0, 0, 0], "xyz": [0, 0, 0]})
    np.testing.assert_allclose(converted, target.geodetic(llh), atol=1e-7)
    inverse = Transformer.from_crs(4978, 4979, always_xy=True)
    recovered = np.column_stack(inverse.transform(*(converted @ target.rotation + target.origin_ecef).T))
    np.testing.assert_allclose(recovered, llh, atol=1e-7)
    rotated = module.to_map(positions, source, target_ref, {"rpy": [0, 0, np.pi / 2], "xyz": [2, 3, 4]})
    np.testing.assert_allclose(rotated[:, :2], np.column_stack([-converted[:, 1] + 2, converted[:, 0] + 3]), atol=1e-7)
