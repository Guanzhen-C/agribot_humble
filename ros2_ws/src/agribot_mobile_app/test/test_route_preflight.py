import math

import numpy as np
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.route_preflight import footprint_cells, nearest_free_endpoint


def make_grid():
    pixels = np.zeros((200, 200), dtype=np.int8)
    pixels[90:110, 100:110] = 100
    return GridData(200, 200, 0.05, -5, -5, 0, pixels.tobytes())


def test_full_rectangle_not_just_center_is_checked():
    grid = make_grid()
    footprint = [[0.7, 0.53], [0.7, -0.53], [-0.7, -0.53], [-0.7, 0.53]]
    pose = {"x": -0.35, "y": 0.0, "yaw": 0.0}
    assert footprint_cells(grid, pose, footprint)["occupied_cells"] > 0
    candidate, shift = nearest_free_endpoint(grid, pose, footprint, radius=2)
    assert 0 < shift <= 2
    assert footprint_cells(grid, candidate, footprint, margin=0.1)["occupied_cells"] == 0
    assert pose["x"] == -0.35


def test_unknown_or_outside_is_not_a_known_free_endpoint():
    grid = GridData(40, 40, 0.1, -2, -2, 0, bytes([255]) * 1600)
    footprint = [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
    with pytest.raises(ValueError, match="No known-free"):
        nearest_free_endpoint(grid, {"x": 0, "y": 0, "yaw": 0}, footprint, radius=0.5)
    assert footprint_cells(grid, {"x": -1.9, "y": 0, "yaw": 0}, footprint)["out_of_bounds"]


def test_grid_origin_rotation_is_applied():
    grid = make_grid()
    rotated = GridData(grid.width, grid.height, grid.resolution, 5, -5, math.pi / 2, grid.data)
    footprint = [[0.7, 0.53], [0.7, -0.53], [-0.7, -0.53], [-0.7, 0.53]]
    original = footprint_cells(grid, {"x": -0.35, "y": 0, "yaw": 0}, footprint)
    turned = footprint_cells(rotated, {"x": 0, "y": -0.35, "yaw": math.pi / 2}, footprint)
    assert original["occupied_cells"] == turned["occupied_cells"]
