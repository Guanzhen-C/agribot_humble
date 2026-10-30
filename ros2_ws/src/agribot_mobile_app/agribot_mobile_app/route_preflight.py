"""Static-map footprint checks for offline route preparation, not a motion gate."""

import math

import cv2
import numpy as np

from .catalog import GridData


def footprint_cells(grid: GridData, pose: dict, footprint, margin=0.0) -> dict:
    angle = pose["yaw"]
    c, s = math.cos(angle), math.sin(angle)
    points = np.asarray(footprint, dtype=float) @ np.array([[c, s], [-s, c]])
    points += [pose["x"] - grid.origin_x, pose["y"] - grid.origin_y]
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    points = points @ np.array([[c, -s], [s, c]]) / grid.resolution
    extra = int(math.ceil(margin / grid.resolution))
    vertices = np.floor(points).astype(np.int32)
    lower = vertices.min(0) - extra - 1
    upper = vertices.max(0) + extra + 2
    if (lower < 0).any() or upper[0] > grid.width or upper[1] > grid.height:
        return {"out_of_bounds": True, "occupied_cells": 0, "unknown_cells": 0}
    data = np.frombuffer(grid.data, dtype=np.int8).reshape(grid.height, grid.width)
    region = data[lower[1]:upper[1], lower[0]:upper[0]]
    mask = np.zeros(region.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, vertices - lower, 1)
    if extra:
        mask = cv2.dilate(mask, np.ones((2 * extra + 1, 2 * extra + 1), dtype=np.uint8))
    values = region[mask != 0]
    return {"out_of_bounds": False, "occupied_cells": int((values >= 100).sum()), "unknown_cells": int((values < 0).sum())}


def nearest_free_endpoint(grid, pose, footprint, radius=2.0, margin=0.1):
    step = grid.resolution
    extent = int(math.floor(radius / step))
    x, y = np.meshgrid(np.arange(-extent, extent + 1), np.arange(-extent, extent + 1))
    offsets = np.column_stack([x.ravel(), y.ravel()]) * step
    distances = np.linalg.norm(offsets, axis=1)
    for index in np.argsort(distances, kind="stable"):
        if distances[index] > radius:
            break
        candidate = {**pose, "x": pose["x"] + float(offsets[index, 0]), "y": pose["y"] + float(offsets[index, 1])}
        cells = footprint_cells(grid, candidate, footprint, margin)
        if not cells["out_of_bounds"] and not cells["occupied_cells"] and not cells["unknown_cells"]:
            return candidate, float(distances[index])
    raise ValueError(f"No known-free endpoint within {radius:.2f} m; manual goal selection is required")
