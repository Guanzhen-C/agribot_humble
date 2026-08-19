"""Build a bounded exponential semantic proximity-cost map for Nav2."""

from __future__ import annotations

import math

import numpy as np

from .catalog import GridData


class RouteCostmapError(RuntimeError):
    pass


def _finite(value, description: str) -> float:
    if isinstance(value, bool):
        raise RouteCostmapError(f"{description} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RouteCostmapError(f"{description} must be finite") from error
    if not math.isfinite(result):
        raise RouteCostmapError(f"{description} must be finite")
    return result


def world_to_grid(grid: GridData, x: float, y: float) -> tuple[int, int]:
    dx = _finite(x, "world x") - grid.origin_x
    dy = _finite(y, "world y") - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        int(math.floor(local_x / grid.resolution)),
        int(math.floor(local_y / grid.resolution)),
    )


def build_proximity_costmap(grid: GridData, avoidance_zones: list[dict]) -> np.ndarray:
    """Return normalized 0..100 costs for semantic avoidance points.

    The Nav2 layer scales these source values below lethal cost.  Each zone uses
    ``100 * exp(-distance / decay_length_m)`` inside its finite influence radius;
    overlapping zones use the maximum cost rather than accumulating.
    """
    if (
        grid.width <= 0
        or grid.height <= 0
        or not math.isfinite(grid.resolution)
        or grid.resolution <= 0.0
    ):
        raise RouteCostmapError("map geometry is invalid")
    if not isinstance(avoidance_zones, list):
        raise RouteCostmapError("semantic avoidance zones must be a list")
    mask = neutral_route_costmap(grid)
    selectors = set()
    for zone in avoidance_zones:
        if not isinstance(zone, dict):
            raise RouteCostmapError("semantic avoidance zone is invalid")
        selector = str(zone.get("selector", ""))
        if not selector or selector in selectors:
            raise RouteCostmapError("semantic avoidance selectors must be unique")
        selectors.add(selector)
        influence_radius = _finite(
            zone.get("influence_radius_m"), "avoidance influence radius"
        )
        decay_length = _finite(
            zone.get("decay_length_m"), "avoidance decay length"
        )
        if influence_radius <= 0.0 or decay_length <= 0.0:
            raise RouteCostmapError(
                "semantic avoidance influence radius and decay length must be positive"
            )
        center_x = _finite(zone.get("x"), "avoidance x")
        center_y = _finite(zone.get("y"), "avoidance y")

        # Work in the map-local frame so rotated OccupancyGrid origins remain valid.
        center_column, center_row = world_to_grid(grid, center_x, center_y)
        radius_cells = int(math.ceil(influence_radius / grid.resolution)) + 1
        column_min = max(0, center_column - radius_cells)
        column_max = min(grid.width, center_column + radius_cells + 1)
        row_min = max(0, center_row - radius_cells)
        row_max = min(grid.height, center_row + radius_cells + 1)
        if column_min >= column_max or row_min >= row_max:
            continue

        columns = np.arange(column_min, column_max, dtype=np.float64) + 0.5
        rows = np.arange(row_min, row_max, dtype=np.float64) + 0.5
        local_x = columns[None, :] * grid.resolution
        local_y = rows[:, None] * grid.resolution
        cosine = math.cos(grid.origin_yaw)
        sine = math.sin(grid.origin_yaw)
        world_x = grid.origin_x + cosine * local_x - sine * local_y
        world_y = grid.origin_y + sine * local_x + cosine * local_y
        distance = np.hypot(world_x - center_x, world_y - center_y)
        costs = np.where(
            distance <= influence_radius,
            100.0 * np.exp(-distance / decay_length),
            0.0,
        )
        normalized = np.rint(costs).astype(np.uint8)
        current = mask[row_min:row_max, column_min:column_max]
        np.maximum(current, normalized, out=current)
    return mask


def neutral_route_costmap(grid: GridData) -> np.ndarray:
    if grid.width <= 0 or grid.height <= 0:
        raise RouteCostmapError("map geometry is invalid")
    return np.zeros((grid.height, grid.width), dtype=np.uint8)
